import base64
import copy
import hashlib
import io
import json
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

from robot.mac.navigation_advisor import GeminiNavigationAdvisor, route_frame_change
from robot.mac.camera_setup_pool import CameraCloudError, GeminiCameraSetupAdvisor
from robot.mac.route_perception import RouteService
from robot.jetson.navigation.local_planner import LocalRoutePlanner
from robot.jetson.navigation.image_corridors import evaluate_corridor, semantic_route_candidates
from robot.jetson.navigation.image_corridors import ImageCorridor, estimate_floor_horizon
from robot.jetson.mission.occlusion_advice import request_occlusion_advice
from robot.jetson.navigation.navigation_reasoning import OCCLUSION_SCHEMA, validate_occlusion_advice
from scripts.diagnostics.simulate_navigation_reasoning import local_scene, route_advice


def response(value):
    handle = io.BytesIO(json.dumps(value).encode())
    return handle


class NavigationAdvisorTests(unittest.TestCase):
    def test_cloud_polygons_match_local_floor_region_and_keep_near_obstacles(self):
        for floor_row in (30, 66):
            mask = [[y >= floor_row for x in range(120)] for y in range(100)]
            # A chair higher in the image is context; a close object is inside
            # the center region and must remain in both interpretations.
            for y in range(80, 95):
                for x in range(48, 72):mask[y][x] = False
            _, evidence = semantic_route_candidates(mask)
            local = local_scene()
            local['floor_horizon_y'] = estimate_floor_horizon(mask)
            client = Mock()
            GeminiNavigationAdvisor(client).route(b'original image', local)
            args = client.interpret_structured.call_args.args
            self.assertEqual([b'original image'], args[0])
            polygons = json.loads(args[4])['corridors']
            for polygon, measured in zip(polygons, evidence):
                corridor = ImageCorridor(**{k: v for k, v in polygon.items()
                                            if k not in ('id', 'vertices')})
                self.assertEqual(measured, evaluate_corridor(mask, corridor))
                self.assertGreaterEqual(corridor.top_y, .50)
                self.assertEqual(corridor.top_y, polygon['vertices'][0][1])
            self.assertLess(evidence[1].floor_fraction, .95)

    def test_adc_multiframe_schema_transport_and_binding(self):
        frames = [b'\xff\xd8' + b'a' * 100, b'\xff\xd8' + b'b' * 100]
        observation = dict(quality='clear', cause='behind_object', inspect_side='right', evidence='Chair overlaps legs.')
        answer = dict(candidates=[dict(finishReason='STOP', content=dict(parts=[
            dict(text=json.dumps(observation))]))])
        adc = NS(authorize=lambda model: ('https://aiplatform.googleapis.com/test', {}))
        client = GeminiCameraSetupAdvisor(adc=adc)
        with patch('robot.mac.camera_setup_pool.urlopen', return_value=response(answer)) as call:
            result = GeminiNavigationAdvisor(client).occlusion(frames, False)
        payload = json.loads(call.call_args.args[0].data)
        parts = payload['contents'][0]['parts']
        decoded = [base64.b64decode(p['inlineData']['data']) for p in parts if 'inlineData' in p]
        self.assertEqual(frames, decoded)
        self.assertEqual(OCCLUSION_SCHEMA, payload['generationConfig']['responseJsonSchema'])
        self.assertEqual([hashlib.sha256(f).hexdigest() for f in frames], result['frame_sha256'])
        self.assertEqual(observation, result['interpretation'])
        self.assertTrue(result['advisory_only'])

    def test_failure_cooldown_and_busy_admission_never_queue_or_retry(self):
        clock = [10.]
        client = Mock();client.interpret_structured.side_effect = TimeoutError('sensitive provider detail')
        advisor = GeminiNavigationAdvisor(client, clock=lambda: clock[0])
        for _ in range(2):
            with self.assertRaises(CameraCloudError) as error:advisor.route(b'jpeg', local_scene())
            self.assertNotIn('sensitive', str(error.exception))
        self.assertEqual(1, client.interpret_structured.call_count)
        clock[0] = 41.
        advisor.lock.acquire()
        try:
            with self.assertRaises(CameraCloudError):advisor.route(b'jpeg', local_scene())
        finally:advisor.lock.release()
        self.assertEqual(1, client.interpret_structured.call_count)

    def service(self):
        engine = NS(planner=LocalRoutePlanner(maximum_step_m=.05))
        service = RouteService(engine)
        before, fresh = local_scene(), local_scene()
        head = dict(reference_id='ref', position=22)
        before.update(frame_sha256=hashlib.sha256(b'first').hexdigest(), camera_head=head)
        fresh.update(frame_sha256=hashlib.sha256(b'second').hexdigest(), camera_head=dict(head),
                     result_age_seconds=.1, camera_received_at_unix=100.)
        service._local_route = Mock(side_effect=[(before, b'first'), (fresh, b'second')])
        service.navigation_advisor = Mock()
        service.navigation_advisor.route.return_value = dict(provider='gemini', advisory_only=True,
            frame_sha256=[before['frame_sha256']], interpretation=route_advice(), model='test')
        return service, before, fresh

    def test_route_rechecks_after_cloud_and_returns_only_fresh_floor_binding(self):
        service, before, fresh = self.service()
        with patch('robot.mac.route_perception.time.time', return_value=100.2), \
             patch('robot.mac.navigation_advisor.route_frame_change', return_value=dict(stable=True)):
            result = service.route()
        self.assertEqual(2, service._local_route.call_count)
        self.assertEqual(fresh['frame_sha256'], result['route_reasoning']['fresh_frame_sha256'])
        self.assertEqual(before['frame_sha256'], result['route_reasoning']['source_frame_sha256'])
        self.assertAlmostEqual(.2, result['result_age_seconds'])
        self.assertEqual(.05, result['decision']['distance_m'])
        self.assertNotIn('jpeg', json.dumps(result))

    def test_cloud_cannot_clear_an_obstacle_arriving_during_latency(self):
        service, _, fresh = self.service()
        for e in fresh['evidence']:e['floor_fraction'] = .5
        with patch('robot.mac.navigation_advisor.route_frame_change', return_value=dict(stable=True)):
            self.assertTrue(service.route()['decision']['blocked'])

    def test_changed_head_or_image_and_mismatched_advice_are_rejected(self):
        for fault in ('head', 'image', 'binding', 'expired'):
            service, _, fresh = self.service()
            if fault == 'head':fresh['camera_head']['position'] = 30
            if fault == 'binding':service.navigation_advisor.route.return_value['frame_sha256'] = ['wrong']
            times = [10., 26.] if fault == 'expired' else [10., 11.]
            with self.subTest(fault=fault), \
                 patch('robot.mac.route_perception.time.monotonic', side_effect=times), \
                 patch('robot.mac.navigation_advisor.route_frame_change', return_value=dict(stable=fault != 'image')), \
                 self.assertRaises(RuntimeError):service.route()

    def test_occlusion_endpoint_is_frame_only_and_checks_input(self):
        service = RouteService(Mock());service.navigation_advisor = Mock()
        service.navigation_advisor.occlusion.return_value = dict(advisory_only=True)
        request = dict(request_id='a' * 32, frames_base64=[base64.b64encode(b'\xff\xd8' + b'x' * 100).decode()] * 2,
                       view_changed=True)
        with patch('robot.mac.route_perception.fetch_snapshot') as fetch:
            self.assertEqual('a' * 32, service.occlusion_advice(request)['request_id'])
            fetch.assert_not_called()
        for bad in (dict(request, view_changed='yes'), dict(request, frames_base64=['bad']),
                    dict(request, request_id='invalid'), dict(request, command='move')):
            with self.assertRaises(ValueError):service.occlusion_advice(bad)

    def test_occlusion_client_rejects_old_wrong_or_nonadvisory_response(self):
        observation = dict(quality='clear', cause='behind_object', inspect_side='left', evidence='Chair overlap.')
        frames = [b'old', b'current']
        result = dict(request_id='a' * 32, provider='gemini', advisory_only=True, interpretation=observation,
                      frame_sha256=[hashlib.sha256(f).hexdigest() for f in frames])
        for fault in (None, 'request_id', 'frame_sha256', 'advisory_only'):
            value = copy.deepcopy(result)
            if fault:value[fault] = None
            with patch('robot.jetson.mission.occlusion_advice.uuid4', return_value=NS(hex='a' * 32)), \
                 patch('robot.jetson.mission.occlusion_advice.urlopen', return_value=response(value)) as call:
                if fault:
                    with self.assertRaises(RuntimeError):request_occlusion_advice(*frames, 'http://localhost:8091/search-view', False)
                else:
                    self.assertEqual(observation, request_occlusion_advice(*frames,
                        'http://localhost:8091/search-view', False)['interpretation'])
                    self.assertEqual('http://localhost:8091/occlusion-advice', call.call_args.args[0].full_url)

    def test_image_guard_detects_scene_change_darkness_and_frozen_source(self):
        try:
            from PIL import Image, ImageDraw
            import numpy
        except ImportError:
            self.skipTest('Pillow/numpy are not installed in this test interpreter')
        def jpeg(image):
            out = io.BytesIO();image.save(out, format='JPEG');return out.getvalue()
        first = Image.new('RGB', (320, 240), (130, 130, 130))
        noise = Image.new('RGB', (320, 240), (131, 131, 131))
        self.assertTrue(route_frame_change(jpeg(first), jpeg(noise))['stable'])
        self.assertFalse(route_frame_change(jpeg(first), jpeg(first))['stable'])
        changed = first.copy();ImageDraw.Draw(changed).rectangle((80, 130, 210, 220), fill='black')
        self.assertFalse(route_frame_change(jpeg(first), jpeg(changed))['stable'])
        self.assertFalse(route_frame_change(jpeg(first), jpeg(Image.new('RGB', (320, 240))))['stable'])


if __name__ == '__main__':unittest.main()
