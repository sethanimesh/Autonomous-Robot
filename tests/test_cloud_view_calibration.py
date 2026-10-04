import copy
import hashlib
import json
from pathlib import Path
import unittest
from unittest.mock import Mock, patch

from robot.mac.cloud_view_calibration import CloudViewCalibration
from robot.mac.camera_setup_pool import CameraCloudError
from robot.mac.route_perception import RouteService
from robot.mac.vision_scene_advisor import GroqAccessError
from robot.jetson.mission.camera_head_calibration import (
    apply_cloud_advice, choose_runtime_positions, select_cloud_frames,
    select_revisited_cloud_frames, verify_view_role, CameraCalibrationError,
)


class CloudCalibrationTests(unittest.TestCase):
    def setUp(self):
        self.now = 10
        self.head = dict(available=True, homed=True, reference_id='ref',
                         approved_reference_id='ref', position=-9)
        self.payloads = [bytes([i]) * 100 for i in (1, 2, 3)]
        self.positions = [34, 12, -9]
        self.digests = [hashlib.sha256(p).hexdigest() for p in self.payloads]
        root = Path(__file__).resolve().parents[1]
        self.result = json.loads((root / 'docs/calibration/vlm_evaluation_20260905/groq_sequence.json').read_text())
        self.result['frame_sha256'] = self.digests
        self.advisor = Mock()
        self.advisor.interpret.side_effect = lambda *a, **k: copy.deepcopy(self.result)
        self.cache = CloudViewCalibration(self.advisor, clock=lambda: self.now,
                                         matcher=lambda a, b: dict(verified=a == b))
        for payload, position in zip(self.payloads, self.positions):
            self.cache.remember(payload, dict(self.head, position=position))
            self.now += 1
        self.request = dict(reference_id='ref', frame_sha256=self.digests)

    def test_advice_requires_final_pose_confirmation_before_installation(self):
        result = self.cache.analyze(self.request, current_position=-9)
        self.assertTrue(result['upper_verified'])
        self.assertFalse(self.cache.match(self.payloads[2], self.head)['verified'])
        self.cache.accept(result)
        self.assertEqual('overhead', self.cache.match(self.payloads[2], self.head)['role'])
        self.assertFalse(self.cache.match(self.payloads[2], dict(self.head, reference_id='reboot'))['verified'])
        self.cache.matcher = lambda a, b: dict(verified=True)
        self.assertFalse(self.cache.match(self.payloads[2], self.head)['verified'])

    def test_expired_reordered_wrong_reference_and_wrong_pose_never_call_cloud(self):
        bad = [dict(self.request, reference_id='reboot'),
               dict(self.request, frame_sha256=list(reversed(self.digests))),
               dict(self.request, frame_sha256=[self.digests[0]] * 3)]
        for request in bad:
            with self.assertRaises(ValueError):
                self.cache.analyze(request)
        with self.assertRaises(ValueError):
            self.cache.analyze(self.request, current_position=0)
        self.now = 611
        with self.assertRaises(ValueError):
            self.cache.analyze(self.request)
        self.advisor.interpret.assert_not_called()

    def test_manual_or_unreferenced_frames_are_not_cached(self):
        for change in (dict(homed=False), dict(moving=True), dict(manual_override=True),
                       dict(approved_reference_id='old')):
            self.cache.remember(b'new', dict(self.head, **change))
        self.assertEqual(3, len(self.cache.frames))

    def test_unknown_ceiling_does_not_install_a_label(self):
        self.result['observations'][2].update(view='unknown', ceiling_visible='unknown')
        result = self.cache.analyze(self.request)
        self.assertFalse(result['upper_verified'])
        self.cache.accept(result)
        self.assertEqual({}, self.cache.references)

    def test_room_view_can_include_floor_or_ceiling_without_exact_photo_match(self):
        self.result['observations'][1].update(view='floor_room', floor_visible='yes', ceiling_visible='yes')
        result = self.cache.analyze(self.request)
        self.assertTrue(result['upper_verified'])
        samples=[dict(position=p, frame_sha256=d, view_usable=True) for p,d in zip(self.positions,self.digests)]
        apply_cloud_advice(samples,result,'ref')
        self.assertTrue(verify_view_role('forward',samples[1]))

    def test_revisits_use_latest_functional_views_and_do_not_claim_an_upward_sequence(self):
        # Revisit floor, upper, then room. The camera remains at the last room
        # capture although the images supplied to Groq are ordered by role.
        self.now += 1
        self.cache.remember(self.payloads[1],dict(self.head,position=12))
        self.cache.analyze(self.request,current_position=12)
        self.assertFalse(self.advisor.interpret.call_args.kwargs['upward_sequence'])
        checks=[dict(role=role,position=p,frame_sha256=d,view_usable=True)
                for role,p,d in zip(('down','forward','up'),self.positions,self.digests)]
        newer=dict(checks[1],frame_sha256='fresh-frame',position=10)
        selected=select_revisited_cloud_frames(checks+[newer])
        self.assertIs(newer,selected[1])
        with self.assertRaises(CameraCalibrationError):
            select_revisited_cloud_frames(checks+[dict(newer,view_usable=False)])

    def test_quota_is_reported_without_retry(self):
        self.advisor.interpret.side_effect = GroqAccessError(429, {'retry-after': '20'})
        with self.assertRaises(GroqAccessError):
            self.cache.analyze(self.request)
        self.assertEqual({'retry-after': '20'}, self.cache.last_quota)
        with self.assertRaises(CameraCloudError):
            self.cache.analyze(self.request)
        self.advisor.interpret.assert_called_once()

    def test_gemini_labels_keep_provider_and_work_in_calibration(self):
        from robot.jetson.mission.camera_head_calibration import cloud_capacity_delay
        self.result.update(provider='gemini', model='gemini-3.5-flash-lite', auth='adc')
        result = self.cache.analyze(self.request)
        self.assertTrue(result['upper_verified'])
        self.cache.accept(result)
        matched = self.cache.match(self.payloads[2], self.head)
        self.assertEqual('gemini', matched['provider'])
        self.assertEqual('gemini', matched['label_source'])
        samples = [dict(position=p, frame_sha256=d, view_usable=True)
                   for p, d in zip(self.positions, self.digests)]
        apply_cloud_advice(samples, result, 'ref')
        self.assertEqual(12, choose_runtime_positions(samples, floor_position=34)['forward_position'])
        self.assertEqual(0, cloud_capacity_delay(result))

    def test_gemini_quota_waits_without_repeated_requests(self):
        self.advisor.interpret.side_effect = CameraCloudError(429, 70)
        for _ in range(2):
            with self.assertRaises(CameraCloudError):
                self.cache.analyze(self.request)
        self.advisor.interpret.assert_called_once()

    def test_head_change_during_inference_rejects_advice(self):
        service = RouteService(Mock())
        service.cloud_views = self.cache
        with patch('robot.mac.route_perception.fetch_camera_status', side_effect=[
                dict(camera_head=self.head), dict(camera_head=dict(self.head, position=0))]):
            with self.assertRaises(ValueError):
                service.scene_advice(self.request)
        self.assertEqual({}, self.cache.references)

    def test_stopped_small_settling_variation_keeps_capture_binding(self):
        from robot.mac.route_perception import same_head_pose
        self.assertTrue(same_head_pose(self.head,dict(self.head,position=-6),3))
        self.assertFalse(same_head_pose(self.head,dict(self.head,position=-5),3))
        self.assertFalse(same_head_pose(self.head,dict(self.head,reference_id='new'),3))
        self.assertFalse(same_head_pose(self.head,dict(self.head,position=-8)))
        result=self.cache.analyze(self.request,current_position=-7)
        self.assertEqual([34,12,-9],result['positions'])

    def test_route_accepts_small_stopped_settling_but_rejects_larger_changes(self):
        for offset, accepted in [(2, True), (4, False)]:
            engine = Mock();engine.infer.return_value = {'ok': True}
            service = RouteService(engine)
            with patch('robot.mac.route_perception.fetch_snapshot', return_value=b'image'), \
                 patch('robot.mac.route_perception.fetch_camera_status', side_effect=[
                     {'camera_head': self.head}, {'camera_head': dict(self.head,position=self.head['position']+offset)},
                     {'camera_head': dict(self.head,position=self.head['position']+offset)}]):
                if accepted:self.assertTrue(service._local_route()['ok'])
                else:
                    with self.assertRaises(RuntimeError):service._local_route()

    def test_successful_service_response_contains_no_image_payloads(self):
        service = RouteService(Mock())
        service.cloud_views = self.cache
        with patch('robot.mac.route_perception.fetch_camera_status', return_value=dict(camera_head=self.head)):
            result = service.scene_advice(self.request)
        self.assertNotIn('reference_payloads', result)
        json.dumps(result)
        self.assertEqual(3, len(self.cache.references))

    def test_calibrator_uses_semantic_roles_without_rewriting_pixel_evidence(self):
        samples = [dict(position=p, target_position=p, frame_sha256=d, view_usable=True,
                        floor_fraction=0, ceiling_fraction=0, known_fraction=.3, route_blocked=True)
                   for p, d in zip(self.positions, self.digests)]
        selected = select_cloud_frames(samples)
        result = self.cache.analyze(self.request)
        for changes in (dict(reference_id='reboot'), dict(positions=[34, 12, -8]),
                        dict(frame_sha256=list(reversed(self.digests))), dict(provider='ollama')):
            with self.assertRaises(CameraCalibrationError):
                apply_cloud_advice(selected, dict(result, **changes), 'ref')
            self.assertTrue(all('cloud_view_role' not in s for s in selected))
        apply_cloud_advice(selected, result, 'ref')
        calibration = choose_runtime_positions(samples, floor_position=34)
        self.assertEqual('cloud_multi_view', calibration['selection_reason'])
        for role, sample in zip(('down', 'forward', 'up'), samples):
            self.assertTrue(verify_view_role(role, sample))
            self.assertEqual(0, sample['ceiling_fraction'])
            self.assertTrue(sample['route_blocked'])
        with self.assertRaises(CameraCalibrationError):
            verify_view_role('invalid', dict(view_usable=True, cloud_advice_id='id'))


if __name__ == '__main__':
    unittest.main()
