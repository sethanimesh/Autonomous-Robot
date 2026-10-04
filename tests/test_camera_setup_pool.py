import base64
import hashlib
import json
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from robot.mac.camera_setup_pool import CameraSetupPool, CameraCloudError, GeminiCameraSetupAdvisor
from robot.mac.vision_scene_advisor import GroqAccessError
from tests.test_camera_visual_setup import view


class Client:
    def __init__(self, provider, model, error=None):
        self.provider, self.model, self.error, self.calls = provider, model, error, 0

    def interpret(self, frames):
        self.calls += 1
        if self.error:
            raise self.error
        return dict(provider=self.provider, model=self.model, observations=[view()],
                    advisory_only=True, frame_sha256=[hashlib.sha256(f).hexdigest() for f in frames])


class RotationTests(unittest.TestCase):
    def test_setup_and_search_use_only_selected_flash(self):
        with patch('robot.mac.camera_setup_pool.VertexADC', return_value=object()):
            search = CameraSetupPool(search=True)
            setup = CameraSetupPool()
        self.assertEqual(['gemini-3.5-flash-lite'], [c.model for c in search.clients])
        self.assertEqual(['gemini-3.5-flash-lite'], [c.model for c in setup.clients])
        self.assertEqual(20, search.clients[0].timeout)

    def test_missing_adc_does_not_silently_change_provider(self):
        with patch('robot.mac.camera_setup_pool.VertexADC', side_effect=RuntimeError('No ADC')):
            pool = CameraSetupPool()
        self.assertEqual([], pool.clients)
        with self.assertRaises(CameraCloudError) as error:
            pool.interpret([b'frame'])
        self.assertEqual(503, error.exception.status)

    def test_groq_limit_immediately_falls_back_to_gemini_and_remembers_cooldown(self):
        groq = Client('groq', 'qwen', GroqAccessError(429, {'retry-after': '20'}))
        gemini = Client('gemini', 'flash')
        now = [0]
        pool = CameraSetupPool([groq, gemini], clock=lambda: now[0])
        for _ in range(2):
            self.assertEqual('gemini', pool.interpret([b'frame'])['provider'])
        self.assertEqual(1, groq.calls)
        now[0] = 22
        groq.error = None
        self.assertEqual('groq', pool.interpret([b'frame'])['provider'])

    def test_timeout_is_not_reported_as_quota_and_does_not_busy_retry(self):
        client = Client('gemini', 'gemini-3.5-flash-lite', TimeoutError())
        pool = CameraSetupPool([client], clock=lambda: 0)
        for _ in range(2):
            with self.assertRaises(CameraCloudError) as error:
                pool.interpret([b'frame'])
            self.assertEqual(503, error.exception.status)
        self.assertEqual(1, client.calls)

    def test_all_default_robot_advisors_and_health_share_the_model(self):
        from unittest.mock import Mock
        from robot.mac.navigation_advisor import GeminiNavigationAdvisor
        from robot.mac.wardrobe_advisor import WardrobeAdvisor
        from robot.mac.vision_scene_advisor import GeminiVisionSceneAdvisor
        from robot.mac.route_perception import RouteService
        with patch('robot.mac.camera_setup_pool.VertexADC', return_value=object()), \
             patch.object(GeminiCameraSetupAdvisor, 'interpret_structured', return_value={}):
            navigation = GeminiNavigationAdvisor()
            navigation._interpret([], '', {}, lambda value: value, '')
            clients = [navigation.client, WardrobeAdvisor().client, GeminiVisionSceneAdvisor().client]
        self.assertEqual(['gemini-3.5-flash-lite'] * 3, [c.model for c in clients])
        health = RouteService(Mock()).health()
        for name in ('gemini_model', 'camera_setup_model', 'person_search_model', 'navigation_model', 'wardrobe_model'):
            self.assertEqual('gemini-3.5-flash-lite', health[name])

    def test_successful_requests_rotate_across_models(self):
        pool = CameraSetupPool([Client('groq', 'first'), Client('gemini', 'second')])
        self.assertEqual(['first', 'second', 'first'], [pool.interpret([b'f'])['model'] for _ in range(3)])

    def test_bad_schema_and_timeout_do_not_fail_whole_request(self):
        clients = [Client('groq', 'bad', ValueError('schema')),
                   Client('gemini', 'slow', TimeoutError()), Client('gemini', 'working')]
        result = CameraSetupPool(clients).interpret([b'f'])
        self.assertEqual('working', result['model'])
        self.assertEqual(2, len(result['model_fallbacks']))

    def test_all_limited_returns_earliest_cooldown_without_busy_retry(self):
        clients = [Client('groq', 'a', GroqAccessError(429, {'retry-after': '30'})),
                   Client('gemini', 'b', CameraCloudError(429, 10))]
        pool = CameraSetupPool(clients, clock=lambda: 0)
        for _ in range(2):
            with self.assertRaises(CameraCloudError) as error:
                pool.interpret([b'f'])
            self.assertEqual('11', error.exception.quota['retry-after'])
        self.assertEqual([1, 1], [client.calls for client in clients])

    def test_gemini_uses_adc_vertex_endpoint_and_same_validated_image(self):
        jpeg = b'\xff\xd8' + b'x' * 100
        class Response:
            def __enter__(self): return self
            def __exit__(self, *args): return False
            def read(self):
                return json.dumps(dict(candidates=[dict(finishReason='STOP', content=dict(parts=[
                    dict(text='Non-JSON reasoning', thought=True),
                    dict(text=json.dumps(dict(observations=[view()]))) ]))])).encode()
        with patch('robot.mac.camera_setup_pool.urlopen', return_value=Response()) as request:
            result = GeminiCameraSetupAdvisor(adc=SimpleNamespace(authorize=lambda model:(
                'https://aiplatform.googleapis.com/v1/projects/test/locations/global/publishers/google/models/'+model+':generateContent',
                {'Authorization':'Bearer test-adc-token'}))).interpret([jpeg])
        sent = request.call_args.args[0]
        self.assertNotIn('test-adc-token', sent.full_url)
        self.assertEqual('Bearer test-adc-token', sent.headers['Authorization'])
        self.assertFalse(any(k.lower() == 'x-goog-api-key' for k in sent.headers))
        self.assertEqual('adc', result['auth'])
        body = json.loads(sent.data)
        self.assertIn('/gemini-3.5-flash-lite:generateContent', sent.full_url)
        self.assertEqual({'thinkingLevel': 'MINIMAL'}, body['generationConfig']['thinkingConfig'])
        self.assertEqual(jpeg, base64.b64decode(body['contents'][0]['parts'][1]['inlineData']['data']))
        self.assertEqual([1], body['generationConfig']['responseJsonSchema']['properties']
                         ['observations']['items']['properties']['frame_index']['enum'])
        self.assertEqual('gemini', result['provider'])
        self.assertEqual([hashlib.sha256(jpeg).hexdigest()], result['frame_sha256'])


if __name__ == '__main__': unittest.main()
