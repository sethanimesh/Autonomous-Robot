import unittest
from robot.mac.vision_scene_advisor import validate_observations, VisionSceneAdvisor

class SceneAdvisorTests(unittest.TestCase):
    def observation(self, index=1):
        return dict(frame_index=index,view='ceiling',floor_visible='no',ceiling_visible='yes',human_visible='no',evidence='Overhead surface above curtains.')

    def test_valid_observations_preserve_explicit_uncertainty(self):
        a=self.observation();b=dict(self.observation(2),view='unknown',ceiling_visible='unknown')
        self.assertEqual([a,b],validate_observations(dict(observations=[a,b]),2))

    def test_missing_reordered_or_injected_commands_are_rejected(self):
        for value,count in [(dict(observations=[]),1),
                            (dict(observations=[self.observation(2)]),1),
                            (dict(observations=[dict(self.observation(),motor_command='down')]),1),
                            (dict(observations=[self.observation()],command='drive'),1)]:
            with self.assertRaises(ValueError):validate_observations(value,count)

    def test_conflicting_or_invalid_observations_are_rejected(self):
        for updates in [dict(ceiling_visible='no'),dict(view='floor_room'),dict(frame_index=True),
                        dict(human_visible=True),dict(evidence='')]:
            with self.assertRaises(ValueError):validate_observations(dict(observations=[dict(self.observation(),**updates)]),1)

    def test_bad_payloads_fail_before_network_request(self):
        for frames in [[],[b'a'],[b'a'*100]*7,['not image bytes']]:
            with self.assertRaises(ValueError):VisionSceneAdvisor().interpret(frames)

class GroqAdvisorTests(unittest.TestCase):
    def test_quota_failure_is_not_retried_or_sent_to_another_provider(self):
        from unittest.mock import patch
        from urllib.error import HTTPError
        from robot.mac.vision_scene_advisor import GroqVisionSceneAdvisor, GroqAccessError
        error=HTTPError('https://api.groq.com/openai/v1/chat/completions',429,'limited',{'retry-after':'20'},None)
        with patch('robot.mac.vision_scene_advisor.urlopen',side_effect=error) as request:
            with self.assertRaises(GroqAccessError) as result:
                GroqVisionSceneAdvisor(api_key='test-secret').interpret([b'x'*100])
            self.assertEqual(429,result.exception.status)
            self.assertEqual('20',result.exception.quota['retry-after'])
            self.assertEqual(1,request.call_count)
            self.assertNotIn('test-secret',str(result.exception))

    def test_success_binds_observations_to_frames_and_records_quota(self):
        import json
        from unittest.mock import patch,MagicMock
        from robot.mac.vision_scene_advisor import GroqVisionSceneAdvisor
        observation=SceneAdvisorTests().observation()
        response=MagicMock();response.__enter__.return_value=response
        response.read.return_value=json.dumps(dict(model='test-model',choices=[dict(finish_reason='stop',message=dict(content=json.dumps(dict(observations=[observation]))))],usage={'total_tokens':7})).encode()
        response.headers={'x-ratelimit-remaining-tokens':'6000','set-cookie':'private'}
        with patch('robot.mac.vision_scene_advisor.urlopen',return_value=response) as request:
            result=GroqVisionSceneAdvisor(api_key='test-secret').interpret([b'x'*100])
        self.assertEqual({'x-ratelimit-remaining-tokens':'6000'},result['quota'])
        self.assertEqual('groq',result['provider']);self.assertTrue(result['advisory_only'])
        body=json.loads(request.call_args.args[0].data)
        self.assertNotIn('tools',body)
        self.assertTrue(body['response_format']['json_schema']['strict'])
        self.assertNotIn('test-secret',json.dumps(result))
        self.assertEqual(64,len(result['frame_sha256'][0]))

    def test_four_images_rejected_before_request(self):
        from robot.mac.vision_scene_advisor import GroqVisionSceneAdvisor
        with self.assertRaises(ValueError):GroqVisionSceneAdvisor(api_key='test').interpret([b'x'*100]*4)
