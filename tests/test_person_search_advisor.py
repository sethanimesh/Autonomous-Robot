import json
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch
from robot.mac.person_search_advisor import SEARCH_PROMPT, validate_search_observations
from robot.mac.camera_setup_pool import GeminiCameraSetupAdvisor


def observation(**changes):
    result=dict(frame_index=1,quality='clear',scene='room',evidence='Torso reaches the top edge.',
                human_visible='yes',visible_parts=['torso'],framing_hint='raise')
    result.update(changes)
    return result


class PersonSearchAdvisorTests(unittest.TestCase):
    def test_rejects_invalid_or_unreadable_body_evidence(self):
        for change in (dict(visible_parts=[]),dict(visible_parts=['chair']),dict(quality='dark'),
                       dict(framing_hint='drive_forward'),dict(frame_index=0)):
            with self.subTest(change=change),self.assertRaises(ValueError):
                validate_search_observations(dict(observations=[observation(**change)]),1)

    def test_search_request_requires_body_parts_and_framing_through_adc(self):
        class Response:
            def __enter__(self):return self
            def __exit__(self,*args):return False
            def read(self):return json.dumps(dict(candidates=[dict(finishReason='STOP',content=dict(parts=[
                dict(text=json.dumps(dict(observations=[observation()]))) ]))])).encode()
        adc=NS(authorize=lambda model:('https://aiplatform.googleapis.com/test',{'Authorization':'Bearer test'}))
        with patch('robot.mac.camera_setup_pool.urlopen',return_value=Response()) as call:
            result=GeminiCameraSetupAdvisor(adc=adc,prompt=SEARCH_PROMPT,search=True).interpret([b'jpeg'*100])
        request=json.loads(call.call_args.args[0].data)
        fields=request['generationConfig']['responseJsonSchema']['properties']['observations']['items']['required']
        self.assertIn('visible_parts',fields);self.assertIn('framing_hint',fields)
        self.assertNotIn('lower_suitable',fields)
        self.assertEqual('MINIMAL',request['generationConfig']['thinkingConfig']['thinkingLevel'])
        self.assertEqual('raise',result['observations'][0]['framing_hint'])
        self.assertEqual('adc',result['auth'])


if __name__=='__main__':unittest.main()
