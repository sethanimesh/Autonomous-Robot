import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import MagicMock, patch
from robot.mac.camera_setup_advisor import (GroqCameraSetupAdvisor,
    setup_view_decision, validate_setup_observations)


def observation(**updates):
    value=dict(frame_index=1,quality='clear',scene='floor_room',
        near_floor_visible='yes',room_context_visible='yes',
        lower_suitable='yes',upper_suitable='no',evidence='Floor with room context.')
    value.update(updates)
    return value


class CameraSetupAdvisorTests(unittest.TestCase):
    def test_useful_views_need_no_encoder_or_room_image_match(self):
        self.assertEqual('candidate_view',setup_view_decision(observation(),'lower')['action'])
        upper=observation(scene='upper_room',near_floor_visible='no',lower_suitable='no',upper_suitable='yes')
        self.assertEqual('candidate_view',setup_view_decision(upper,'upper')['action'])

    def test_floor_only_raises_and_ceiling_only_lowers(self):
        for scene, direction in (('floor_only','look_up'),('ceiling_only','look_down')):
            for goal in ('lower','upper'):
                with self.subTest(scene=scene,goal=goal):
                    view=observation(scene=scene,lower_suitable='no',room_context_visible='no')
                    self.assertEqual(direction,setup_view_decision(view,goal)['action'])
                    self.assertEqual(5,setup_view_decision(view,goal)['step_counts'])

    def test_black_or_obstructed_image_does_not_invent_a_floor_limit(self):
        for quality in ('dark','occluded','blurred','unknown'):
            view=observation(quality=quality,scene='unknown',lower_suitable='unknown',upper_suitable='unknown')
            decision=setup_view_decision(view,'lower')
            self.assertEqual('recover_visibility',decision['action'])
            self.assertEqual(0,decision['step_counts'])

    def test_unknown_scene_requests_another_observation(self):
        view=observation(scene='unknown',lower_suitable='unknown',upper_suitable='unknown')
        self.assertEqual('observe_again',setup_view_decision(view,'upper')['action'])

    def test_room_context_failure_does_not_keep_raising_past_an_upper_view(self):
        # Regression: the manual upper image was called a room, but the model
        # rejected its context and the old rule unconditionally proposed up.
        for context in ('no','unknown'):
            for goal in ('lower','upper'):
                with self.subTest(context=context,goal=goal):
                    view=observation(scene='room',near_floor_visible='no',
                        room_context_visible=context,lower_suitable='no',upper_suitable='no')
                    decision=setup_view_decision(view,goal)
                    self.assertEqual('observe_again',decision['action'])
                    self.assertEqual(0,decision['step_counts'])

    def test_uncertain_suitability_does_not_invent_a_tilt_direction(self):
        for goal in ('lower','upper'):
            view=observation(scene='room',lower_suitable='unknown',upper_suitable='unknown')
            decision=setup_view_decision(view,goal)
            self.assertEqual('observe_again',decision['action'])
            self.assertEqual(0,decision['step_counts'])

    def test_room_view_can_be_an_upper_candidate_without_ceiling(self):
        view=observation(scene='room',near_floor_visible='no',lower_suitable='no',
            upper_suitable='yes',evidence='Vertical room features above the near-floor region.')
        self.assertEqual('candidate_view',setup_view_decision(view,'upper')['action'])

    def test_commands_and_contradictory_suitability_are_rejected(self):
        invalid=[observation(motor_force=100),observation(frame_index=2),
            observation(quality='dark'),observation(near_floor_visible='no'),
            observation(scene='ceiling_only'),observation(evidence='')]
        for view in invalid:
            with self.subTest(view=view),self.assertRaises(ValueError):
                validate_setup_observations({'observations':[view]},1)

    def test_groq_result_binds_image_digest_and_contains_no_actuator_interface(self):
        response=MagicMock();response.__enter__.return_value=response
        response.read.return_value=json.dumps(dict(model='vision-test',choices=[dict(
            finish_reason='stop',message=dict(content=json.dumps({'observations':[observation()]})))])).encode()
        response.headers={}
        with patch('robot.mac.vision_scene_advisor.urlopen',return_value=response) as request:
            result=GroqCameraSetupAdvisor(api_key='not-a-real-key').interpret([b'x'*100])
        payload=json.loads(request.call_args.args[0].data)
        self.assertNotIn('tools',payload)
        self.assertIn('Never infer motor angle or mechanical limits',payload['messages'][0]['content'])
        self.assertEqual(64,len(result['frame_sha256'][0]))
        self.assertTrue(result['advisory_only'])
        self.assertNotIn('not-a-real-key',json.dumps(result))

    def test_image_evaluation_does_not_claim_motor_calibration(self):
        from scripts.diagnostics.camera_setup_images import evaluate
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'forward.jpg';path.write_bytes(b'jpeg-placeholder')
            client=MagicMock()
            client.interpret.return_value={'observations':[observation()]}
            result=evaluate([path],Path(directory)/'report.json',advisor=client)
            self.assertFalse(result['calibration_applied'])
            self.assertEqual('candidate_view',result['image_tests'][0]['lower_decision']['action'])
            self.assertEqual([b'jpeg-placeholder'],client.interpret.call_args.args[0])


if __name__=='__main__':unittest.main()
