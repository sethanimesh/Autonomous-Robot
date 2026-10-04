import copy
import json
from pathlib import Path
import unittest
from unittest.mock import Mock

from robot.jetson.mission.camera_head_calibration import (
    CameraCalibrationError, cloud_capacity_delay, recover_cloud_room,
    choose_runtime_positions,
    usable_motion_feedback,
    select_cloud_frames, validate_runtime_positions,
)


class SemanticRecoveryTests(unittest.TestCase):
    def test_restart_sweep_selects_and_validates_only_current_cloud_sequence(self):
        path = Path(__file__).resolve().parents[1] / 'docs/calibration/recovery_live_20260905/service_restart_test.json'
        report = json.loads(path.read_text())
        selected = select_cloud_frames(report['samples'])
        selected, _ = recover_cloud_room(
            selected, report['cloud_scene_advice']['reference_id'], Mock(), Mock(),
            -41, -3, [], initial_advice=report['cloud_scene_advice'])
        calibration = choose_runtime_positions(selected, floor_position=-3)
        self.assertEqual((-3, -25, -41), tuple(calibration[k] for k in (
            'down_position', 'forward_position', 'up_position')))
        self.assertTrue(validate_runtime_positions(selected, calibration, floor_position=-3)['verified'])

    def test_user_upper_limit_accepts_recorded_raised_room_without_claiming_ceiling(self):
        path = Path(__file__).resolve().parents[1] / 'docs/calibration/user_limits_20260905/result.json'
        report = json.loads(path.read_text())
        advice = report['semantic_recovery'][0]['attempts'][0]['advice']
        samples = report['samples']
        move = Mock()
        with self.assertRaises(CameraCalibrationError):
            recover_cloud_room(copy.deepcopy(samples), advice['reference_id'], Mock(), move,
                               -7, 32, [], initial_advice=advice)
        selected, returned = recover_cloud_room(samples, advice['reference_id'], Mock(), move,
            -7, 32, [], initial_advice=advice, allow_raised_room=True)
        self.assertEqual('raised', selected[2]['cloud_view_role'])
        self.assertFalse(returned['upper_verified'])
        self.assertEqual(-7, choose_runtime_positions(selected, floor_position=32)['up_position'])
        move.assert_not_called()
        unknown = copy.deepcopy(advice)
        unknown['observations'][2]['view'] = 'unknown'
        with self.assertRaises(CameraCalibrationError):
            recover_cloud_room(samples, advice['reference_id'], Mock(), move,
                -7, 32, [], initial_advice=unknown, allow_raised_room=True)

    def setUp(self):
        root=Path(__file__).resolve().parents[1] / 'docs/calibration/groq_manual_20260905'
        self.actual=json.loads((root/'groq_functional_views.json').read_text())
        self.selected=[dict(position=p,target_position=p,frame_sha256=h,view_usable=True)
                       for p,h in zip((2,-22,-44),self.actual['frame_sha256'])]
        for sample,target in zip(self.selected,(0,-21,-41)):
            sample['target_position']=target
        self.events=[]

    def reply(self, samples, good=False):
        result=copy.deepcopy(self.actual)
        result.update(ok=True,advice_id='test-advice',reference_id='ref',
                      positions=[s['position'] for s in samples],
                      frame_sha256=[s['frame_sha256'] for s in samples],upper_verified=good)
        if good:
            # Useful forward composition may include an overhead edge.
            result['observations'][1].update(view='room',ceiling_visible='yes')
        return result

    def capture(self, target):
        return dict(position=target,target_position=target,frame_sha256='new-'+str(target),view_usable=True)

    def run_recovery(self, query, move=None, **kwargs):
        return recover_cloud_room(copy.deepcopy(self.selected),'ref',query,move or self.capture,
                                  minimum=-41,maximum=0,events=self.events,**kwargs)

    def test_saved_overhead_middle_triggers_bounded_lower_step_then_accepts_shifted_room(self):
        replies=[]
        def query(samples):
            replies.append(samples[1]['position'])
            return self.reply(samples,good=len(replies)==2)
        move=Mock(side_effect=self.capture)
        selected,advice=self.run_recovery(query,move)
        move.assert_called_once_with(-17)
        self.assertEqual([-22,-17],replies)
        self.assertEqual('room',selected[1]['cloud_view_role'])
        self.assertEqual(-17,choose_runtime_positions(selected,floor_position=0)['forward_position'])
        self.assertEqual(2,len(self.events))
        self.assertEqual('ceiling',self.events[0]['advice']['observations'][1]['view'])

    def test_unknown_endpoints_wrong_reference_and_wrong_frame_never_move(self):
        for change in ('unknown','reference','frame','unknown_middle'):
            result=self.reply(self.selected)
            if change=='unknown':result['observations'][2]['ceiling_visible']='unknown'
            if change=='reference':result['reference_id']='old-boot'
            if change=='frame':result['frame_sha256'][1]='different-image'
            if change=='unknown_middle':result['observations'][1]['view']='unknown'
            move=Mock()
            with self.subTest(change=change),self.assertRaises(CameraCalibrationError):
                self.run_recovery(Mock(return_value=result),move)
            move.assert_not_called()

    def test_persistent_overhead_stops_after_three_steps(self):
        query=Mock(side_effect=lambda samples:self.reply(samples))
        move=Mock(side_effect=self.capture)
        with self.assertRaises(CameraCalibrationError):self.run_recovery(query,move)
        self.assertEqual([-17,-12,-7],[call.args[0] for call in move.call_args_list])
        self.assertEqual(4,query.call_count)

    def test_lower_boundary_keeps_room_between_floor_and_upper(self):
        self.selected[1].update(position=-4,target_position=-4)
        move=Mock()
        with self.assertRaises(CameraCalibrationError):
            self.run_recovery(lambda samples:self.reply(samples),move)
        move.assert_not_called()

    def test_failed_motion_or_bad_image_cannot_be_accepted(self):
        for changed in (dict(position=-23),dict(view_usable=False),dict(frame_sha256=None)):
            move=Mock(side_effect=lambda target:dict(self.capture(target),**changed))
            query=Mock(side_effect=lambda samples:self.reply(samples))
            with self.subTest(changed=changed),self.assertRaises(CameraCalibrationError):
                self.run_recovery(query,move)
            self.assertEqual(1,query.call_count)

    def test_quota_failure_propagates_without_movement_or_retry(self):
        query=Mock(side_effect=CameraCalibrationError('Groq quota/rate limit reached'))
        move=Mock()
        with self.assertRaises(CameraCalibrationError):self.run_recovery(query,move)
        query.assert_called_once();move.assert_not_called()

    def test_usable_room_needs_no_correction_even_when_composition_differs(self):
        move=Mock()
        selected,_=self.run_recovery(lambda samples:self.reply(samples,good=True),move)
        move.assert_not_called()
        self.assertEqual(-22,selected[1]['position'])

    def test_recorded_lower_alternative_is_accepted_from_actual_groq_result(self):
        root=Path(__file__).resolve().parents[1] / 'docs/calibration/groq_manual_20260905'
        result=json.loads((root/'groq_lower_forward_candidate.json').read_text())
        samples=[dict(position=p,target_position=t,frame_sha256=h,view_usable=True)
                 for p,t,h in zip(result['positions'],result['target_positions'],result['frame_sha256'])]
        result.update(ok=True,upper_verified=True,advice_id='recorded-replay',reference_id='ref')
        move=Mock()
        selected,_=recover_cloud_room(samples,'ref',Mock(return_value=result),move,-41,0,[])
        self.assertEqual(-13,choose_runtime_positions(selected,floor_position=0)['forward_position'])
        self.assertNotEqual(-26,selected[1]['position'])
        move.assert_not_called()

    def test_pacing_uses_provider_reset_durations(self):
        for value,expected in [('53.91s',65),('1m2.5s',67.5),('500ms',65),('bad',65)]:
            self.assertAlmostEqual(expected,cloud_capacity_delay({'quota':{'x-ratelimit-reset-tokens':value}}))
        self.assertEqual(65,cloud_capacity_delay({}))
        with self.assertRaises(CameraCalibrationError):
            cloud_capacity_delay({'quota':{'x-ratelimit-reset-tokens':'1h'}})

    def test_recorded_scene_change_is_diagnostic_but_unusable_view_is_rejected(self):
        path=Path(__file__).resolve().parents[1] / 'docs/calibration/recovery_live_20260905/first_attempt.json'
        feedback=json.loads(path.read_text())['motion_feedback'][0]['visual']
        self.assertTrue(usable_motion_feedback(feedback))
        self.assertFalse(usable_motion_feedback(dict(feedback,settled=dict(usable=False,median_luma=0))))


if __name__=='__main__':unittest.main()
