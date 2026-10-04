import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from robot.jetson.mission.autonomous_find import run, parse_args
from robot.jetson.mission.recovery_policy import recovery_kind
from robot.jetson.perception.recognition_core import ConfirmationWindow
from robot.jetson.perception.person_continuity import PersonContinuity


def target():
    return dict(outcome='target_found', final_cable_heading_degrees=12.,
                target_head_position=14, target_head_reference='head',
                target_observation=dict(confirmed=True, box_height_fraction=.12))


class Phase6RecoveryTests(unittest.TestCase):
    def test_delays_blocked_route_and_lost_target_recover_in_one_mission(self):
        with tempfile.TemporaryDirectory() as folder:
            args=parse_args(['--execute','--skip-camera-calibration','--cable-zero-confirmed',
                             '--one-step-test','--retry-delay-seconds','0','--report',folder+'/run.json'])
            results=[dict(outcome='failure',error='robot status is stale',final_cable_heading_degrees=5.),target(),
                     dict(outcome='failure',error='route perception reports blocked',final_cable_heading_degrees=20.),
                     dict(outcome='success',final_cable_heading_degrees=20.),
                     dict(outcome='scan_complete_no_target',final_cable_heading_degrees=22.),target()]
            with patch('robot.jetson.mission.autonomous_find.run_child',side_effect=results) as child:
                result=run(args)
            self.assertEqual('step_complete_target_reacquired',result['outcome'])
            self.assertEqual(['feedback','route','identity'],[r['kind'] for r in result['recoveries']])
            self.assertEqual(6,child.call_count)
            self.assertEqual(1,result['events'].count('approach_step_complete'))
            self.assertEqual(result,json.loads(Path(args.report).read_text()))

    def test_candidate_is_revisited_before_searching_past_them(self):
        candidate=dict(outcome='person_found_unidentified', final_cable_heading_degrees=10.,
                       target_head_position=17,target_head_reference='head')
        with tempfile.TemporaryDirectory() as folder:
            args=parse_args(['--execute','--camera-only','--skip-camera-calibration','--retry-delay-seconds','0',
                             '--report',folder+'/run.json'])
            with patch('robot.jetson.mission.autonomous_find.run_child',side_effect=[candidate,candidate,target()]) as child:
                result=run(args)
            self.assertEqual('target_found_not_at_standoff',result['outcome'])
            self.assertIn('--continue-past-unidentified',child.call_args_list[2].args[0])
            self.assertEqual('17',child.call_args_list[1].args[0][child.call_args_list[1].args[0].index('--preferred-head-position')+1])

    def test_persistent_gap_saves_pause_after_retries(self):
        with tempfile.TemporaryDirectory() as folder:
            args=parse_args(['--execute','--camera-only','--skip-camera-calibration','--retry-delay-seconds','0',
                             '--recovery-attempts','2','--report',folder+'/run.json'])
            with patch('robot.jetson.mission.autonomous_find.run_child',return_value=dict(
                    outcome='failure',error='robot status is stale',final_cable_heading_degrees=18.)) as child:
                result=run(args)
            self.assertEqual('paused',result['outcome'])
            self.assertEqual(3,child.call_count)
            self.assertEqual(18.,result['cable_heading_degrees'])

    def test_unknown_pose_and_failed_stop_are_never_retried_blindly(self):
        for extra in (dict(cable_heading_known=False),dict(stop_error='no acknowledgement')):
            self.assertIsNone(recovery_kind(dict(error='stale feedback',**extra)))
        self.assertEqual('identity', recovery_kind(dict(error='Target disappeared before centering')))
        self.assertEqual('feedback', recovery_kind(dict(error='Camera view is too dark after movement')))

    def test_worker_timeout_marks_heading_unknown_and_preserves_a_pause(self):
        import subprocess
        with tempfile.TemporaryDirectory() as folder:
            args = parse_args(['--execute', '--camera-only', '--skip-camera-calibration',
                               '--report', folder + '/run.json'])
            with patch('robot.jetson.mission.autonomous_find.subprocess.run',
                       side_effect=subprocess.TimeoutExpired('scan', 10)) as worker:
                result = run(args)
            self.assertEqual('paused', result['outcome'])
            self.assertIsNone(result['cable_heading_degrees'])
            self.assertEqual(1, worker.call_count)

    def test_household_matches_need_two_supporting_observations(self):
        window=ConfirmationWindow(5,2,.40)
        self.assertFalse(window.add(.41))
        self.assertFalse(window.add(.20))
        self.assertTrue(window.add(.43))
        self.assertFalse(ConfirmationWindow(5,2,.40).add(None))

    def test_camera_drop_after_drive_starts_reacquires_before_another_drive(self):
        with tempfile.TemporaryDirectory() as folder:
            args = parse_args(['--execute', '--skip-camera-calibration', '--cable-zero-confirmed',
                               '--one-step-test', '--retry-delay-seconds', '0', '--report', folder+'/run.json'])
            partial = dict(outcome='failure', error='camera frames are stale', drive_started=True,
                           cable_heading_known=True, final_cable_heading_degrees=12.)
            with patch('robot.jetson.mission.autonomous_find.run_child',
                       side_effect=[target(), partial, target()]) as child:
                result = run(args)
            self.assertEqual('step_complete_target_reacquired', result['outcome'])
            self.assertIn('interrupted_drive_requires_reacquisition', result['events'])
            self.assertEqual(['scan', 'target_approach', 'scan'], [s['kind'] for s in result['steps']])
            self.assertEqual(3, child.call_count)

    def test_two_face_anchors_can_seed_body_continuity(self):
        tracker=PersonContinuity(required_face_hits=2)
        person=dict(box=[.1,.1,.8,.95],appearance=[1.,0.])
        face=[.25,.15,.4,.3]
        tracker.observe([person],[face],1.,1.)
        self.assertEqual('face_confirmed',tracker.observe([person],[face],1.1,1.1)['state'])
        self.assertEqual('body_continuity',tracker.observe([person],[],1.2,1.2)['state'])
