import unittest

from robot.jetson.mission.autonomous_find import heading_after_detour
from robot.jetson.mission.autonomous_find import heading_after_relative_scan
from robot.jetson.mission.autonomous_find import body_height
from robot.jetson.mission.autonomous_find import calibration_command
from robot.jetson.mission.autonomous_find import parse_args
from robot.jetson.mission.autonomous_find import target_height
from robot.jetson.mission.autonomous_find import target_is_at_standoff


class AutonomousFindTests(unittest.TestCase):
    def test_multistep_cap_stops_after_four_reacquisitions_without_reporting_failure(self):
        import tempfile
        from unittest.mock import patch
        from robot.jetson.mission.autonomous_find import run
        with tempfile.TemporaryDirectory() as folder:
            args = parse_args(['--execute', '--skip-camera-calibration', '--cable-zero-confirmed',
                               '--preserve-initial-heading', '--initial-cable-heading-degrees', '-94.117',
                               '--maximum-search-moves', '0', '--maximum-approach-steps', '4',
                               '--report', folder+'/mission.json'])
            target = dict(outcome='target_found', target_observation=dict(confirmed=True, box_height_fraction=.14))
            results = [target] + [item for _ in range(4) for item in (dict(outcome='success'), target)]
            with patch('robot.jetson.mission.autonomous_find.run_child', side_effect=results) as execute:
                result = run(args)
            self.assertEqual('target_found_not_at_standoff', result['outcome'])
            self.assertEqual(4, result['events'].count('approach_step_complete'))
            self.assertEqual(4, result['events'].count('target_reacquired'))
            self.assertIn('approach_step_limit_reached', result['events'])
            self.assertIn('--preserve-initial-heading', execute.call_args_list[0].args[0])
            self.assertEqual(9, execute.call_count)

    def test_one_step_test_stops_after_reacquisition_or_loss(self):
        import tempfile
        from unittest.mock import patch
        from robot.jetson.mission.autonomous_find import run
        for reacquired in (True, False):
            with self.subTest(reacquired=reacquired), tempfile.TemporaryDirectory() as folder:
                args = parse_args(['--execute', '--one-step-test', '--cable-zero-confirmed',
                                   '--skip-camera-calibration', '--reacquire-retries', '0', '--report', folder+'/mission.json'])
                self.assertEqual((0, 1), (args.maximum_search_moves, args.maximum_approach_steps))
                target = dict(outcome='target_found', target_centered=True,
                              target_head_position=11, target_head_reference='same-boot',
                              target_observation=dict(confirmed=True, box_height_fraction=.08))
                final = target if reacquired else dict(outcome='scan_complete_no_target')
                with patch('robot.jetson.mission.autonomous_find.run_child',
                           side_effect=[target, dict(outcome='success'), final]) as execute:
                    result = run(args)
                self.assertEqual('step_complete_target_reacquired' if reacquired else 'paused', result['outcome'])
                self.assertEqual(['scan', 'target_approach', 'scan'], [s['kind'] for s in result['steps']])
                self.assertEqual(3, execute.call_count)
                self.assertIn('--center-target', execute.call_args_list[0].args[0])
                command = execute.call_args_list[1].args[0]
                self.assertEqual('0.05', command[command.index('--maximum-step-distance')+1])
                reacquire = execute.call_args_list[2].args[0]
                self.assertEqual('11', reacquire[reacquire.index('--preferred-head-position')+1])
                self.assertEqual('same-boot', reacquire[reacquire.index('--preferred-head-reference')+1])

    def test_one_step_test_does_not_force_motion_when_already_close(self):
        import tempfile
        from unittest.mock import patch
        from robot.jetson.mission.autonomous_find import run
        with tempfile.TemporaryDirectory() as folder:
            args = parse_args(['--execute', '--one-step-test', '--cable-zero-confirmed',
                               '--skip-camera-calibration', '--report', folder+'/mission.json'])
            with patch('robot.jetson.mission.autonomous_find.run_child', return_value=dict(
                    outcome='target_found', target_observation=dict(confirmed=True, box_height_fraction=.30))) as execute:
                result = run(args)
            self.assertEqual('target_found_at_standoff', result['outcome'])
            self.assertEqual(1, execute.call_count)

    def test_only_a_confirmed_finite_target_has_height(self):
        self.assertIsNone(target_height({}))
        self.assertIsNone(
            target_height(
                {"target_observation": {"confirmed": False, "box_height_fraction": 0.8}}
            )
        )
        self.assertEqual(
            0.32,
            target_height(
                {"target_observation": {"confirmed": True, "box_height_fraction": 0.32}}
            ),
        )

    def test_closer_standoff_allows_old_fifteen_percent_face_size(self):
        report = {
            "target_observation": {"confirmed": True, "box_height_fraction": 0.165}
        }
        self.assertFalse(target_is_at_standoff(report))
        report['target_observation']['box_height_fraction'] = .22
        self.assertFalse(target_is_at_standoff(report))
        report['target_observation']['box_height_fraction'] = .28
        self.assertTrue(target_is_at_standoff(report))

    def test_cropped_body_does_not_stop_an_approach_when_face_is_small(self):
        report = {
            "target_observation": {
                "confirmed": True,
                "box_height_fraction": 0.10,
            },
            "body_guided_tilts": [
                {"body": {"height_fraction": 0.9854}}
            ],
        }
        self.assertAlmostEqual(0.9854, body_height(report))
        self.assertFalse(target_is_at_standoff(report))

    def test_standing_body_at_distance_does_not_end_approach_prematurely(self):
        report = dict(target_observation=dict(confirmed=True, box_height_fraction=.1341),
                      body_guided_tilts=[dict(body=dict(height_fraction=.738))])
        self.assertFalse(target_is_at_standoff(report))

    def test_cable_heading_tracks_scan_and_detour_turns(self):
        self.assertEqual(35.0, heading_after_relative_scan(20, {"target_heading_degrees": 15}))
        self.assertEqual(35.0, heading_after_detour(20, {"turned_degrees": 15}))

    def test_interrupted_turn_stops_mission_without_reusing_old_cable_heading(self):
        import tempfile
        from unittest.mock import patch
        from robot.jetson.mission.autonomous_find import run
        with tempfile.TemporaryDirectory() as folder:
            args = parse_args(['--execute', '--camera-only', '--skip-camera-calibration',
                               '--report', folder+'/mission.json'])
            child = dict(outcome='failure', error='odometry is stale',
                         cable_heading_known=False, final_cable_heading_degrees=None)
            with patch('robot.jetson.mission.autonomous_find.run_child', return_value=child) as execute:
                result = run(args)
            self.assertEqual('paused', result['outcome'])
            self.assertIsNone(result['cable_heading_degrees'])
            self.assertIn('restore cable neutral', result['error'])
            self.assertEqual(child, result['steps'][0]['report'])
            self.assertEqual(1, execute.call_count)

    def test_failed_approach_retains_its_turn_or_marks_heading_unknown(self):
        import tempfile
        from unittest.mock import patch
        from robot.jetson.mission.autonomous_find import run
        for known in (True, False):
            with self.subTest(known=known), tempfile.TemporaryDirectory() as folder:
                args = parse_args(['--execute', '--cable-zero-confirmed', '--skip-camera-calibration',
                                   '--recovery-attempts', '0', '--report', folder+'/mission.json'])
                scan = dict(outcome='target_found', final_cable_heading_degrees=10,
                            target_observation=dict(confirmed=True, box_height_fraction=.05))
                move = dict(outcome='failure', error='path blocked', cable_heading_known=known,
                            final_cable_heading_degrees=25 if known else None)
                with patch('robot.jetson.mission.autonomous_find.run_child', side_effect=[scan,move]) as execute:
                    result = run(args)
                self.assertEqual('paused', result['outcome'])
                self.assertEqual(25 if known else None, result['cable_heading_degrees'])
                self.assertEqual(2, execute.call_count)

    def test_camera_only_mode_is_explicit(self):
        self.assertTrue(parse_args(["--camera-only"]).camera_only)

    def test_mission_runs_calibration_before_search(self):
        args = parse_args([])
        command = calibration_command(args, "/tmp/head.json")

        self.assertIn("camera_head_calibration.py", command[1])
        self.assertIn("--execute", command)
        self.assertEqual("/tmp/head.json", command[-1])
        self.assertFalse(args.skip_camera_calibration)
        self.assertEqual(600.0,args.calibration_timeout_seconds)
        self.assertEqual(180.0,args.child_timeout_seconds)
        self.assertEqual(900.0,args.scan_timeout_seconds)


if __name__ == "__main__":
    unittest.main()
