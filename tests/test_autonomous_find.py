import unittest

from robot.jetson.mission.autonomous_find import heading_after_detour
from robot.jetson.mission.autonomous_find import heading_after_relative_scan
from robot.jetson.mission.autonomous_find import body_height
from robot.jetson.mission.autonomous_find import calibration_command
from robot.jetson.mission.autonomous_find import parse_args
from robot.jetson.mission.autonomous_find import target_height
from robot.jetson.mission.autonomous_find import target_is_at_standoff


class AutonomousFindTests(unittest.TestCase):
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

    def test_tested_fifteen_percent_face_height_is_standoff(self):
        report = {
            "target_observation": {"confirmed": True, "box_height_fraction": 0.165}
        }
        self.assertTrue(target_is_at_standoff(report))

    def test_large_body_is_standoff_even_when_face_box_is_small(self):
        report = {
            "target_observation": {
                "confirmed": True,
                "box_height_fraction": 0.1692,
            },
            "body_guided_tilts": [
                {"body": {"height_fraction": 0.9854}}
            ],
        }
        self.assertAlmostEqual(0.9854, body_height(report))
        self.assertTrue(target_is_at_standoff(report))

    def test_cable_heading_tracks_scan_and_detour_turns(self):
        self.assertEqual(35.0, heading_after_relative_scan(20, {"target_heading_degrees": 15}))
        self.assertEqual(5.0, heading_after_detour(20, {"turned_degrees": 15}))

    def test_camera_only_mode_is_explicit(self):
        self.assertTrue(parse_args(["--camera-only"]).camera_only)

    def test_mission_runs_calibration_before_search(self):
        args = parse_args([])
        command = calibration_command(args, "/tmp/head.json")

        self.assertIn("camera_head_calibration.py", command[1])
        self.assertIn("--execute", command)
        self.assertEqual("/tmp/head.json", command[-1])
        self.assertFalse(args.skip_camera_calibration)


if __name__ == "__main__":
    unittest.main()
