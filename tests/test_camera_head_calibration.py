import unittest

from robot.jetson.mission.camera_head_calibration import center_floor_fraction
from robot.jetson.mission.camera_head_calibration import choose_runtime_positions
from robot.jetson.mission.camera_head_calibration import dry_run_report
from robot.jetson.mission.camera_head_calibration import next_upward_position
from robot.jetson.mission.camera_head_calibration import parse_args
from robot.jetson.mission.camera_head_calibration import track_motion_active


class CameraHeadCalibrationTests(unittest.TestCase):
    def test_upward_steps_are_bounded_and_end_exactly_on_limit(self):
        positions = [0]
        while True:
            value = next_upward_position(positions[-1], -52, 15)
            if value is None:
                break
            positions.append(value)
        self.assertEqual([0, -15, -30, -45, -52], positions)

    def test_target_face_selects_runtime_forward_position(self):
        result = choose_runtime_positions(
            [
                {"position": 0, "floor_fraction": 0.95},
                {"position": -15, "body_height_fraction": 0.8},
                {"position": -30, "target_score": 0.51},
                {"position": -45, "target_score": 0.67, "target_confirmed": True},
            ]
        )
        self.assertEqual(0, result["down_position"])
        self.assertEqual(-45, result["forward_position"])
        self.assertEqual("target_face", result["selection_reason"])

    def test_body_and_floor_are_valid_fallbacks(self):
        body = choose_runtime_positions(
            [
                {"position": 0, "floor_fraction": 0.9},
                {"position": -15, "body_height_fraction": 0.4},
                {"position": -30, "body_height_fraction": 0.8},
            ]
        )
        self.assertEqual(-30, body["forward_position"])

        floor = choose_runtime_positions(
            [
                {"position": 0, "floor_fraction": 0.95},
                {"position": -15, "floor_fraction": 0.48},
                {"position": -30, "floor_fraction": 0.05},
            ]
        )
        self.assertEqual(-15, floor["forward_position"])

    def test_route_center_floor_fraction_uses_nearest_heading(self):
        value = center_floor_fraction(
            {
                "evidence": [
                    {"heading_degrees": -20, "floor_fraction": 0.1},
                    {"heading_degrees": 5, "floor_fraction": 0.7},
                    {"heading_degrees": 0, "floor_fraction": 0.8},
                ]
            }
        )
        self.assertEqual(0.8, value)

    def test_tool_motion_never_counts_as_chassis_motion(self):
        self.assertFalse(
            track_motion_active(
                {
                    "motion_active": True,
                    "track_motion_active": False,
                    "tool_motion_active": True,
                }
            )
        )

    def test_dry_run_contains_no_chassis_commands(self):
        report = dry_run_report(parse_args([]))
        self.assertTrue(report["chassis_locked"])
        self.assertEqual(0, report["planned_positions"][0])
        self.assertEqual(-150, report["planned_positions"][-1])


if __name__ == "__main__":
    unittest.main()
