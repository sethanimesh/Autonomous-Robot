import json
import math
import os
import tempfile
import unittest

from robot.jetson.ev3_bridge.calibration import CalibrationError
from robot.jetson.ev3_bridge.calibration import calibration_result
from robot.jetson.ev3_bridge.calibration import encoder_deltas
from robot.jetson.ev3_bridge.calibration import load_report
from robot.jetson.ev3_bridge.calibration import motor_positions
from robot.jetson.ev3_bridge.calibration import validate_capture
from robot.jetson.ev3_bridge.calibration import validate_encoder_motion
from robot.jetson.ev3_bridge.calibration import write_report


def status(left, right):
    return {
        "motors": {
            "left": {"position": left},
            "right": {"position": right},
        }
    }


class CaptureHelperTests(unittest.TestCase):
    def test_extracts_positions_and_deltas(self):
        self.assertEqual((10, 20), motor_positions(status(10, 20)))
        self.assertEqual((30, -15), encoder_deltas(status(10, 20), status(40, 5)))

    def test_rejects_incomplete_status(self):
        with self.assertRaises(CalibrationError):
            motor_positions({"motors": {"left": {"position": 10}}})

    def test_capture_limits_are_enforced(self):
        validate_capture("straight", 0.1, 5.0)
        validate_capture("turn", -0.5, 1.0)
        for arguments in (
            ("sideways", 0.1, 1.0),
            ("straight", 0.0, 1.0),
            ("straight", 0.16, 1.0),
            ("turn", 1.01, 1.0),
            ("turn", 0.5, 5.01),
        ):
            with self.assertRaises(CalibrationError):
                validate_capture(*arguments)

    def test_encoder_motion_pattern_is_checked(self):
        self.assertTrue(validate_encoder_motion("straight", 20, 19))
        self.assertTrue(validate_encoder_motion("turn", -20, 19))
        for arguments in (
            ("straight", 20, -19),
            ("turn", 20, 19),
            ("straight", 9, 20),
        ):
            with self.assertRaises(CalibrationError):
                validate_encoder_motion(*arguments)


class CalibrationResultTests(unittest.TestCase):
    def test_calculates_wheel_radius_from_straight_capture(self):
        report = {
            "outcome": "success",
            "motion": "straight",
            "encoder_delta": {"left": 360, "right": 360},
            "encoder_counts_per_rev": 360,
            "wheel_radius_m": 0.03,
        }
        result = calibration_result(report, measured_distance_m=0.2)
        self.assertAlmostEqual(0.2 / (2.0 * math.pi), result["wheel_radius_m"])

    def test_calculates_track_width_from_turn_capture(self):
        report = {
            "outcome": "success",
            "motion": "turn",
            "encoder_delta": {"left": -180, "right": 180},
            "encoder_counts_per_rev": 360,
            "wheel_radius_m": 0.03,
        }
        result = calibration_result(report, measured_yaw_degrees=90.0)
        self.assertAlmostEqual(0.12, result["track_width_m"])

    def test_turn_can_use_refined_wheel_radius(self):
        report = {
            "outcome": "success",
            "motion": "turn",
            "encoder_delta": {"left": -180, "right": 180},
            "encoder_counts_per_rev": 360,
            "wheel_radius_m": 0.03,
        }
        result = calibration_result(
            report,
            measured_yaw_degrees=90.0,
            wheel_radius_m=0.025,
        )
        self.assertAlmostEqual(0.10, result["track_width_m"])

    def test_rejects_failed_capture(self):
        with self.assertRaises(CalibrationError):
            calibration_result({"outcome": "failure"}, measured_distance_m=1.0)

    def test_report_round_trip(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "nested", "report.json")
            write_report(path, {"outcome": "success", "value": 3})
            self.assertEqual(
                {"outcome": "success", "value": 3},
                load_report(path),
            )
            with open(path, "r") as source:
                self.assertEqual("success", json.load(source)["outcome"])


if __name__ == "__main__":
    unittest.main()
