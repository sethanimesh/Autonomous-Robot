import math
import unittest

from robot.jetson.ev3_bridge.odometry import DifferentialOdometry
from robot.jetson.ev3_bridge.odometry import estimate_track_width
from robot.jetson.ev3_bridge.odometry import estimate_wheel_radius


class DifferentialOdometryTests(unittest.TestCase):
    def test_first_sample_only_establishes_baseline(self):
        odometry = DifferentialOdometry(0.03, 0.12)

        state = odometry.update(100, 200, 1.0)

        self.assertEqual(0.0, state["x"])
        self.assertEqual(0.0, state["heading"])

    def test_one_equal_revolution_moves_straight(self):
        odometry = DifferentialOdometry(0.03, 0.12)
        odometry.update(0, 0, 1.0)

        state = odometry.update(360, 360, 2.0)

        self.assertAlmostEqual(2.0 * math.pi * 0.03, state["x"])
        self.assertAlmostEqual(0.0, state["y"])
        self.assertAlmostEqual(0.0, state["heading"])
        self.assertAlmostEqual(2.0 * math.pi * 0.03, state["linear_velocity"])

    def test_opposite_wheels_turn_in_place(self):
        odometry = DifferentialOdometry(0.03, 0.12)
        odometry.update(0, 0, 1.0)

        state = odometry.update(-90, 90, 2.0)

        expected_heading = (math.pi * 0.03) / 0.12
        self.assertAlmostEqual(0.0, state["x"])
        self.assertAlmostEqual(expected_heading, state["heading"])

    def test_encoder_signs_are_applied(self):
        odometry = DifferentialOdometry(0.03, 0.12, left_sign=-1, right_sign=1)
        odometry.update(0, 0, 1.0)

        state = odometry.update(-360, 360, 2.0)

        self.assertGreater(state["x"], 0)
        self.assertAlmostEqual(0.0, state["heading"])


class CalibrationTests(unittest.TestCase):
    def test_wheel_radius_estimate_recovers_known_radius(self):
        distance = 2.0 * math.pi * 0.03

        radius = estimate_wheel_radius(distance, 360, 360)

        self.assertAlmostEqual(0.03, radius)

    def test_track_width_estimate_recovers_known_width(self):
        radius = 0.03
        yaw = math.pi / 2.0
        count = 90

        width = estimate_track_width(yaw, -count, count, radius)

        self.assertAlmostEqual(0.06, width)

    def test_invalid_calibration_input_is_rejected(self):
        with self.assertRaises(ValueError):
            estimate_wheel_radius(0.0, 10, 10)


if __name__ == "__main__":
    unittest.main()
