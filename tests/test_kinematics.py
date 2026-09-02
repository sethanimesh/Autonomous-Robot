import unittest

from robot.jetson.ev3_bridge.kinematics import twist_to_motor_speeds


class KinematicsTests(unittest.TestCase):
    def test_forward_gives_equal_speeds(self):
        left, right = twist_to_motor_speeds(0.03, 0.0, 0.03, 0.12)

        self.assertEqual(left, right)
        self.assertGreater(left, 0)

    def test_turn_in_place_gives_opposite_speeds(self):
        left, right = twist_to_motor_speeds(0.0, 1.0, 0.03, 0.12)

        self.assertEqual(left, -right)
        self.assertLess(left, 0)
        self.assertGreater(right, 0)

    def test_speed_limit_is_applied(self):
        left, right = twist_to_motor_speeds(99.0, 0.0, 0.03, 0.12, max_motor_speed=80)

        self.assertEqual(80, left)
        self.assertEqual(80, right)

    def test_motor_signs_are_applied(self):
        left, right = twist_to_motor_speeds(
            0.03, 0.0, 0.03, 0.12, left_sign=-1, right_sign=1
        )

        self.assertLess(left, 0)
        self.assertGreater(right, 0)

    def test_invalid_geometry_is_rejected(self):
        with self.assertRaises(ValueError):
            twist_to_motor_speeds(0.1, 0.0, 0.0, 0.12)


if __name__ == "__main__":
    unittest.main()
