import unittest

from robot.jetson.ev3_bridge.kinematics import twist_to_motor_speeds


class KinematicsTests(unittest.TestCase):
    def test_phase6_speeds_reach_controller_without_old_bridge_clipping(self):
        import math
        from robot.ev3.server.ev3_server import DEFAULT_DRIVE_SPEED_LIMIT
        from robot.jetson.mission.bounded_target_scan import parse_args as scan_args
        from robot.jetson.navigation.closed_loop_detour import parse_args as route_args
        scan = scan_args([])
        route = route_args(['--route-url', 'http://unused/route'])
        self.assertEqual(.6, scan.turn_speed)
        self.assertEqual(scan.turn_speed, route.turn_speed)
        self.assertEqual(.06, route.drive_speed)
        radius, width = .0144504, .182557
        for linear, angular in ((route.drive_speed, 0.), (0., scan.turn_speed)):
            actual = twist_to_motor_speeds(linear, angular, radius, width)
            expected = tuple(round(speed / radius * 180 / math.pi) for speed in
                             (linear-angular*width/2, linear+angular*width/2))
            self.assertEqual(expected, actual)
            self.assertGreater(max(abs(v) for v in actual), 120)
            self.assertLessEqual(max(abs(v) for v in actual), DEFAULT_DRIVE_SPEED_LIMIT)

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
