import unittest

from robot.jetson.navigation.closed_loop_detour import DetourError
from robot.jetson.navigation.closed_loop_detour import center_floor_fraction
from robot.jetson.navigation.closed_loop_detour import validate_route_result


class ClosedLoopDetourTests(unittest.TestCase):
    def valid_result(self):
        return {
            "ok": True,
            "result_age_seconds": 0.02,
            "decision": {
                "blocked": False,
                "heading_degrees": -30,
                "distance_m": 0.10,
            },
            "camera_head": {
                "available": True,
                "homed": True,
                "calibrated": True,
                "moving": False,
                "homing": False,
                "position": 19,
                "down_position": 17,
            },
            "evidence": [
                {"heading_degrees": -30, "floor_fraction": 0.97},
                {"heading_degrees": 0, "floor_fraction": 0.93},
            ],
        }

    def test_valid_bounded_decision(self):
        self.assertEqual(validate_route_result(self.valid_result()), (-30.0, 0.1))

    def test_stale_result_is_rejected(self):
        value = self.valid_result()
        value["result_age_seconds"] = 2.0
        with self.assertRaisesRegex(DetourError, "stale"):
            validate_route_result(value)

    def test_blocked_result_is_rejected(self):
        value = self.valid_result()
        value["decision"]["blocked"] = True
        with self.assertRaisesRegex(DetourError, "blocked"):
            validate_route_result(value)

    def test_overlong_motion_is_rejected(self):
        value = self.valid_result()
        value["decision"]["distance_m"] = 0.20
        with self.assertRaisesRegex(DetourError, "distance"):
            validate_route_result(value)

    def test_unstable_camera_head_is_rejected(self):
        value = self.valid_result()
        value["camera_head"]["moving"] = True
        with self.assertRaisesRegex(DetourError, "camera head"):
            validate_route_result(value)

    def test_center_fraction(self):
        self.assertEqual(center_floor_fraction(self.valid_result()), 0.93)


if __name__ == "__main__":
    unittest.main()
