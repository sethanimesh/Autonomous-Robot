import unittest

from robot.jetson.mission.target_gate import TargetGateError
from robot.jetson.mission.target_gate import build_target_observation


def confirmed_status():
    return {
        "state": "target_confirmed",
        "confirmation": {"confirmed": True, "hits": 3, "required": 3},
    }


class TargetGateTests(unittest.TestCase):
    def test_confirmed_status_requires_current_box(self):
        result = build_target_observation(confirmed_status(), 0.1, 0.05, [])
        self.assertFalse(result.confirmed)

    def test_box_height_becomes_image_fraction(self):
        result = build_target_observation(confirmed_status(), 0.1, 0.05, [240])
        self.assertTrue(result.confirmed)
        self.assertEqual(result.box_height_fraction, 0.5)

    def test_largest_current_match_is_used(self):
        result = build_target_observation(
            confirmed_status(), 0.1, 0.05, [100, 300, 200]
        )
        self.assertEqual(result.box_height_fraction, 0.625)

    def test_stale_recognition_cannot_confirm(self):
        result = build_target_observation(confirmed_status(), 3.0, 0.1, [300])
        self.assertFalse(result.confirmed)

    def test_searching_status_cannot_confirm(self):
        status = confirmed_status()
        status["state"] = "searching"
        result = build_target_observation(status, 0.1, 0.05, [300])
        self.assertFalse(result.confirmed)

    def test_invalid_geometry_is_rejected(self):
        with self.assertRaises(TargetGateError):
            build_target_observation(confirmed_status(), 0.1, 0.1, [100], 0)


if __name__ == "__main__":
    unittest.main()
