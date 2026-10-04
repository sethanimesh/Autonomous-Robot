import unittest

from robot.jetson.mission.target_gate import TargetGateError
from robot.jetson.mission.target_gate import build_target_observation
from robot.jetson.mission.target_gate import target_box_position


def confirmed_status():
    return {
        "state": "target_confirmed",
        "confirmation": {"confirmed": True, "hits": 3, "required": 3},
    }


class TargetGateTests(unittest.TestCase):
    def test_position_uses_largest_target_match_and_actual_image_size(self):
        self.assertEqual({'center_x_fraction': .75, 'center_y_fraction': .5},
                         target_box_position([(100, 100, 40, 50), (960, 360, 100, 120)], 1280, 720))
        for box in ((float('nan'), 100, 40, 50), (1500, 100, 40, 50), (100, 100, 40, -1)):
            self.assertIsNone(target_box_position([box], 1280, 720))
        self.assertIsNone(target_box_position([], 0, 0))

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
