import unittest

from robot.jetson.mission.bounded_target_scan import incremental_scan_turns
from robot.jetson.mission.bounded_target_scan import target_is_confirmed
from robot.jetson.navigation.local_planner import cable_safe_scan_headings


class BoundedTargetScanTests(unittest.TestCase):
    def test_incremental_turns_are_small_and_unwind(self):
        headings = cable_safe_scan_headings(30, 180)
        turns = incremental_scan_turns(headings)
        self.assertTrue(all(abs(value) == 30 for value in turns))
        self.assertEqual(sum(turns), 0)
        self.assertEqual(headings[-1], 0)

    def test_target_requires_current_box_and_fresh_confirmation(self):
        value = {
            "ok": True,
            "confirmed": True,
            "age_seconds": 0.1,
            "box_height_fraction": 0.4,
        }
        self.assertTrue(target_is_confirmed(value))
        value["age_seconds"] = 1.0
        self.assertFalse(target_is_confirmed(value))
        value["age_seconds"] = 0.1
        value["box_height_fraction"] = 0.0
        self.assertFalse(target_is_confirmed(value))

    def test_invalid_target_payload_is_not_confirmation(self):
        for value in (None, {}, {"ok": False}, {"ok": True, "age_seconds": "x"}):
            self.assertFalse(target_is_confirmed(value))


if __name__ == "__main__":
    unittest.main()
