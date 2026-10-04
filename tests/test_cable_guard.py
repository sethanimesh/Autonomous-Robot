import unittest

from robot.jetson.navigation.cable_guard import CableLimitError
from robot.jetson.navigation.cable_guard import project_cable_turn
from robot.jetson.navigation.cable_guard import record_cable_turn
from robot.jetson.navigation.cable_guard import relative_scan_headings_within_cable_limit
from robot.jetson.navigation.local_planner import cable_safe_scan_headings


class CableGuardTests(unittest.TestCase):
    def test_turn_is_rejected_before_crossing_the_absolute_envelope(self):
        self.assertEqual(105.0, project_cable_turn(80, 25, 120, margin=5))
        with self.assertRaisesRegex(CableLimitError, "exceeds"):
            project_cable_turn(100, 20, 120, margin=5)

    def test_measured_turn_updates_absolute_heading(self):
        self.assertEqual(42.0, record_cable_turn(20, 20, 22, 120))
        with self.assertRaisesRegex(CableLimitError, "opposite"):
            record_cable_turn(20, 15, -14, 120)
        self.assertEqual(117, record_cable_turn(110, 5, 7, 120, 8, margin=5))
        with self.assertRaisesRegex(CableLimitError, "exceeds"):
            record_cable_turn(110, 5, 11, 120, 8, margin=5)

    def test_live_reserve_excursion_continues_inward_without_reset(self):
        heading = record_cable_turn(-65, -15, -15.3428846881, 90, 25, margin=10)
        self.assertAlmostEqual(-80.3428846881, heading)
        self.assertAlmostEqual(heading + .1, project_cable_turn(heading, .1, 90, 10))
        self.assertAlmostEqual(heading + 25, project_cable_turn(heading, 25, 90, 10))
        with self.assertRaises(CableLimitError):
            project_cable_turn(heading, -1, 90, 10)
        with self.assertRaises(CableLimitError):
            project_cable_turn(heading, 161, 90, 10)
        safe = relative_scan_headings_within_cable_limit(
            cable_safe_scan_headings(15, 45), heading, 90, 10)
        self.assertEqual(0, safe[0])
        self.assertAlmostEqual(0, heading + safe[-1])
        self.assertTrue(all(abs(heading + delta) <= 80 for delta in safe[1:]))

    def test_local_scan_is_clipped_near_one_cable_edge_and_returns(self):
        relative = cable_safe_scan_headings(15, 45)

        safe = relative_scan_headings_within_cable_limit(
            relative, initial_heading_degrees=100, limit_degrees=120, margin=5
        )

        self.assertNotIn(30.0, safe)
        self.assertIn(-45.0, safe)
        self.assertEqual(0.0, safe[0])
        self.assertEqual(0.0, safe[-1])
        self.assertTrue(all(abs(100 + value) <= 115 for value in safe))


if __name__ == "__main__":
    unittest.main()
