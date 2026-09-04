import unittest

from robot.jetson.mission.autonomous_find import heading_after_detour
from robot.jetson.mission.autonomous_find import heading_after_relative_scan
from robot.jetson.mission.autonomous_find import target_height
from robot.jetson.mission.autonomous_find import target_is_at_standoff


class AutonomousFindTests(unittest.TestCase):
    def test_only_a_confirmed_finite_target_has_height(self):
        self.assertIsNone(target_height({}))
        self.assertIsNone(
            target_height(
                {"target_observation": {"confirmed": False, "box_height_fraction": 0.8}}
            )
        )
        self.assertEqual(
            0.32,
            target_height(
                {"target_observation": {"confirmed": True, "box_height_fraction": 0.32}}
            ),
        )

    def test_tested_thirty_percent_face_height_is_standoff(self):
        report = {
            "target_observation": {"confirmed": True, "box_height_fraction": 0.328}
        }
        self.assertTrue(target_is_at_standoff(report))

    def test_cable_heading_tracks_scan_and_detour_turns(self):
        self.assertEqual(35.0, heading_after_relative_scan(20, {"target_heading_degrees": 15}))
        self.assertEqual(5.0, heading_after_detour(20, {"turned_degrees": 15}))


if __name__ == "__main__":
    unittest.main()
