import math
import unittest

from robot.jetson.navigation.local_planner import cable_safe_scan_headings
from robot.jetson.navigation.local_planner import LocalRoutePlanner
from robot.jetson.navigation.local_planner import RouteCandidate


def route(heading, clearance, confidence=0.9, known=0.95):
    return RouteCandidate(heading, clearance, confidence, known)


class CableSafeScanTests(unittest.TestCase):
    def test_complete_scan_returns_to_start_without_over_twisting(self):
        headings = cable_safe_scan_headings(30, 180)
        self.assertEqual(0, headings[0])
        self.assertEqual(0, headings[-1])
        self.assertIn(180, headings)
        self.assertIn(-180, headings)
        self.assertTrue(all(abs(value) <= 180 for value in headings))
        self.assertTrue(
            all(abs(right - left) <= 30 for left, right in zip(headings, headings[1:]))
        )

    def test_invalid_scan_geometry_is_rejected(self):
        for step, limit in ((0, 180), (100, 180), (30, 181), (40, 180)):
            with self.assertRaises(ValueError):
                cable_safe_scan_headings(step, limit)


class LocalRoutePlannerTests(unittest.TestCase):
    def setUp(self):
        self.planner = LocalRoutePlanner()

    def test_measured_footprint_produces_thirty_centimetre_corridor(self):
        self.assertAlmostEqual(0.30, self.planner.corridor_width_m)

    def test_prefers_aligned_route_when_evidence_is_equal(self):
        decision = self.planner.choose([route(-15, 0.30), route(0, 0.30), route(15, 0.30)])
        self.assertFalse(decision.blocked)
        self.assertEqual(0, decision.heading_degrees)
        self.assertAlmostEqual(0.10, decision.distance_m)

    def test_chooses_detour_when_target_corridor_is_blocked(self):
        decision = self.planner.choose(
            [route(-30, 0.40), route(0, 0.08), route(30, 0.25)],
            desired_heading_degrees=0,
        )
        self.assertFalse(decision.blocked)
        self.assertEqual(-30, decision.heading_degrees)

    def test_unknown_or_uncertain_route_fails_closed(self):
        decision = self.planner.choose(
            [route(0, 1.0, confidence=0.69), route(15, 1.0, known=0.84)]
        )
        self.assertTrue(decision.blocked)
        self.assertEqual(0.0, decision.distance_m)

    def test_safety_margin_reduces_commanded_distance(self):
        decision = self.planner.choose([route(0, 0.12)])
        self.assertFalse(decision.blocked)
        self.assertAlmostEqual(0.07, decision.distance_m)

    def test_non_finite_evidence_is_ignored(self):
        decision = self.planner.choose([route(0, math.nan)])
        self.assertTrue(decision.blocked)


if __name__ == "__main__":
    unittest.main()

