import unittest

from robot.jetson.navigation.image_corridors import DEFAULT_CORRIDORS
from robot.jetson.navigation.image_corridors import evaluate_corridor
from robot.jetson.navigation.image_corridors import estimate_floor_horizon
from robot.jetson.navigation.image_corridors import semantic_route_candidates
from robot.jetson.navigation.local_planner import LocalRoutePlanner


class ImageCorridorTests(unittest.TestCase):
    def test_clear_floor_prefers_straight_route(self):
        mask = [[True for _ in range(120)] for _ in range(90)]
        candidates, evidence = semantic_route_candidates(mask)
        decision = LocalRoutePlanner().choose(candidates)
        self.assertFalse(decision.blocked)
        self.assertEqual(0, decision.heading_degrees)
        self.assertTrue(all(item.floor_fraction == 1.0 for item in evidence))

    def test_central_non_floor_object_forces_detour(self):
        mask = [[True for _ in range(120)] for _ in range(90)]
        for y in range(48, 70):
            for x in range(50, 76):
                mask[y][x] = False
        candidates, evidence = semantic_route_candidates(mask)
        decision = LocalRoutePlanner().choose(candidates)
        center = next(item for item in evidence if item.heading_degrees == 0)
        self.assertLess(center.floor_fraction, 0.90)
        self.assertFalse(decision.blocked)
        self.assertNotEqual(0, decision.heading_degrees)

    def test_unknown_pixels_fail_known_coverage(self):
        mask = [[None for _ in range(100)] for _ in range(80)]
        candidates, _ = semantic_route_candidates(mask)
        self.assertTrue(LocalRoutePlanner().choose(candidates).blocked)

    def test_invalid_mask_is_rejected(self):
        with self.assertRaises(ValueError):
            evaluate_corridor([], DEFAULT_CORRIDORS[0])
        with self.assertRaises(ValueError):
            evaluate_corridor([[True], [True, False]], DEFAULT_CORRIDORS[0])

    def test_floor_horizon_skips_a_wall_but_keeps_floor_obstacles(self):
        mask = [[False for _ in range(120)] for _ in range(100)]
        for y in range(55, 100):
            for x in range(120):
                mask[y][x] = True
        for y in range(60, 82):
            for x in range(50, 76):
                mask[y][x] = False
        self.assertAlmostEqual(0.5556, estimate_floor_horizon(mask), places=3)
        candidates, evidence = semantic_route_candidates(mask)
        decision = LocalRoutePlanner().choose(candidates)
        center = next(item for item in evidence if item.heading_degrees == 0)
        self.assertLess(center.floor_fraction, 0.95)
        self.assertFalse(decision.blocked)
        self.assertNotEqual(decision.heading_degrees, 0)

    def test_lower_quarter_floor_is_usable_but_a_thin_bottom_strip_is_not(self):
        for first_floor_row, blocked in [(365, False), (432, True)]:
            mask = [[y >= first_floor_row for x in range(120)] for y in range(480)]
            candidates, _ = semantic_route_candidates(mask)
            self.assertEqual(blocked, LocalRoutePlanner().choose(candidates).blocked)

    def test_no_detected_floor_fails_closed(self):
        mask = [[False for _ in range(120)] for _ in range(90)]
        candidates, _ = semantic_route_candidates(mask)
        self.assertTrue(LocalRoutePlanner().choose(candidates).blocked)

    def test_far_object_does_not_block_a_ten_centimetre_near_floor_step(self):
        mask = [[False for _ in range(120)] for _ in range(100)]
        for y in range(30, 100):
            for x in range(120):
                mask[y][x] = True
        for y in range(32, 48):
            for x in range(42, 78):
                mask[y][x] = False
        candidates, _ = semantic_route_candidates(mask)
        decision = LocalRoutePlanner().choose(candidates)
        self.assertFalse(decision.blocked)
        self.assertEqual(0, decision.heading_degrees)


if __name__ == "__main__":
    unittest.main()
