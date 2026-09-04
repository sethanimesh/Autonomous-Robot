import unittest

from robot.jetson.navigation.image_corridors import DEFAULT_CORRIDORS
from robot.jetson.navigation.image_corridors import evaluate_corridor
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


if __name__ == "__main__":
    unittest.main()

