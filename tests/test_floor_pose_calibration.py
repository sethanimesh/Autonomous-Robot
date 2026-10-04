import math
import unittest

import numpy as np

from robot.mac.person_range import infer_floor_pose, floor_plane_orientation


class FloorPoseCalibrationTests(unittest.TestCase):
    intrinsics = dict(fx=220., fy=220., cx=160., cy=120.)

    def plane(self, pitch=12., height=.18, scale=1.):
        ys, xs = np.indices((240, 320))
        angle = math.radians(pitch)
        normal = np.array([0., math.cos(angle), math.sin(angle)])
        denominator = (ys-120.)/220.*normal[1]+normal[2]
        mask = (ys > 130) & (denominator > .1)
        depth = np.zeros_like(denominator)
        depth[mask] = height/denominator[mask]/scale
        return depth, mask

    def test_plane_tilt_ignores_uniform_metric_scale_error(self):
        for pitch in (-8., 0., 22.):
            for scale in (.65, 1., 1.5):
                depth, mask = self.plane(pitch=pitch, scale=scale)
                result = infer_floor_pose(depth, mask, self.intrinsics, .18)
                self.assertAlmostEqual(pitch, result['pitch_degrees'], places=5)
                self.assertAlmostEqual(scale, result['floor_scale_hint'], places=5)
                self.assertFalse(result['validated'])

    def test_furniture_mislabeled_as_floor_does_not_set_tilt(self):
        depth, mask = self.plane()
        depth[145:215, 20:105] *= .45
        result = infer_floor_pose(depth, mask, self.intrinsics, .18)
        self.assertAlmostEqual(12., result['pitch_degrees'], places=3)
        self.assertGreater(result['floor_inlier_fraction'], .7)

    def test_empty_narrow_or_nonplanar_floor_requests_another_view(self):
        depth, mask = self.plane()
        for bad in (np.zeros_like(mask), mask & (np.indices(mask.shape)[1] < 20)):
            with self.assertRaises(ValueError):
                infer_floor_pose(depth, bad, self.intrinsics, .18)
        random = np.random.default_rng(4).uniform(.1, 3., depth.shape)
        with self.assertRaises(ValueError):
            infer_floor_pose(random, mask, self.intrinsics, .18)

    def test_unmeasured_height_is_rejected(self):
        depth, mask = self.plane()
        for height in (None, float('nan'), 0):
            with self.assertRaises(ValueError):
                infer_floor_pose(depth, mask, self.intrinsics, height)

    def test_orientation_only_does_not_invent_physical_measurements(self):
        depth, mask = self.plane(pitch=8., scale=1.4)
        result = floor_plane_orientation(depth, mask, self.intrinsics)
        self.assertAlmostEqual(8., result['pitch_degrees'], places=5)
        self.assertNotIn('height_m', result)
        self.assertNotIn('floor_scale_hint', result)
        self.assertFalse(result['validated'])
