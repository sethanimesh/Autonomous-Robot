import unittest
import numpy as np

from robot.mac.person_range import estimate_range, person_surface


class PersonSurfaceTests(unittest.TestCase):
    def scene(self):
        person = np.zeros((120,160), bool)
        person[20:80,60:100] = True
        depth = np.full(person.shape, .9)
        ys = np.indices(person.shape)[0]
        depth[ys>60] = .18/((ys[ys>60]-60)/100.)
        depth[person] = .9
        floor = np.zeros_like(person); floor[100:] = True
        camera = dict(intrinsics=dict(fx=100,fy=100,cx=80,cy=60),
                      depth_scale=1.,near_error_m=.1,far_relative_error=.2,validated=False)
        pose = dict(height_m=.18,pitch_degrees=0.,front_offset_m=0.,fraction=0.)
        return person,depth,floor,camera,pose

    def test_connected_foot_outside_box_contributes_nearer_surface(self):
        person,depth,floor,camera,pose = self.scene()
        person[80:91,65:85] = True; depth[80:91,65:85] = .6
        floor[91:,65:85] = True
        value = estimate_range(depth,person,floor,[60,20,100,80],camera,pose)
        self.assertAlmostEqual(.6,value['distance_m'])
        self.assertEqual(220,value['surface_evidence']['extension_pixels'])
        self.assertTrue(value['feet_checked'])

    def test_unconnected_nearby_person_or_furniture_is_not_added(self):
        person,depth,floor,camera,pose = self.scene()
        person[82:90,70:95] = True; depth[82:90,70:95] = .2
        value = estimate_range(depth,person,floor,[60,20,100,80],camera,pose)
        self.assertAlmostEqual(.9,value['distance_m'])
        self.assertEqual(0,value['surface_evidence']['extension_pixels'])

    def test_large_connected_region_cannot_claim_the_other_person(self):
        person,*_ = self.scene()
        person[50:115,100:155] = True
        selected, evidence = person_surface(person,[60,20,100,80])
        self.assertFalse(selected[:,100:].any())
        self.assertFalse(evidence['unique_component'])
        self.assertTrue(evidence['lower_continuation'])

    def test_unfinished_lower_extension_does_not_confirm_feet(self):
        person,depth,floor,camera,pose = self.scene()
        person[80:118,65:75] = True
        floor[92:,75:100] = True
        value = estimate_range(depth,person,floor,[60,20,100,80],camera,pose)
        self.assertTrue(value['surface_evidence']['lower_continuation'])
        self.assertFalse(value['feet_checked'])
        self.assertEqual('lower',value['next_view'])

    def test_floor_at_unrelated_box_corner_does_not_confirm_feet(self):
        person,depth,floor,camera,pose = self.scene()
        floor[:] = False; floor[80:100,100:120] = True
        value = estimate_range(depth,person,floor,[40,20,120,80],camera,pose)
        self.assertFalse(value['feet_checked'])

    def test_image_edge_remains_clipped_even_when_depth_resembles_ground(self):
        person,depth,floor,camera,pose = self.scene()
        person[80:120,65:75] = True
        value = estimate_range(depth,person,floor,[60,20,100,120],camera,pose)
        self.assertTrue(value['surface_evidence']['image_bottom_clipped'])
        self.assertFalse(value['feet_checked'])


if __name__ == '__main__': unittest.main()
