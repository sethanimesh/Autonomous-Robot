import math
import unittest
import numpy as np
from robot.mac.range_anchor import floor_point_pose,fit_floor_correction,corrected_surfaces


class RangeAnchorTests(unittest.TestCase):
    camera=dict(image_size=[640,480],intrinsics=dict(fx=420.,fy=420.,cx=320.,cy=240.),distortion=[])

    def scene(self):
        y,x=np.mgrid[270:460:5,20:620:10];x=x.ravel();y=y.ravel()
        pose=dict(height_m=.1524,pitch_degrees=4.,roll_degrees=6.)
        p,r=map(math.radians,[4.,6.]);v=(x-320)/420*math.sin(r)+(y-240)/420*math.cos(r)
        true_z=.1524/(v*math.cos(p)+math.sin(p))
        # inverse true depth = 4 / model depth - 1.2
        model_z=4/(1/true_z+1.2)
        return np.c_[x,y,model_z],pose

    def test_independent_pose_corrects_depth_bias_without_teaching_from_person_depth(self):
        samples,pose=self.scene();fit=fit_floor_correction(samples,self.camera,pose)
        self.assertAlmostEqual(4.,fit['inverse_scale'],places=6)
        self.assertAlmostEqual(-1.2,fit['inverse_offset'],places=6)
        p=math.radians(pose['pitch_degrees']);distance=.8636;true_z=distance/math.cos(p)
        person=[[320,240,4/(1/true_z+1.2)]]
        result=corrected_surfaces(person,self.camera,fit)
        self.assertAlmostEqual(distance,result['forward_m'][0],places=6)
        self.assertFalse(fit['validated'])

    def test_operator_floor_point_handles_roll_and_camera_forward_distance(self):
        h=.1524;d=.8636;p=math.radians(4.);r=math.radians(6.);x=260.
        v=math.tan(math.atan2(h,d)-p)
        y=240+420*(v-(x-320)/420*math.sin(r))/math.cos(r)
        pose=floor_point_pose([x,y],d,h,6.,self.camera)
        self.assertAlmostEqual(4.,pose['pitch_degrees'],places=5)
        self.assertFalse(pose['validated'])

    def test_mislabeled_floor_and_invalid_extrapolation(self):
        samples,pose=self.scene();samples[::8,2]*=.5
        fit=fit_floor_correction(samples,self.camera,pose)
        self.assertAlmostEqual(4.,fit['inverse_scale'],places=4)
        with self.assertRaisesRegex(ValueError,'extrapolates'):
            corrected_surfaces([[320,240,100]],self.camera,fit)
        with self.assertRaises(ValueError):fit_floor_correction(samples[:5],self.camera,pose)
        with self.assertRaises(ValueError):floor_point_pose([320,240],float('nan'),.1524,0,self.camera)


if __name__=='__main__':unittest.main()
