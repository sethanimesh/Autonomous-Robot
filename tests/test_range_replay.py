import copy
import unittest
from unittest.mock import patch
import numpy as np

from robot.mac.range_replay import numeric_samples, surface_record, replay_documents


def record():
    ys,xs=np.mgrid[70:95,40:65]
    person=np.column_stack((xs.ravel(),ys.ravel(),np.full(xs.size,.9))).tolist()
    ys,xs=np.mgrid[90:100,0:100]
    floor=np.column_stack((xs.ravel(),ys.ravel(),.15/((ys.ravel()-50)/100.))).tolist()
    return dict(camera=dict(image_size=[100,100],intrinsics=dict(fx=100,fy=100,cx=50,cy=50),distortion=[]),
        capture=dict(image_sha256='frame-a',frame_key=['cam',1,0],pose_binding=dict(
            head=dict(reference_id='boot',position=8),
            motors={side:dict(generation=side,position=8 if side=='tool' else 0) for side in ('left','right','tool')})),
        operator_lower_height_m=.1524,operator_gap_m=.8636,front_offset_m=None,
        people=[dict(box=[40,60,65,95],person_depth_samples=person,floor_depth_samples=floor,
                     floor_fit=dict(pitch_degrees=10.,plane_distance_model_units=.6))])


class RangeReplayTests(unittest.TestCase):
    def test_numeric_replay_does_not_label_bottom_pixels_as_feet_or_validate_error(self):
        value=replay_documents([('one',record())])['results'][0]
        self.assertTrue(value['ok'])
        self.assertFalse(value['sampled_floor_estimate']['lowest_band_is_identified_foot'])
        self.assertFalse(value['comparison']['available'])
        self.assertTrue(value['sample_at_detector_bottom'])
        self.assertFalse(value['detector_bottom_contact_proves_clipping'])
        self.assertFalse(value['mask_extension_replay_available'])

    def test_bad_record_does_not_abort_other_replays_and_nan_is_rejected(self):
        bad=record();bad['people'][0]['person_depth_samples'][0][2]=float('nan')
        value=replay_documents([('bad',bad),('good',record())])
        self.assertFalse(value['results'][0]['ok'])
        self.assertTrue(value['results'][1]['ok'])

    def test_error_requires_the_same_measured_surface_and_origin(self):
        a=record();a['front_offset_m']=.1
        self.assertFalse(replay_documents([('a',a)])['results'][0]['comparison']['available'])
        a['measured_region']='nearest_visible_surface'
        value=replay_documents([('a',a)])['results'][0]
        self.assertTrue(value['comparison']['available'])
        self.assertFalse(value['comparison']['formal_accuracy_validation'])

    def test_scene_dependent_pitch_is_compared_only_for_same_camera_and_pose(self):
        for fault in (None,'boot','head','track','intrinsics','frame'):
            a=record();b=copy.deepcopy(a);b['capture']['frame_key'][1]=2
            b['people'][0]['floor_fit']['pitch_degrees']=25.
            if fault=='boot':b['capture']['pose_binding']['head']['reference_id']='new'
            if fault=='head':b['capture']['pose_binding']['head']['position']=20
            if fault=='track':b['capture']['pose_binding']['motors']['left']['position']=50
            if fault=='intrinsics':b['camera']['intrinsics']['fy']=110
            if fault=='frame':b['capture']['frame_key']=a['capture']['frame_key']
            value=replay_documents([('a',a),('b',b)])['same_pose_comparisons']
            with self.subTest(fault=fault):
                if fault:self.assertEqual([],value)
                else:
                    self.assertEqual(15.,value[0]['depth_pitch_change_degrees'])
                    self.assertFalse(value[0]['proves_physical_tilt_change'])

    def test_old_geometry_cannot_be_applied_after_reboot(self):
        a=record();geometry=dict(camera=a['camera'],pose_binding=copy.deepcopy(a['capture']['pose_binding']))
        geometry['pose_binding']['head']['reference_id']='old'
        with patch('robot.mac.range_replay.fit_floor_correction',side_effect=AssertionError('wrong geometry reached fitter')):
            value=replay_documents([('a',a)],geometry)['results'][0]
        self.assertFalse(value['anchored_candidate']['available'])

    def test_missing_pose_metadata_preserves_individual_replays(self):
        a=record();b=record();b['capture']['frame_key'][1]=2
        b['capture']['pose_binding']={'head':{}}
        value=replay_documents([('a',a),('b',b)])
        self.assertTrue(all(r['ok'] for r in value['results']))
        self.assertEqual([],value['same_pose_comparisons'])

    def test_capture_keeps_extension_context_without_storing_images(self):
        mask=np.zeros((100,100),bool);mask[20:80,40:60]=True
        mask[80:90,45:55]=True
        value=surface_record(np.ones(mask.shape),dict(person=mask,floor=~mask),[40,20,60,80])
        self.assertEqual(100,value['surface_evidence']['extension_pixels'])
        self.assertEqual(89,max(s[1] for s in value['person_depth_samples']))
        self.assertEqual(79,max(s[1] for s in value['detector_person_depth_samples']))
        self.assertNotIn('image',value)
        self.assertEqual(value['person_depth_samples'],surface_record(np.ones(mask.shape),dict(person=mask,floor=~mask),[40,20,60,80])['person_depth_samples'])

    def test_empty_and_nonfinite_pixels_do_not_produce_fake_samples(self):
        depth=np.full((10,10),float('nan'))
        self.assertEqual([],numeric_samples(depth,np.ones_like(depth,dtype=bool)))


if __name__=='__main__': unittest.main()
