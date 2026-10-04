import ast
import base64
import copy
import hashlib
import io
import json
from pathlib import Path
from types import SimpleNamespace as NS
import threading
import time
import unittest
from unittest.mock import patch

import cv2
import numpy as np
import yaml

from robot.jetson.perception.range_capture import capture_pose, same_capture_pose
from robot.mac.person_range import camera_pose, head_fraction, MODEL
from robot.mac.range_diagnostics import compare_range_models, capture_diagnostic


def hardware():
    head = dict(reference_id='boot', homed=True, position=19, down_position=17,
                up_position=3, settle_tolerance=4, moving=False)
    motors = {side: dict(generation=side+'-boot', position=19 if side=='tool' else 100,
                        speed=0, commanded_speed=0, state=[]) for side in ('left','right','tool')}
    return head, dict(status='ok', tool_reference_id='boot', motors=motors)


class RangeCaptureTests(unittest.TestCase):
    def test_capture_rejects_stale_feedback_stopped_command_pending_and_changed_reference(self):
        head, robot = hardware()
        self.assertIsNotNone(capture_pose(head, robot, .1, .2))
        self.assertIsNone(capture_pose(head, robot, 8, .1))
        self.assertIsNone(capture_pose(head, robot, .1, None))
        robot['motors']['left']['commanded_speed'] = 100
        self.assertIsNone(capture_pose(head, robot, .1, .1))
        head, robot = hardware();robot['tool_reference_id'] = 'rebooted'
        self.assertIsNone(capture_pose(head, robot, .1, .1))
        for field,value in (('motors',None),('motors',{'left':None})):
            head,robot=hardware();robot[field]=value
            self.assertIsNone(capture_pose(head,robot,.1,.1))
        head,robot=hardware();robot['motors']['tool']['state']=None
        self.assertIsNone(capture_pose(head,robot,.1,.1))

    def test_image_pose_cannot_be_replaced_by_feedback_after_a_move(self):
        head, robot = hardware();before = capture_pose(head, robot, .1, .1)
        self.assertTrue(same_capture_pose(before, capture_pose(head, robot, .1, .1)))
        head['position'] += 5;robot['motors']['tool']['position'] += 5
        self.assertEqual(19, before['head']['position'])
        self.assertFalse(same_capture_pose(before, capture_pose(head, robot, .1, .1)))
        head, robot = hardware();robot['motors']['left']['generation'] = 'rebooted'
        self.assertFalse(same_capture_pose(before, capture_pose(head, robot, .1, .1)))

    def test_endpoint_returns_exact_image_geometry_and_rejects_stale_or_moved_capture(self):
        source = Path('robot/jetson/perception/enrollment_console.py').read_text()
        tree = ast.parse(source)
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name=='EnrollmentNode')
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name=='family_range_frame')
        definition = ast.ClassDef(name='Harness',bases=[],keywords=[],decorator_list=[],body=[method])
        namespace = dict(time=time, capture_pose=capture_pose, same_capture_pose=same_capture_pose,
                         cv2=cv2,base64=base64,hashlib=hashlib)
        exec(compile(ast.fix_missing_locations(ast.Module(body=[definition],type_ignores=[])), '<range-capture>', 'exec'), namespace)
        node=namespace['Harness']();head,robot=hardware()
        node.lock=threading.Lock();node.camera_head_status=head;node.robot_status=robot
        node.camera_head_status_time=node.robot_status_time=time.monotonic()
        node.args=NS(camera_intrinsics_file='config/camera_calibration.yaml')
        key=['camera',10,0]
        node.family_status=dict(frame_key=key,body_box=[10,10,200,200],identity_confirmed=True,
                                profile_id='mom',track_id='track')
        pose=capture_pose(head,robot,.1,.1)
        node.raw_frames={tuple(key):(np.zeros((480,640,3),np.uint8),time.monotonic(),pose)}
        result=node.family_range_frame()
        self.assertEqual(1,result['capture_schema']);self.assertEqual(pose,result['pose_binding'])
        self.assertEqual([640,480],result['camera']['image_size'])
        self.assertEqual(result['image_sha256'],hashlib.sha256(base64.b64decode(result['image'])).hexdigest())
        node.args.camera_intrinsics_file='/missing-camera-calibration.yaml'
        with self.assertRaisesRegex(ValueError,'calibration is unavailable'):node.family_range_frame()
        node.args.camera_intrinsics_file='config/camera_calibration.yaml'
        node.camera_head_status_time -= 10
        with self.assertRaises(ValueError):node.family_range_frame()
        node.camera_head_status_time=time.monotonic()
        node.camera_head_status['position'] += 3;node.robot_status['motors']['tool']['position'] += 3
        with self.assertRaises(ValueError):node.family_range_frame()


class RangeGeometryComparisonTests(unittest.TestCase):
    def scene(self):
        ys,xs=np.indices((240,320));floor=ys>130
        depth=np.zeros((240,320));depth[floor]=4*.30/((ys[floor]-120)/220.)
        person=(xs>=120)&(xs<200)&(ys>=40)&(ys<186)
        depth[person]=4*(34*.0254+.1);floor[person]=False
        camera=dict(image_size=[320,240],intrinsics=dict(fx=220,fy=220,cx=160,cy=120),distortion=[])
        head,_=hardware()
        return depth,dict(floor=floor,person=person),[120,40,200,186],camera,head

    def test_measured_height_and_front_offset_separate_model_scale_from_origin_error(self):
        result=compare_range_models(*self.scene(),height_m=.30,front_offset_m=.1,measured_gap_m=34*.0254)
        cases=result['cases']
        self.assertAlmostEqual(4*.9636,cases['raw_model']['distance_m'],places=5)
        self.assertAlmostEqual(.9636/2,cases['historical_15cm_prior']['distance_m'],places=5)
        self.assertAlmostEqual(.9636,cases['measured_height']['distance_m'],places=5)
        self.assertAlmostEqual(.8636,cases['measured_height_front_gap']['distance_m'],places=5)
        self.assertTrue(result['comparison']['point_within_target'])
        self.assertFalse(result['validated']);self.assertFalse(result['comparison']['formal_accuracy_validation'])

    def test_different_origins_or_absent_geometry_do_not_claim_accuracy(self):
        result=compare_range_models(*self.scene(),height_m=.30,measured_gap_m=.8636)
        self.assertFalse(result['comparison']['available'])
        self.assertNotIn('measured_height_front_gap',result['cases'])
        result=compare_range_models(*self.scene(),height_m=.15,front_offset_m=.1,measured_gap_m=.8636)
        self.assertFalse(result['comparison']['point_within_target'])

    def test_perfect_floor_fit_cannot_validate_depth_with_inverse_depth_bias(self):
        # A model error can mimic camera tilt while preserving a perfectly
        # planar floor. Physical height alone cannot distinguish these cases.
        ys,xs=np.indices((240,320));height=.1524;gap=.8636;offset=.10
        floor=ys>130;depth=np.zeros((240,320))
        depth[floor]=height/((ys[floor]-120)/220.)
        person=(xs>=120)&(xs<200)&(ys>=40)&(ys<155)
        depth[person]=gap+offset;floor[person]=False
        camera=dict(image_size=[320,240],intrinsics=dict(fx=220,fy=220,cx=160,cy=120),distortion=[])
        head=dict(position=0,down_position=0,up_position=-30)
        def compare(values):
            return compare_range_models(values,dict(floor=floor,person=person),[120,40,200,155],
                camera,head,height_m=height,front_offset_m=offset,measured_gap_m=gap)
        baseline=compare(depth)
        self.assertAlmostEqual(gap,baseline['cases']['measured_height_front_gap']['distance_m'])
        valid=depth>0;depth[valid]=1/(1/depth[valid]+1.6)
        biased=compare(depth)
        self.assertGreater(biased['floor_fit']['floor_inlier_fraction'],.99)
        self.assertLess(biased['floor_fit']['floor_residual_fraction'],.001)
        self.assertGreater(biased['floor_fit']['pitch_degrees'],10)
        self.assertGreater(abs(biased['comparison']['signed_error_m']),.5)
        self.assertFalse(biased['comparison']['point_within_target'])
        self.assertFalse(biased['validated'])

    def test_invalid_measurements_or_missing_person_cannot_fabricate_a_result(self):
        for kwargs in (dict(height_m=float('nan')),dict(height_m=True),dict(front_offset_m=.1),dict(measured_gap_m=-1)):
            with self.assertRaises(ValueError):compare_range_models(*self.scene(),**kwargs)
        depth,masks,box,camera,head=self.scene();masks['person'][:]=False
        result=compare_range_models(depth,masks,box,camera,head,height_m=.3,front_offset_m=.1,measured_gap_m=.8)
        self.assertTrue(all(not c['available'] for c in result['cases'].values()))
        self.assertFalse(result['comparison']['available'])

    def test_settling_is_consistent_for_capture_approximation_and_measured_geometry(self):
        head,_=hardware();self.assertEqual(0,head_fraction(head))
        calibration=dict(schema=1,model=MODEL,validated=True,validated_head_fraction=[0,1],
                         head_poses=[dict(fraction=f,height_m=.30,pitch_degrees=0,front_offset_m=.1) for f in (0,1)])
        self.assertEqual(0,camera_pose(calibration,head)['fraction'])
        for change in (dict(position=25),dict(settle_tolerance=float('nan')),dict(position=True)):
            with self.assertRaises(ValueError):head_fraction(dict(head,**change))

    def test_capture_runs_models_once_and_retains_only_numbers_and_bindings(self):
        depth,masks,box,camera,head=self.scene();_,robot=hardware()
        jpeg=cv2.imencode('.jpg',np.zeros((240,320,3),np.uint8))[1].tobytes()
        capture=dict(capture_schema=1,image=base64.b64encode(jpeg).decode(),image_sha256=hashlib.sha256(jpeg).hexdigest(),
                     camera=camera,head=head,pose_binding=capture_pose(head,robot,.1,.1),
                     track=dict(body_box=box,frame_key=['cam',1,0],profile_id='mom',track_id='t'))
        calls=[]
        backend=NS(infer=lambda image:calls.append('depth') or depth)
        segmentation=NS(device='cpu',infer=lambda *a,**kw:calls.append('masks') or masks)
        responses=[io.BytesIO(json.dumps(dict(robot_ready=True,mission={},manual_drive={})).encode()),
                   io.BytesIO(json.dumps(capture).encode())]
        with patch('robot.mac.range_diagnostics.urlopen',side_effect=responses):
            result=capture_diagnostic('http://test',height_m=.3,backend=backend,segmentation=segmentation)
        self.assertEqual(['depth','masks'],calls)
        self.assertNotIn('image',result);self.assertNotIn('depth',result)
        self.assertEqual(capture['image_sha256'],result['frame_sha256'])
        self.assertFalse(result['commands_sent']);self.assertFalse(result['images_saved'])


if __name__=='__main__':unittest.main()
