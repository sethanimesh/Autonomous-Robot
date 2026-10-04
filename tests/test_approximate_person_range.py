import base64
import copy
import hashlib
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import numpy as np
from PIL import Image

from robot.mac.person_range import PersonRangeService,MODEL
from robot.jetson.mission.person_approach import approach_decision


def observation(distance=.18, **changes):
    value=dict(identity_confirmed=True,profile_id='mom',track_id='t',age_seconds=.1,
        identity_source='clothing',clothing_approach_enabled=True,approximate_approach_enabled=True,
        range=dict(distance_m=distance,lower_m=max(0,distance-.15),upper_m=distance+.15,
            available=True,validated=False,mode='approximate',source='metric_depth_floor_estimate',
            distance_reference='camera_ground_projection',feet_checked=True,consistent_samples=2,
            track_id='t',profile_id='mom',age_seconds=.2))
    value['range'].update(changes)
    return value


class ApproximateRangeTests(unittest.TestCase):
    def scene(self):
        self.clock=[10.]
        ys,xs=np.indices((240,320))
        self.floor=ys>130
        self.depth=np.zeros((240,320))
        self.depth[self.floor]=.18/((ys[self.floor]-120)/220.)
        self.person=(xs>=120)&(xs<200)&(ys>=40)&(ys<186)
        self.depth[self.person]=.6;self.floor[self.person]=False
        output=io.BytesIO();Image.new('RGB',(320,240),(100,90,80)).save(output,format='JPEG')
        jpeg=output.getvalue()
        self.request=dict(mode='approximate',calibration={},image=base64.b64encode(jpeg).decode(),
            binding=dict(image_sha256=hashlib.sha256(jpeg).hexdigest(),track_id='t',profile_id='mom'),
            box=[120,40,200,186],head=dict(position=52,down_position=51,up_position=24,
                settle_tolerance=4,reference_id='boot',moving=False),
            camera=dict(image_size=[320,240],intrinsics=dict(fx=220,fy=220,cx=160,cy=120),
                        distortion=[],nominal_lower_height_m=.18))
        service=PersonRangeService(NS(device='cpu',infer=lambda *a,**k:dict(person=self.person,floor=self.floor)),
                                  backend=NS(infer=lambda image:self.depth),clock=lambda:self.clock[0])
        return service

    def test_no_manual_file_estimates_tilt_height_and_person_range_without_validating_itself(self):
        service=self.scene()
        result=service.estimate(self.request)
        value=result['range']
        self.assertEqual(self.request['binding'],result['binding'])
        self.assertAlmostEqual(.6,value['distance_m'],places=5)
        self.assertFalse(value['validated'])
        self.assertFalse(value['front_offset_measured'])
        self.assertEqual('camera_ground_projection',value['distance_reference'])
        geometry=value['geometry']
        self.assertEqual('boot',geometry['head_reference_id'])
        self.assertEqual('nominal_prior_not_current_measurement',geometry['height_source'])
        self.assertAlmostEqual(value['distance_m'],
                               geometry['person_forward_model_units']*geometry['depth_scale'])
        self.assertTrue(value['feet_checked'])
        self.assertEqual('approach',approach_decision(observation(**value))['action'])

    def test_floor_reference_corrects_large_network_scale_bias(self):
        service=self.scene();self.depth*=4
        value=service.estimate(self.request)['range']
        self.assertAlmostEqual(.6,value['distance_m'],places=5)
        self.assertEqual('nominal_lower_height_reference',value['scale_source'])
        self.assertAlmostEqual(.25,value['geometry']['depth_scale'],places=5)
        self.assertAlmostEqual(2.4,value['geometry']['person_forward_model_units'],places=5)
        self.assertFalse(value['validated'])

    def test_existing_observer_requests_use_nominal_lower_height_default(self):
        service=self.scene();self.request['camera'].pop('nominal_lower_height_m')
        value=service.estimate(self.request)['range']
        self.assertEqual(.15,value['nominal_lower_height_m'])
        self.assertAlmostEqual(.6*.15/.18,value['distance_m'],places=5)
        self.assertFalse(value['validated'])

    def test_first_upper_view_requests_lower_initialization_then_other_views_use_saved_scale(self):
        service=self.scene();self.request['head']['position']=30
        self.assertFalse(service.estimate(self.request)['range']['available'])
        self.request['head']['position']=52;service.estimate(self.request)
        self.request['head']['position']=30;self.request['binding']['epoch']=2
        value=service.estimate(self.request)['range']
        self.assertTrue(value['available']);self.assertFalse(value['feet_checked'])
        self.request['head']['reference_id']='newboot'
        self.assertFalse(service.estimate(self.request)['range']['available'])

    def test_near_furniture_is_excluded_from_person_surface(self):
        service=self.scene()
        self.person[80:160,125:155]=False
        self.depth[80:160,125:155]=.25
        self.assertAlmostEqual(.6,service.estimate(self.request)['range']['distance_m'],places=5)

    def test_pose_cache_survives_brief_floor_occlusion_only_at_same_reference_and_angle(self):
        for fault in (None,'time','head','reference','intrinsics','motion'):
            service=self.scene();service.estimate(self.request)
            self.floor[:]=False
            if fault=='time':self.clock[0]=41.
            if fault=='head':self.request['head']['position']=30
            if fault=='reference':self.request['head']['reference_id']='newboot'
            if fault=='intrinsics':self.request['camera']['intrinsics']['fy']=250
            if fault=='motion':self.request['binding']['epoch']=2
            value=service.estimate(self.request)['range']
            with self.subTest(fault=fault):
                if fault:
                    self.assertFalse(value['available']);self.assertEqual('lower',value['next_view'])
                else:
                    self.assertEqual('recent_same_view_floor_estimate',value['pose_source'])
                    self.assertFalse(value['feet_checked'])

    def test_measured_calibration_takes_priority_and_remains_unchanged(self):
        service=self.scene()
        calibration=dict(self.request['camera'],schema=1,model=MODEL,validated=True,
            depth_scale=1.,near_error_m=.03,far_relative_error=.05,head_poses=[
                dict(fraction=f,height_m=.18,pitch_degrees=0.,front_offset_m=.10) for f in (0,1)])
        self.request['calibration']=copy.deepcopy(calibration)
        result=service.estimate(self.request)['range']
        self.assertTrue(result['validated']);self.assertAlmostEqual(.5,result['distance_m'])
        self.assertEqual([],service.floor_pose_cache)
        self.assertEqual(calibration,self.request['calibration'])

    def test_missing_person_mask_never_uses_furniture_or_floor_as_target(self):
        service=self.scene();self.person[:]=False
        result=service.estimate(self.request)['range']
        self.assertFalse(result['available']);self.assertNotIn('distance_m',result)

    def test_image_binding_and_resolution_are_required(self):
        for fault in ('binding','size'):
            service=self.scene()
            if fault=='binding':self.request['binding']['image_sha256']='wrong'
            else:self.request['camera']['image_size']=[640,480]
            with self.assertRaises(ValueError):service.estimate(self.request)

    def test_approximate_policy_short_steps_close_and_hidden_feet(self):
        self.assertEqual('arrived_estimate',approach_decision(observation())['action'])
        self.assertEqual('close',approach_decision(observation(.1))['action'])
        self.assertEqual(.05,approach_decision(observation(1.5))['distance_m'])
        self.assertEqual(.02,approach_decision(observation(.22))['distance_m'])
        self.assertEqual('look_lower',approach_decision(observation(.8,feet_checked=False))['action'])
        self.assertEqual('inspect',approach_decision(observation(1.5,consistent_samples=1))['action'])

    def test_previously_premature_stop_estimates_now_allow_short_approach(self):
        for value in (.244435,.321137,.342118,.5,.7):
            decision=approach_decision(observation(value))
            with self.subTest(estimate=value):
                self.assertEqual('approach',decision['action'])
                self.assertEqual(.02 if value<=.3 else .05,decision['distance_m'])
        self.assertEqual('arrived_estimate',approach_decision(observation(.2))['action'])
        self.assertEqual('arrived_estimate',approach_decision(observation(.12))['action'])
        self.assertEqual('look_lower',approach_decision(observation(.24,feet_checked=False))['action'])

    def test_bad_or_disabled_estimates_do_not_become_arrival_or_movement(self):
        for field,value in (('age_seconds',4),('track_id','other'),('profile_id','other'),
                            ('distance_m',float('nan')),('source','gemini_guess'),
                            ('distance_reference','unknown'),('upper_m',.1),('consistent_samples','two')):
            self.assertEqual('inspect',approach_decision(observation(**{field:value}))['action'])
        value=observation();value['approximate_approach_enabled']=False
        self.assertEqual('inspect',approach_decision(value)['action'])

    def test_current_lower_view_allows_short_step_without_claiming_checked_feet(self):
        value=observation(.347601,feet_checked=False,pose_source='current_floor_estimate',
                          geometry=dict(head_fraction=0.,head_position=8))
        self.assertEqual(dict(action='approach',distance_m=.02,estimated=True),approach_decision(value))
        self.assertFalse(value['range']['feet_checked'])
        self.assertFalse(value['range']['validated'])
        for fraction in (.5,-.1,True,float('nan'),None):
            value['range']['geometry']['head_fraction']=fraction
            self.assertEqual('look_lower',approach_decision(value)['action'])
        value['range']['geometry']['head_fraction']=0.
        value['range']['pose_source']='recent_floor_estimate'
        self.assertEqual('look_lower',approach_decision(value)['action'])
        value['range']['pose_source']='current_floor_estimate'
        value['range']['age_seconds']=4.
        self.assertEqual('inspect',approach_decision(value)['action'])

    def test_lower_view_uncertain_feet_mission_steps_then_reports_estimated_arrival(self):
        from robot.jetson.mission.autonomous_find import parse_args,run
        def lower_target(distance):
            return dict(outcome='target_found',target_observation=observation(distance,
                feet_checked=False,pose_source='current_floor_estimate',geometry=dict(head_fraction=0.)))
        with tempfile.TemporaryDirectory() as directory:
            args=parse_args(['--execute','--skip-camera-calibration','--cable-zero-confirmed','--report',directory+'/mission.json'])
            replies=[lower_target(.347601),dict(outcome='success',drive_started=True,travelled_m=.02),lower_target(.18)]
            with patch('robot.jetson.mission.autonomous_find.run_child',side_effect=replies) as child:
                report=run(args)
            self.assertEqual(3,child.call_count)
            command=child.call_args_list[1].args[0]
            self.assertEqual('0.02',command[command.index('--maximum-step-distance')+1])
            self.assertEqual('target_found_at_estimated_standoff',report['outcome'])
            self.assertFalse(report['target_observation']['range']['feet_checked'])
            self.assertFalse(report['target_observation']['range']['validated'])

    def test_mission_and_ui_report_estimated_arrival_without_commanding_a_step(self):
        from robot.jetson.mission.autonomous_find import parse_args,run
        from robot.jetson.perception.mission_control import mission_result
        with tempfile.TemporaryDirectory() as directory:
            args=parse_args(['--execute','--skip-camera-calibration','--cable-zero-confirmed','--report',directory+'/mission.json'])
            with patch('robot.jetson.mission.autonomous_find.run_child',return_value=dict(
                    outcome='target_found',target_observation=observation())) as child:
                report=run(args)
            self.assertEqual(1,child.call_count)
            self.assertEqual('target_found_at_estimated_standoff',report['outcome'])
            self.assertNotIn('safe_standoff_confirmed',report['events'])
            self.assertIn('approximate',mission_result(report)['message'])

    def test_combined_mission_moves_on_clothing_estimate_and_stops_at_estimated_arrival(self):
        from robot.jetson.mission.autonomous_find import parse_args,run
        with tempfile.TemporaryDirectory() as directory:
            args=parse_args(['--execute','--skip-camera-calibration','--cable-zero-confirmed','--report',directory+'/mission.json'])
            replies=[dict(outcome='target_found',target_observation=observation(.8)),
                     dict(outcome='success',drive_started=True,travelled_m=.05),
                     dict(outcome='target_found',target_observation=observation(.18))]
            with patch('robot.jetson.mission.autonomous_find.run_child',side_effect=replies) as child:
                report=run(args)
            self.assertEqual('target_found_at_estimated_standoff',report['outcome'])
            command=child.call_args_list[1].args[0]
            self.assertEqual('0.05',command[command.index('--maximum-step-distance')+1])
            self.assertIn('target_reacquired',report['events'])
            self.assertFalse(report['target_observation']['range']['validated'])

    def test_missing_range_requests_lower_view_keeps_identity_and_never_claims_arrival(self):
        from robot.jetson.mission.autonomous_find import parse_args,run
        target=observation(available=False,next_view='lower')
        with tempfile.TemporaryDirectory() as directory:
            args=parse_args(['--execute','--skip-camera-calibration','--cable-zero-confirmed','--report',directory+'/mission.json'])
            with patch('robot.jetson.mission.autonomous_find.run_child',return_value=dict(
                    outcome='target_found',target_observation=target)) as child:
                report=run(args)
            self.assertEqual('target_found_not_at_standoff',report['outcome'])
            self.assertTrue(report['target_observation']['identity_confirmed'])
            self.assertTrue(all('--range-lower-view' in call.args[0] for call in child.call_args_list[1:]))

    def test_observer_requests_approximate_range_without_file_but_never_mixes_image_and_box_frames(self):
        from robot.jetson.perception.family_observer import FamilyObserver
        with tempfile.TemporaryDirectory() as directory:
            sent=[];observer=FamilyObserver.__new__(FamilyObserver)
            params=dict(range_calibration_file=directory+'/missing.json',approximate_approach_enabled=True,person_range_url='unused')
            observer.node=NS(get_parameter=lambda name:NS(value=params[name]))
            observer.motion=False;observer.image=np.zeros((20,20,3),np.uint8)
            observer.image_frame_key=['cam',10,0];observer.range_future=None;observer.last_range_at=-100
            observer.calibration={};observer.calibration_mtime=None;observer.range_camera=dict(image_size=[20,20])
            observer.head=dict(reference_id='boot')
            observer.range_executor=NS(submit=lambda fn,url,request:sent.append(request) or NS())
            track=dict(id='t',epoch=1,profile_id='mom',revision='v',frame_key=['cam',9,0],box=[1,1,19,19],seen_at=9.)
            observer.range_poll(track,10.);self.assertFalse(sent)
            track.update(frame_key=['cam',10,0],seen_at=10.)
            observer.range_poll(track,10.);self.assertEqual('approximate',sent[0]['mode'])
            self.assertEqual({},sent[0]['calibration'])


if __name__=='__main__':unittest.main()
