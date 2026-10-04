import base64
from concurrent.futures import Future
import json
import math
from pathlib import Path
import tempfile
import unittest

from robot.jetson.perception.family_store import FamilyStore, FamilyTargetStore
from robot.jetson.perception.recognition_core import TargetStore
from robot.jetson.perception.family_faces import FamilyFaceMatcher
from robot.jetson.perception.wardrobe_tracking import WardrobeTracker, similarity
from robot.mac.wardrobe_advisor import validate
from robot.jetson.mission.person_approach import approach_decision
from robot.jetson.navigation.ground_plane import expected_floor_z_m


class FamilyMemoryTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.path=Path(self.tmp.name)/'people.sqlite3';self.store=FamilyStore(self.path)
        self.payload=dict(label='Me',samples=[dict(embedding=[1.,0.])],model=dict(id='test',sha256='a'*64))
        self.me=self.store.save_profile(self.payload)
        self.mom=self.store.save_profile(dict(self.payload,label='Mom'))
        self.anchor=dict(source='face',confirmed=True,frame_key=['cam',10,0])

    def remember(self,p=None,jpeg=None):
        p=p or self.me
        return self.store.remember(p['profile_id'],p['revision'],[],{},jpeg or b'x'*120,self.anchor)

    def test_profiles_survive_restart_and_selection(self):
        self.store.select(self.me['profile_id'])
        reopened=FamilyStore(self.path)
        self.assertEqual('Me',reopened.load()['label']);self.assertEqual(2,len(reopened.profiles()))

    def test_rename_preserves_identity_and_wardrobe(self):
        oid=self.remember();self.store.rename(self.me['profile_id'],'Animesh')
        p=self.store.load(self.me['profile_id'])
        self.assertEqual(self.me['revision'],p['revision']);self.assertEqual(oid,self.store.wardrobe(p['profile_id'])[0]['id'])

    def test_clothing_only_never_teaches_ownership(self):
        self.anchor['source']='clothing'
        with self.assertRaises(ValueError):self.remember()

    def test_stale_profile_cannot_receive_delayed_learning(self):
        self.store.save_profile(self.payload,self.me['profile_id'])
        with self.assertRaises(ValueError):self.remember()

    def test_multiple_outfits_and_cascading_delete(self):
        self.remember();self.remember(jpeg=b'y'*120);self.remember(self.mom)
        self.store.delete(self.me['profile_id'])
        self.assertEqual(1,len(self.store.wardrobe()))
        self.assertEqual(self.mom['profile_id'],self.store.wardrobe()[0]['profile_id'])

    def test_reference_limit_does_not_remove_outfits(self):
        oid=self.remember()
        for i in range(10):
            self.store.remember(self.me['profile_id'],self.me['revision'],[],{},bytes([i])*120,self.anchor,oid)
        self.assertEqual(6,self.store.wardrobe(self.me['profile_id'],images=True)[0]['view_count'])
        with self.assertRaises(ValueError):
            self.store.remember(self.mom['profile_id'],self.mom['revision'],[],{},b'z'*120,self.anchor,oid)

    def test_legacy_migration_exactly_once_preserves_file(self):
        path=Path(self.tmp.name)/'legacy'/'target.json';legacy=TargetStore(path,'test','a'*64)
        legacy.save('Old',[dict(embedding=[1.,0.]) for _ in range(18)],'embeddings');old=path.read_bytes()
        self.assertEqual(18,len(FamilyTargetStore(legacy).load()['samples']))
        adapter=FamilyTargetStore(legacy);adapter.save('Second',[dict(embedding=[0.,1.])],'embeddings')
        self.assertEqual(2,len(FamilyTargetStore(legacy).family.profiles()));self.assertEqual(old,path.read_bytes())
        adapter.family.delete(adapter.family.selected_id())
        self.assertEqual(1,len(FamilyTargetStore(legacy).family.profiles()))

    def test_face_margin_and_track_histories(self):
        tracker=FamilyFaceMatcher()
        def face(box,a,b):return dict(box=box,scores=[dict(profile_id='a',revision='1',score=a),dict(profile_id='b',revision='1',score=b)])
        left,right=[0,0,10,10],[40,0,50,10]
        self.assertFalse(tracker.observe([face(left,.6,.1)],1)[0]['confirmed'])
        self.assertFalse(tracker.observe([face(right,.6,.1)],1.1)[0]['confirmed'])
        self.assertTrue(tracker.observe([face(left,.6,.1)],1.2)[0]['confirmed'])
        self.assertFalse(tracker.observe([face(left,.6,.59)],1.3)[0]['confirmed'])

    def test_movement_preserves_reference_not_position(self):
        tracker=WardrobeTracker(self.store,'unused');self.addCleanup(tracker.close)
        tracker.tracks['t']=dict(profile_id=self.me['profile_id'],reference_descriptor={'saved':1},range={},position_valid=True)
        tracker.invalidate_position();t=tracker.tracks['t']
        self.assertEqual({'saved':1},t['reference_descriptor']);self.assertFalse(t['position_valid']);self.assertIsNone(t['range'])

    def test_delayed_cloud_cannot_attach_after_camera_move(self):
        tracker=WardrobeTracker(self.store,'unused');self.addCleanup(tracker.close)
        binding=dict(track_id='t',epoch=0)
        tracker.tracks['t']=dict(epoch=1,position_valid=True,seen_at=10,profile_id=None)
        future=Future();future.set_result(dict(ok=True,binding=binding,interpretation={'decision':'match'}))
        tracker.future=future;tracker.pending=dict(binding=binding)
        tracker.poll(10);self.assertIsNone(tracker.tracks['t']['profile_id'])


class WardrobeResponseTests(unittest.TestCase):
    def answer(self):
        return dict(regions=[dict(region=r,visible=True,colour='blue',pattern='striped',detail='navy trousers') for r in ('upper','lower','footwear')],
                    decision='match',reference_id='one',supporting_regions=['lower'],conflicting_regions=[],next_view='hold',evidence='Same stripe pattern')

    def test_paraphrase_is_explanation_not_identifier(self):
        a=self.answer();self.assertEqual('one',validate(a,['one'],'compare')['reference_id'])
        a['regions'][1]['detail']='dark blue pants';self.assertEqual('one',validate(a,['one'],'compare')['reference_id'])

    def test_rejects_unknown_ids_conflicts_and_commands(self):
        for change in (dict(reference_id='invented'),dict(conflicting_regions=['lower']),dict(next_view='drive'),dict(extra='command')):
            with self.assertRaises(ValueError):validate(dict(self.answer(),**change),['one'],'compare')
        with self.assertRaises(ValueError):validate(self.answer(),['one'],'describe')

    def test_identical_references_cannot_become_unique_by_model_choice(self):
        import hashlib
        from types import SimpleNamespace
        from robot.mac.wardrobe_advisor import WardrobeAdvisor
        advisor=WardrobeAdvisor(SimpleNamespace(interpret_structured=lambda *a,**kw:dict(interpretation=self.answer())))
        image=base64.b64encode(b'example pixels').decode()
        binding=dict(request_id='r',frame_key=['cam',1,0],track_id='t',revision='v',image_sha256=hashlib.sha256(b'example pixels').hexdigest())
        r=advisor.interpret(dict(operation='compare',binding=binding,image=image,
            references=[dict(id=i,image=image) for i in ('one','two')]))
        self.assertEqual('uncertain',r['interpretation']['decision'])
        self.assertEqual('',r['interpretation']['reference_id'])

    def test_structured_view_advice_respects_both_encoder_directions(self):
        from robot.jetson.mission.person_approach import wardrobe_view_target
        for lower,upper in [(30,-30),(-30,30)]:
            head=dict(position=0,down_position=lower,up_position=upper)
            target=wardrobe_view_target(dict(age_seconds=.1,next_view='raise'),head)
            self.assertGreater(target*upper,0)
            self.assertIsNone(wardrobe_view_target(dict(age_seconds=2,next_view='raise'),head))


class FamilyRangePolicyTests(unittest.TestCase):
    def observation(self):
        return dict(identity_confirmed=True,profile_id='me',track_id='t',identity_source='clothing',age_seconds=.1,
                    clothing_approach_enabled=True,range=dict(distance_m=.6,lower_m=.55,upper_m=.65,
                    validated=True,feet_checked=True,consistent_samples=2))

    def test_arrives_without_face(self):
        self.assertEqual('arrived',approach_decision(self.observation())['action'])

    def test_unvalidated_missing_hidden_feet_and_stale_ranges(self):
        o=self.observation();o['range']['validated']=False;self.assertEqual('inspect',approach_decision(o)['action'])
        o=self.observation();o['range']['feet_checked']=False;self.assertEqual('look_lower',approach_decision(o)['action'])
        o=self.observation();o['range']=None;self.assertEqual('inspect',approach_decision(o)['action'])
        o=self.observation();o['age_seconds']=10;self.assertEqual('reacquire',approach_decision(o)['action'])
        o=self.observation();o['clothing_approach_enabled']=False;self.assertEqual('reacquire',approach_decision(o)['action'])
        o=self.observation();o['range']['age_seconds']=4;self.assertEqual('inspect',approach_decision(o)['action'])
        o=self.observation();o['range']['track_id']='crossing_person';self.assertEqual('inspect',approach_decision(o)['action'])

    def test_shortens_step_and_never_uses_body_size_as_metres(self):
        o=self.observation();o['range'].update(distance_m=.75,lower_m=.67,upper_m=.83)
        self.assertAlmostEqual(.07,approach_decision(o)['distance_m'])
        o['range'].update(distance_m=2.,lower_m=1.8,upper_m=2.2)
        self.assertEqual(.1,approach_decision(o)['distance_m'])

    def test_optical_depth_is_not_ground_forward_distance(self):
        z=expected_floor_z_m(0,400,0,30,.15)
        self.assertAlmostEqual(.30,z)
        self.assertNotAlmostEqual(.15/math.tan(math.radians(30)),z)

    def test_lower_view_request_reaches_mission_worker_before_arrival(self):
        from unittest.mock import patch
        from robot.jetson.mission.autonomous_find import parse_args,run
        with tempfile.TemporaryDirectory() as directory:
            args=parse_args(['--execute','--skip-camera-calibration','--cable-zero-confirmed','--report',directory+'/report.json'])
            first=self.observation();first['range']['feet_checked']=False
            results=[dict(outcome='target_found',target_observation=first),
                     dict(outcome='target_found',target_observation=self.observation())]
            with patch('robot.jetson.mission.autonomous_find.run_child',side_effect=results) as child:
                report=run(args)
            self.assertEqual('target_found_at_standoff',report['outcome'])
            self.assertIn('--range-lower-view',child.call_args_list[1].args[0])
            self.assertEqual(2,child.call_count,'Only view checks, no approach when already in the arrival band')


class NumericalFamilyTests(unittest.TestCase):
    def setUp(self):
        try:
            import numpy as np
            import cv2
        except ImportError:self.skipTest('OpenCV/NumPy unavailable')
        self.np=np

    def test_range_excludes_background_and_checks_ground_contact(self):
        from robot.mac.person_range import estimate_range
        np=self.np;depth=np.full((100,100),8.);person=np.zeros((100,100),bool)
        person[20:80,40:60]=True;depth[person]=.75
        floor=np.zeros_like(person);floor[80:,:]=True
        calibration=dict(intrinsics=dict(fy=100.,cy=50.),depth_scale=1.,near_error_m=.05,far_relative_error=.1,validated=True)
        pose=dict(pitch_degrees=0.,height_m=.15,front_offset_m=.15,fraction=0.)
        r=estimate_range(depth,person,floor,[35,15,65,80],calibration,pose)
        self.assertAlmostEqual(.6,r['distance_m']);self.assertTrue(r['feet_checked'])
        r=estimate_range(depth,person,floor,[35,15,65,80],calibration,dict(pose,fraction=.8))
        self.assertFalse(r['feet_checked'])
        r=estimate_range(depth,person,floor,[35,15,65,80],calibration,dict(pose,height_m=.6))
        self.assertFalse(r['feet_checked'],'Floor below a torso is not visible feet')
        r=estimate_range(depth,person,floor,[35,15,65,100],calibration,pose)
        self.assertTrue(r['feet_checked'],'The complete person mask matters, not a loose box reaching the image edge')
        with self.assertRaises(ValueError):estimate_range(depth,person&False,floor,[35,15,65,80],calibration,pose)

    def test_real_frames_preserve_track_after_head_move_without_relearning(self):
        from robot.jetson.perception.wardrobe_tracking import descriptor
        np=self.np
        with tempfile.TemporaryDirectory() as directory:
            store=FamilyStore(Path(directory)/'people.sqlite3')
            me=store.save_profile(dict(label='Me',samples=[dict(embedding=[1.,0.])]))
            tracker=WardrobeTracker(store,'unused');self.addCleanup(tracker.close)
            tracker.retry_at=1000 # isolate frame association from the cloud
            image=np.full((160,160,3),(200,80,30),np.uint8)
            boxes=[[20,10,140,155]]
            faces=[dict(box=[50,20,90,60],confirmed=True,profile_id=me['profile_id'],revision=me['revision'])]
            tracker.observe(image,boxes,faces,['cam',1,0],1)
            first=tracker.selected(me['profile_id'],1)
            self.assertIsNotNone(first)
            tracker.invalidate_position()
            self.assertIsNone(tracker.selected(me['profile_id'],1))
            tracker.observe(image,boxes,[],['cam',2,0],2)
            again=tracker.selected(me['profile_id'],2)
            self.assertEqual(first['id'],again['id']);self.assertEqual('tracking',again['identity_source'])
            self.assertEqual([],store.wardrobe())

    def test_measured_acceptance_never_reuses_training_frames(self):
        from scripts.phase6.family_memory import calibrate
        document=dict(intrinsics=dict(fy=400,cy=240),image_size=[640,480],head_poses=[
            dict(fraction=f,height_m=.15,pitch_degrees=0,front_offset_m=.1) for f in (0,1)],samples=[])
        for i,distance in enumerate([.6,1.,2.,.5,.7,1.,1.5,2.,3.]):
            document['samples'].append(dict(frame_sha256=str(i),posture='seated' if i%2 else 'standing',
                measured_m=distance,estimated_m=distance,head_fraction=(i%2)*.3,split='train' if i<3 else 'validation'))
        self.assertTrue(calibrate(document)['validated'])
        document['samples'][-1]['frame_sha256']='0'
        with self.assertRaises(ValueError):calibrate(document)

    def test_detector_gaps_reacquire_same_position_without_track_explosion(self):
        np=self.np
        with tempfile.TemporaryDirectory() as directory:
            store=FamilyStore(Path(directory)/'people.sqlite3')
            tracker=WardrobeTracker(store,'unused');self.addCleanup(tracker.close)
            tracker.retry_at=float('inf')
            image=np.full((200,200,3),(200,80,30),np.uint8)
            left,right=[0,0,80,180],[120,0,200,180]
            tracker.observe(image,[left,right],[],['cam',1,0],1.)
            original=next(t['id'] for t in tracker.tracks.values() if t['box']==left)
            for index in range(8):
                now=1.1+index*.1
                tracker.observe(image,[],[],['cam',2,index*2],now)
                self.assertFalse(any(t['position_valid'] for t in tracker.tracks.values()))
                tracker.observe(image,[left],[],['cam',2,index*2+1],now+.05)
                current=[t['id'] for t in tracker.tracks.values() if t['position_valid']]
                self.assertEqual([original],current)
            self.assertEqual(2,len(tracker.tracks))
            # Camera movement invalidates this spatial tie-breaker. With two
            # identical appearances, an old screen position cannot pick a person.
            tracker.invalidate_position()
            tracker.observe(image,[left],[],['cam',3,0],2.1)
            current=next(t['id'] for t in tracker.tracks.values() if t['position_valid'])
            self.assertNotEqual(original,current)

    def test_completed_clothing_answer_waits_for_brief_body_gap_but_not_head_move(self):
        with tempfile.TemporaryDirectory() as directory:
            store=FamilyStore(Path(directory)/'people.sqlite3')
            tracker=WardrobeTracker(store,'unused');self.addCleanup(tracker.close)
            binding=dict(track_id='t',epoch=0,profile_id=None,revision='unidentified')
            tracker.tracks['t']=dict(id='t',epoch=0,profile_id=None,revision=None,
                position_valid=False,seen_at=9.8)
            future=Future();future.set_result(dict(ok=True,binding=binding,interpretation=dict(
                decision='uncertain',reference_id='',regions=[],next_view='hold')))
            tracker.future=future;tracker.pending=dict(binding=binding,face=None,references=[],signature='s')
            tracker.last_cloud_at=9.
            tracker.poll(10.)
            self.assertIs(future,tracker.future)
            tracker.tracks['t'].update(position_valid=True,seen_at=10.1)
            tracker.poll(10.1)
            self.assertIsNone(tracker.future)
            self.assertEqual('hold',tracker.tracks['t']['next_view'])
            for moved,now in [(True,10.2),(False,20.)]:
                tracker.epoch=0
                tracker.tracks['t'].update(position_valid=False,epoch=0)
                tracker.future=future;tracker.pending=dict(binding=binding)
                if moved:tracker.invalidate_position()
                tracker.poll(now)
                self.assertIsNone(tracker.future)

    def test_body_updates_continue_when_face_worker_has_no_reply(self):
        from collections import OrderedDict
        from types import SimpleNamespace as NS
        from robot.jetson.perception.family_observer import FamilyObserver
        observer=FamilyObserver.__new__(FamilyObserver)
        observer.cache={k:OrderedDict() for k in ('image','people','faces')}
        observer.now=lambda:1.4;observer.motion=False;observer.not_before=0.;observer.last_frame_time=0.
        received=[];observer.tracker=NS(observe=lambda *args:received.append(args))
        header=NS(frame_id='cam',stamp=NS(sec=1,nanosec=0))
        frame=NS(header=header,encoding='bgr8',width=24,height=24,step=72,data=bytes([80])*24*24*3)
        person=NS(bbox=NS(center=NS(position=NS(x=12,y=12)),size_x=20,size_y=20))
        observer.offer('image',frame);observer.offer('people',NS(header=header,detections=[person]))
        self.assertEqual(1,len(received));self.assertEqual([],received[0][2]);self.assertEqual(1.,received[0][-1])
        observer.offer('faces',NS(data=json.dumps(dict(frame_key=['cam',1,0],faces=[]))))
        self.assertEqual(1,len(received),'A late face batch must not roll the body track backward')

    def test_observation_only_does_not_change_face_steering_and_active_mode_keeps_face_truthful(self):
        from types import SimpleNamespace as NS
        from unittest.mock import patch
        from robot.jetson.perception.family_observer import FamilyObserver
        observer=FamilyObserver.__new__(FamilyObserver);observer.now=lambda:10.
        observer.image=self.np.zeros((100,100,3),self.np.uint8);observer.intrinsics=None;observer.motion=False
        observer.diagnostic=None;observer.range_poll=lambda *args:None
        track=dict(id='t',identity_source='tracking',box=[0,0,100,100],revision='v',frame_key=['cam',10,0],seen_at=10.)
        observer.tracker=NS(poll=lambda *a:None,selected=lambda *a:track,guidance=lambda *a:None,last_error=None)
        observer.store=NS(selected_id=lambda:'me',load=lambda *a:dict(label='Me',revision='v'))
        observer.publisher=NS(publish=lambda *a:None)
        with patch.dict('sys.modules',{'std_msgs.msg':NS(String=NS)}):
            observer.node=NS(get_parameter=lambda *a:NS(value=False))
            result=observer.enrich(dict(confirmed=True,center_x_fraction=.2,center_y_fraction=.3,age_seconds=.2))
            self.assertEqual(.2,result['center_x_fraction']);self.assertTrue(result['confirmed'])
            observer.node=NS(get_parameter=lambda *a:NS(value=True))
            result=observer.enrich(dict(confirmed=True,center_x_fraction=.2))
            self.assertFalse(result['confirmed']);self.assertEqual(.5,result['center_x_fraction'])
            track.update(identity_source='face',face_box=[10,10,30,30])
            result=observer.enrich({})
            self.assertTrue(result['confirmed']);self.assertEqual(.2,result['center_x_fraction'])

    def test_restart_matches_saved_clothes_without_face_or_learning_new_ownership(self):
        import cv2
        from robot.jetson.perception.wardrobe_tracking import descriptor
        np=self.np
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'people.sqlite3';store=FamilyStore(path)
            me=store.save_profile(dict(label='Me',samples=[dict(embedding=[1.,0.])]))
            image=np.full((160,160,3),(200,80,30),np.uint8);box=[20,10,140,155]
            desc=descriptor(image[10:155,20:140]);ok,jpeg=cv2.imencode('.jpg',image[50:155,20:140])
            oid=store.remember(me['profile_id'],me['revision'],[],desc,jpeg.tobytes(),dict(source='face',confirmed=True,frame_key=['cam',1,0]))
            requests=[]
            def cloud(url,request):
                requests.append(request)
                return dict(ok=True,binding=request['binding'],interpretation=dict(decision='match',reference_id=oid,
                    supporting_regions=['lower'],conflicting_regions=[],regions=[dict(region='lower',visible=True)],next_view='hold'))
            tracker=WardrobeTracker(FamilyStore(path),'unused',transport=cloud);self.addCleanup(tracker.close)
            tracker.observe(image,[box],[],['cam',2,0],2)
            tracker.observe(image,[box],[],['cam',3,0],3)
            tracker.future.result(timeout=1);tracker.poll(3)
            target=tracker.selected(me['profile_id'],3)
            self.assertEqual('clothing',target['identity_source']);self.assertEqual(oid,target['outfit_id'])
            tracker.observe(image,[box],[],['cam',4,0],4)
            self.assertEqual(1,len(requests));self.assertEqual(1,len(store.wardrobe()))
            self.assertNotIn('confirmed',tracker.selected(me['profile_id'],4))

    def test_recorded_evaluation_cannot_pass_with_self_images(self):
        from scripts.phase6.family_memory import evaluate_wardrobe
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);(root/'same.jpg').write_bytes(b'x'*150)
            doc=dict(references=[dict(id='one',profile_id='me',image='same.jpg')],
                     cases=[dict(id='case',image='same.jpg',references=['one'],expected_profile='me',scenario='same_outfit')])
            with self.assertRaisesRegex(ValueError,'differ'):
                evaluate_wardrobe(doc,root,None)

    def test_crossing_people_stay_ambiguous_without_face_and_low_light_is_unknown(self):
        from robot.jetson.perception.wardrobe_tracking import descriptor
        np=self.np
        with tempfile.TemporaryDirectory() as directory:
            store=FamilyStore(Path(directory)/'people.sqlite3')
            me=store.save_profile(dict(label='Me',samples=[dict(embedding=[1.,0.])]))
            mom=store.save_profile(dict(label='Mom',samples=[dict(embedding=[0.,1.])]))
            tracker=WardrobeTracker(store,'unused');self.addCleanup(tracker.close);tracker.retry_at=1000
            image=np.full((200,200,3),(200,80,30),np.uint8)
            boxes=[[5,0,95,190],[105,0,195,190]]
            faces=[dict(box=b,confirmed=True,profile_id=p['profile_id'],revision=p['revision'])
                   for b,p in [([25,10,65,40],me),([125,10,165,40],mom)]]
            tracker.observe(image,boxes,faces,['cam',1,0],1)
            self.assertIsNotNone(tracker.selected(me['profile_id'],1))
            tracker.observe(image,[[40,0,160,190]],[],['cam',2,0],2)
            self.assertIsNone(tracker.selected(me['profile_id'],2))
            self.assertIsNone(tracker.selected(mom['profile_id'],2))
            self.assertIsNone(descriptor(np.zeros_like(image)))

    def test_newer_face_overrides_delayed_clothing_and_timeout_keeps_local_track(self):
        with tempfile.TemporaryDirectory() as directory:
            store=FamilyStore(Path(directory)/'people.sqlite3')
            me=store.save_profile(dict(label='Me',samples=[dict(embedding=[1.,0.])]))
            tracker=WardrobeTracker(store,'unused');self.addCleanup(tracker.close)
            binding=dict(track_id='t',epoch=0,profile_id=None,revision='unidentified')
            t=dict(id='t',epoch=0,position_valid=True,seen_at=10,profile_id=me['profile_id'],revision=me['revision'],identity_source='face')
            tracker.tracks['t']=t;tracker.last_cloud_at=9.
            future=Future();future.set_result(dict(ok=True,binding=binding,interpretation={'decision':'match'}))
            tracker.future=future;tracker.pending=dict(binding=binding)
            tracker.poll(10)
            self.assertEqual(me['profile_id'],t['profile_id']);self.assertIsNone(tracker.last_error)
            future=Future();future.set_exception(TimeoutError('Timed out'))
            tracker.future=future;tracker.pending=dict(binding=binding)
            tracker.poll(10)
            self.assertEqual(me['profile_id'],t['profile_id']);self.assertGreater(tracker.retry_at,10)


if __name__=='__main__':unittest.main()
