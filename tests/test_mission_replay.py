import copy
import json
import tempfile
import unittest
from pathlib import Path
from scripts.diagnostics.replay_mission import load_recording,rebase_times,replay_tracker,summary,COMMAND_TOPICS


class ReplayTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.root=Path(self.tmp.name)
        self.manifest=dict(schema='echora.mission_recording',version=1,started_at_unix=1000,status='complete')
        (self.root/'manifest.json').write_text(json.dumps(self.manifest))

    def save(self,events):
        (self.root/'events.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in events))

    def test_format_order_and_path_checks(self):
        self.save([]);self.assertEqual([],load_recording(self.root)[2])
        events=[dict(kind='frame',elapsed_ns=0,data=dict(path='../outside.jpg'))]
        self.save(events)
        with self.assertRaisesRegex(ValueError,'frame path'):load_recording(self.root)
        self.save([dict(kind='message',elapsed_ns=n) for n in (2,1)])
        with self.assertRaisesRegex(ValueError,'ordered'):load_recording(self.root)
        (self.root/'events.jsonl').write_text('{')
        with self.assertRaisesRegex(ValueError,'Incomplete'):load_recording(self.root)

    def test_rebase_keeps_frame_bindings_and_ages(self):
        d=dict(header=dict(stamp=dict(sec=1000,nanosec=900000000),frame_id='cam'),
               frame_key=['cam',1000,900000000],frame_time=1000.9,age_seconds=.2,
               nested=dict(observed_at=1000.9,reference_id='boot',distance_m=.3))
        original=copy.deepcopy(d);r=rebase_times(d,100.25)
        self.assertEqual(['cam',1101,150000000],r['frame_key'])
        self.assertEqual(dict(sec=1101,nanosec=150000000),r['header']['stamp'])
        self.assertAlmostEqual(1101.15,r['frame_time']);self.assertEqual(.2,r['age_seconds'])
        self.assertEqual(original,d);self.assertEqual('boot',r['nested']['reference_id'])

    def test_generated_crop_recording_replays_without_cloud_or_permanent_enrollment(self):
        import cv2
        import numpy as np
        events=[]
        def add(t,topic,data,kind='message',typ='std_msgs/msg/String'):
            events.append(dict(sequence=len(events),elapsed_ns=int(t*1e9),received_at_unix=1000+t,
                               topic=topic,type=typ,kind=kind,data=data))
        def frame(t,image):
            sec=int(1000+t);ns=round((1000+t-sec)*1e9)
            header=dict(frame_id='cam',stamp=dict(sec=sec,nanosec=ns));h,w=image.shape[:2]
            path=f'{len(events)}.jpg';cv2.imwrite(str(self.root/path),image)
            add(t,'/perception/person_detections',dict(header=header,detections=[dict(bbox=dict(
                center=dict(position=dict(x=w/2,y=h/2)),size_x=w,size_y=h))]),typ='vision_msgs/msg/Detection2DArray')
            add(t,'/camera/image_raw',dict(path=path,header=header,width=w,height=h),kind='frame',typ='sensor_msgs/msg/Image')
            return ['cam',sec,ns]
        full=np.zeros((240,100,3),np.uint8);full[:120]=(150,50,20);full[120:210]=210;full[210:]=(90,130,190)
        add(0,'/camera_head/status',dict(position=-2,reference_id='boot',moving=False))
        key=frame(.1,full)
        add(.1,'/perception/people_tracks',dict(identity_confirmed=True,identity_source='clothing',profile_id='mom',
            target_revision='rev',target_label='Mom',frame_key=key,body_box=[0,0,100,240]))
        add(.2,'/camera_head/status',dict(position=-2,reference_id='boot',moving=True))
        add(.8,'/camera_head/status',dict(position=9,reference_id='boot',moving=False))
        frame(1.,full[120:]);frame(1.2,full[120:]);frame(1.4,full[120:])
        self.save(events);root,manifest,loaded=load_recording(self.root)
        result=replay_tracker(root,loaded)
        self.assertEqual(['mom'],result['seeded_profiles'])
        self.assertEqual(4,result['evaluated_frames']);self.assertEqual(3,result['frames_with_retained_identity'])
        self.assertEqual(0,result['cloud_requests']);self.assertFalse(result['physical_commands_sent'])
        tracks=[r['tracks']['mom'] for r in result['observations']]
        self.assertIsNone(tracks[1]);self.assertEqual(tracks[0]['track_id'],tracks[-1]['track_id'])
        self.assertEqual([0,0,100,120],tracks[-1]['box'])
        self.assertEqual(4,summary(manifest,loaded)['frames'])
        # Source ROS time can start at zero while wall time is an epoch clock.
        shifted=copy.deepcopy(loaded)
        for e in shifted:e['data']=rebase_times(e['data'],-1000.)
        self.manifest['source_clock_unix']=0.
        (self.root/'manifest.json').write_text(json.dumps(self.manifest))
        shifted_result=replay_tracker(root,shifted)
        self.assertEqual(result['frames_with_retained_identity'],shifted_result['frames_with_retained_identity'])
        # Delayed detection cannot resurrect an image rejected by live freshness.
        late=copy.deepcopy(shifted)
        first_detection=next(e for e in late if e['topic']=='/perception/person_detections')
        first_detection['elapsed_ns']+=2_000_000_000
        self.assertEqual('no_matching_identity_anchor',replay_tracker(root,late)['status'])
        changed=copy.deepcopy(shifted)
        anchor=copy.deepcopy(next(e for e in changed if e['topic']=='/perception/people_tracks'))
        anchor['data']['target_revision']='new-revision';changed.append(anchor)
        with self.assertRaisesRegex(ValueError,'Enrollment changed'):replay_tracker(root,changed)

    def test_command_topics_are_observation_only(self):
        self.assertIn('/cmd_vel',COMMAND_TOPICS);self.assertIn('/camera_head/command',COMMAND_TOPICS)


if __name__=='__main__':unittest.main()
