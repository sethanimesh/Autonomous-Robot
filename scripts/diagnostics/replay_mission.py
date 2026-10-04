#!/usr/bin/env python3
"""Replay a recorded run offline or on an isolated ROS bus, never to an EV3.

Recorded feedback is a fixed trajectory, not a counterfactual camera/world model.
"""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
COMMAND_TOPICS={'/cmd_vel','/camera_head/command'}
DERIVED_TOPICS={'/perception/people_tracks','/mission/target_observation'}


def load_recording(directory):
    root=Path(directory).resolve()
    manifest=json.loads((root/'manifest.json').read_text())
    if manifest.get('schema')!='echora.mission_recording' or manifest.get('version')!=1:
        raise ValueError('Unsupported recording format')
    events=[];last=-1
    with (root/'events.jsonl').open() as source:
        for line_number,line in enumerate(source,1):
            try:e=json.loads(line)
            except ValueError as exc:raise ValueError(f'Incomplete event at line {line_number}') from exc
            elapsed=e.get('elapsed_ns')
            if type(elapsed) is not int or elapsed<last:raise ValueError('Recording times must be ordered')
            if e.get('kind') not in ('frame','message'):raise ValueError('Unknown event kind')
            if e['kind']=='frame':
                path=(root/e['data']['path']).resolve()
                if not path.is_relative_to(root) or not path.is_file():raise ValueError('Missing or invalid frame path')
            events.append(e);last=elapsed
    return root,manifest,events


def frame_key(data):
    header=data.get('header') or {};stamp=header.get('stamp') or {}
    if 'sec' not in stamp:return None
    return (header.get('frame_id',''),stamp['sec'],stamp.get('nanosec',0))


def source_clock(manifest):
    value=manifest.get('source_clock_unix')
    return manifest['started_at_unix'] if value is None else value


def summary(manifest,events):
    topics={};frames=[]
    for e in events:
        topics[e['topic']]=topics.get(e['topic'],0)+1
        if e['kind']=='frame':frames.append(e['elapsed_ns']/1e9)
    return dict(recording_status=manifest.get('status'),events=len(events),frames=len(frames),topics=topics,
        duration_seconds=events[-1]['elapsed_ns']/1e9 if events else 0,
        largest_frame_gap_seconds=max((b-a for a,b in zip(frames,frames[1:])),default=0),
        physical_commands_sent=False,limitation='Recorded views and feedback cannot simulate a different physical route.')


def rebase_times(value,offset):
    """Preserve source-frame associations while presenting fresh ROS/wall times."""
    if isinstance(value,list):return [rebase_times(v,offset) for v in value]
    if not isinstance(value,dict):return value
    result={k:rebase_times(v,offset) for k,v in value.items()}
    if type(value.get('sec')) is int and type(value.get('nanosec')) is int:
        ns=round((value['sec']+offset)*1e9)+value['nanosec']
        result.update(sec=ns//1_000_000_000,nanosec=ns%1_000_000_000)
    for key in ('frame_key','image_frame_key'):
        v=value.get(key)
        if isinstance(v,(list,tuple)) and len(v)==3 and type(v[1]) is int and type(v[2]) is int:
            ns=round((v[1]+offset)*1e9)+v[2];result[key]=[v[0],ns//1_000_000_000,ns%1_000_000_000]
    for key in ('frame_time','observed_at','generated_at_unix','camera_received_at_unix','captured_at_unix'):
        if type(value.get(key)) in (int,float):result[key]=value[key]+offset
    return result


class ReplayProfiles:
    """Read-only profile labels; no face templates or permanent wardrobe writes."""
    def __init__(self,profiles):self.profiles=profiles
    def load(self,pid):return self.profiles.get(pid)
    def wardrobe(self,*args,**kwargs):return []
    def remember(self,*args,**kwargs):raise RuntimeError('Replay cannot teach permanent ownership')


def boxes_from_message(data):
    result=[]
    for d in data.get('detections',[]):
        b=d['bbox'];center=b['center'];p=center.get('position',center)
        x,y=p['x'],p['y'];w,h=b['size_x'],b['size_y']
        result.append([x-w/2,y-h/2,x+w/2,y+h/2])
    return result


def replay_tracker(root,events,profile_id=None,tracker_class=None):
    """Seed each identity once from baseline evidence, then test local continuity.

    This does not evaluate face recognition or call Gemini. Baseline identity is
    an explicit replay input, never a manufactured live face observation.
    """
    import cv2
    from robot.jetson.perception.wardrobe_tracking import WardrobeTracker
    people={};people_times={};anchors={};profiles={}
    for e in events:
        d=e['data']
        if not isinstance(d,dict):continue
        if e['topic']=='/perception/person_detections':
            people[frame_key(d)]=d;people_times[frame_key(d)]=e['elapsed_ns']
        if e['topic'] in DERIVED_TOPICS and d.get('identity_confirmed') and d.get('frame_key') and d.get('body_box'):
            pid=d.get('profile_id');revision=d.get('target_revision')
            if pid and revision:
                if pid in profiles and profiles[pid]['revision']!=revision:
                    raise ValueError('Enrollment changed during recording; replay separate revision segments')
                profiles[pid]=dict(profile_id=pid,label=d.get('target_label',pid),revision=revision)
                anchors[tuple(d['frame_key'])]=d
    tracker=(tracker_class or WardrobeTracker)(ReplayProfiles(profiles),'offline-disabled')
    tracker.retry_at=float('inf')
    manifest=json.loads((root/'manifest.json').read_text())
    source_start=source_clock(manifest)
    source_delta=source_start-manifest['started_at_unix']
    timeline=[]
    for e in events:
        if e['kind']=='frame':
            e=dict(e,elapsed_ns=max(e['elapsed_ns'],people_times.get(frame_key(e['data']),e['elapsed_ns'])))
        timeline.append(e)
    timeline.sort(key=lambda e:e['elapsed_ns'])
    robot={};head={};pose=None;moving=False;seeded=set();rows=[];missing=0;not_before_source=-1.
    try:
        for e in timeline:
            d=e['data'];now=e['elapsed_ns']/1e9
            if not isinstance(d,dict):continue
            if e['topic'] in ('/robot_status','/camera_head/status'):
                if e['topic']=='/robot_status':robot=d
                else:head=d
                motors=robot.get('motors',{})
                active=bool(head.get('moving') or head.get('homing') or any(
                    abs(motors.get(k,{}).get('speed',0))>1 or abs(motors.get(k,{}).get('commanded_speed',0))>1
                    or 'running' in motors.get(k,{}).get('state',[]) for k in ('left','right')))
                current=(head.get('reference_id'),head.get('position'),motors.get('left',{}).get('generation'),
                    motors.get('right',{}).get('generation'),motors.get('left',{}).get('position'),motors.get('right',{}).get('position'))
                changed=pose is not None and (current[0]!=pose[0] or current[2:4]!=pose[2:4] or any(
                    type(current[i]) in (int,float) and type(pose[i]) in (int,float) and abs(current[i]-pose[i])>3 for i in (1,4,5)))
                if changed or moving!=active:
                    tracker.invalidate_position();not_before_source=e.get('received_at_unix',-1.)+source_delta
                if changed or pose is None:pose=current
                moving=active
            if e['kind']!='frame' or moving:continue
            key=frame_key(d)
            if key and key[1]+key[2]/1e9<=not_before_source:continue
            if key not in people:missing+=1;continue
            observed_at=(key[1]*1_000_000_000+key[2]-round(source_start*1e9))/1e9
            if not 0<=now-observed_at<=1.:continue
            image=cv2.imread(str(root/d['path']))
            if image is None:raise ValueError('Unreadable recorded image')
            tracker.observe(image,boxes_from_message(people[key]),[],list(key),observed_at)
            anchor=anchors.get(key)
            if anchor and anchor['profile_id'] not in seeded:
                from robot.jetson.perception.family_faces import iou
                matches=[t for t in tracker.tracks.values() if t['position_valid'] and iou(t['box'],anchor['body_box'])>=.8]
                if len(matches)==1:
                    t=matches[0];pid=anchor['profile_id'];seeded.add(pid)
                    t.update(profile_id=pid,revision=profiles[pid]['revision'],identity_source='replay_anchor',
                        reference_descriptor=copy.deepcopy(t['descriptor']),reference_bands=copy.deepcopy(t.get('bands')))
            selected={pid:tracker.selected(pid,now) for pid in seeded if not profile_id or pid==profile_id}
            rows.append(dict(elapsed_seconds=now,frame_key=list(key),head_position=head.get('position'),
                baseline_profile_id=anchor.get('profile_id') if anchor else None,
                tracks={pid:dict(track_id=t['id'],box=t['box'],source=t['identity_source']) if t else None for pid,t in selected.items()}))
    finally:tracker.close()
    return dict(mode='tracker',status='evaluated' if seeded else 'no_matching_identity_anchor',seeded_profiles=sorted(seeded),evaluated_frames=len(rows),
        frames_without_matching_detections=missing,frames_with_retained_identity=sum(any(r['tracks'].values()) for r in rows),
        observations=rows,cloud_requests=0,physical_commands_sent=False,
        limitation='One recorded identity anchor per profile; tests local continuity, not independent identification.')


def isolated_environment(directory):
    # Override the runtime's normal LAN transport configuration explicitly.
    xml=Path(directory)/'replay-transport.xml'
    xml.write_text('''<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
<transport_descriptors><transport_descriptor><transport_id>replay_loopback</transport_id><type>UDPv4</type>
<interfaceWhiteList><address>127.0.0.1</address></interfaceWhiteList></transport_descriptor></transport_descriptors>
<participant profile_name="replay" is_default_profile="true"><rtps><userTransports><transport_id>replay_loopback</transport_id></userTransports>
<useBuiltinTransports>false</useBuiltinTransports></rtps></participant></profiles>''')
    os.environ.update(ROS_DOMAIN_ID='193',ROS_LOCALHOST_ONLY='1',RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
        FASTRTPS_DEFAULT_PROFILES_FILE=str(xml),FASTDDS_DEFAULT_PROFILES_FILE=str(xml))
    os.environ.pop('ROS_DISCOVERY_SERVER',None)
    return {k:os.environ[k] for k in ('ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','RMW_IMPLEMENTATION','FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE')}


def replay_ros(root,manifest,events,report_path,include_derived=False):
    """Publish recorded sensors on domain193 loopback; capture new commands only."""
    import tempfile
    with tempfile.TemporaryDirectory(prefix='echora-replay-') as directory:
        environment=isolated_environment(directory)
        print('Start replay consumers in another terminal with these environment values:',flush=True)
        print(json.dumps(environment,indent=2),flush=True)
        import cv2
        import rclpy
        from rosidl_runtime_py.utilities import get_message
        from rosidl_runtime_py.set_message import set_message_fields
        from rosidl_runtime_py.convert import message_to_ordereddict
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
        from std_msgs.msg import String
        from geometry_msgs.msg import Twist
        rclpy.init();node=rclpy.create_node('echora_recorded_ev3');publishers={};commands=[]
        def command(topic,message):commands.append(dict(elapsed_seconds=time.monotonic()-started,topic=topic,data=message_to_ordereddict(message)))
        node.create_subscription(Twist,'/cmd_vel',lambda m:command('/cmd_vel',m),10)
        node.create_subscription(String,'/camera_head/command',lambda m:command('/camera_head/command',m),10)
        try:
            # Let separately launched consumers discover publishers before playback.
            for e in events:
                topic=e['topic']
                if topic in COMMAND_TOPICS or (topic in DERIVED_TOPICS and not include_derived):continue
                if topic not in publishers:
                    cls=get_message(e['type']);publishers[topic]=(node.create_publisher(cls,topic,10),cls)
            print('Replay ready; press Enter after starting the isolated consumers.',flush=True)
            input()
            started=time.monotonic();offset=time.time()-source_clock(manifest)
            for e in events:
                while time.monotonic()-started<e['elapsed_ns']/1e9:
                    rclpy.spin_once(node,timeout_sec=min(.05,max(0,e['elapsed_ns']/1e9-(time.monotonic()-started))))
                topic=e['topic']
                if topic not in publishers:continue
                pub,cls=publishers[topic];d=rebase_times(e['data'],offset)
                if e['kind']=='frame':
                    image=cv2.imread(str(root/d['path']));msg=Image()
                    if image is None:raise ValueError('Unreadable recorded image')
                    msg.height,msg.width=image.shape[:2];msg.encoding='bgr8';msg.step=msg.width*3;msg.data=image.tobytes()
                    set_message_fields(msg.header,d['header'])
                elif cls is String:msg=String(data=d if isinstance(d,str) else json.dumps(d))
                else:msg=cls();set_message_fields(msg,d)
                pub.publish(msg);rclpy.spin_once(node,timeout_sec=0)
            return dict(mode='ros',**summary(manifest,events),commands_observed=commands,
                recorded_commands=[e for e in events if e['topic'] in COMMAND_TOPICS],
                feedback='Recorded trajectory; new commands are logged and do not change camera views or motor feedback.')
        finally:node.destroy_node();rclpy.shutdown()


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('recording',type=Path);p.add_argument('--mode',choices=('inspect','tracker','ros'),default='inspect')
    p.add_argument('--report',type=Path,required=True);p.add_argument('--profile-id')
    p.add_argument('--baseline-tracker',type=Path,help='Tracker mode: compare a previous wardrobe_tracking.py against current code')
    p.add_argument('--include-derived',action='store_true',help='ROS mode only: replay original tracking outputs instead of recomputing them')
    args=p.parse_args(argv);root,manifest,events=load_recording(args.recording)
    if args.mode=='tracker':
        result=replay_tracker(root,events,args.profile_id)
        if args.baseline_tracker:
            import importlib.util
            spec=importlib.util.spec_from_file_location('robot.jetson.perception._replay_baseline',args.baseline_tracker)
            baseline=importlib.util.module_from_spec(spec);spec.loader.exec_module(baseline)
            old=replay_tracker(root,events,args.profile_id,baseline.WardrobeTracker)
            result['baseline']=old
            result['retained_frame_change']=result['frames_with_retained_identity']-old['frames_with_retained_identity']
    elif args.mode=='ros':result=replay_ros(root,manifest,events,args.report,args.include_derived)
    else:result=summary(manifest,events)
    args.report.parent.mkdir(parents=True,exist_ok=True);args.report.write_text(json.dumps(result,indent=2)+'\n')
    compact={k:v for k,v in result.items() if k not in ('observations','commands_observed','recorded_commands','baseline')}
    if 'baseline' in result:
        compact['baseline_retained_frames']=result['baseline']['frames_with_retained_identity']
    print(json.dumps(compact,indent=2))


if __name__=='__main__':main()
