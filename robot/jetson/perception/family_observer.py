"""ROS adapter for persistent family tracking, shared by all scan workers."""
import base64
from collections import OrderedDict
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import math
import os
import time
import uuid

try:
    from .family_store import FamilyStore
    from .wardrobe_tracking import WardrobeTracker, post_json
except ImportError:
    from family_store import FamilyStore
    from wardrobe_tracking import WardrobeTracker, post_json


class FamilyObserver:
    def __init__(self,node):
        from sensor_msgs.msg import Image
        from std_msgs.msg import String
        from vision_msgs.msg import Detection2DArray
        from rclpy.qos import qos_profile_sensor_data
        self.node=node
        node.declare_parameter('family_database','/home/animesh/echora/data/people.sqlite3')
        node.declare_parameter('wardrobe_url','http://127.0.0.1:18091/wardrobe')
        node.declare_parameter('person_range_url','http://127.0.0.1:18091/person-range')
        node.declare_parameter('clothing_approach_enabled',True)
        node.declare_parameter('approximate_approach_enabled',True)
        node.declare_parameter('range_calibration_file','/home/animesh/echora/data/person_range_calibration.json')
        node.declare_parameter('camera_intrinsics_file','/home/animesh/echora/camera_calibration.yaml')
        self.store=FamilyStore(node.get_parameter('family_database').value)
        self.tracker=WardrobeTracker(self.store,node.get_parameter('wardrobe_url').value)
        self.cache={k:OrderedDict() for k in ('image','people','faces')}
        self.robot={};self.head={};self.pose=None;self.motion=False;self.not_before=0.
        self.image=None;self.image_frame_key=None;self.last_range_at=-100.;self.range_future=None;self.range_pending=None
        self.last_frame_time=0.
        self.range_executor=ThreadPoolExecutor(max_workers=1)
        self.calibration={};self.calibration_mtime=None;self.diagnostic=None
        self.intrinsics=None;self.range_camera=None
        try:
            import yaml
            with open(node.get_parameter('camera_intrinsics_file').value) as f:camera=yaml.safe_load(f)
            matrix=camera['camera_matrix']['data']
            self.intrinsics=dict(fx=matrix[0],cx=matrix[2],width=camera['image_width'])
            self.range_camera=dict(image_size=[camera['image_width'],camera['image_height']],
                intrinsics=dict(fx=matrix[0],fy=matrix[4],cx=matrix[2],cy=matrix[5]),
                distortion=camera.get('distortion_coefficients',{}).get('data',[]))
        except (OSError,KeyError,ValueError,TypeError):pass
        self.publisher=node.create_publisher(String,'/perception/people_tracks',10)
        node.create_subscription(Image,'/camera/image_raw',lambda m:self.offer('image',m),qos_profile_sensor_data)
        node.create_subscription(Detection2DArray,'/perception/person_detections',lambda m:self.offer('people',m),10)
        node.create_subscription(String,'/perception/family_matches',lambda m:self.offer('faces',m),10)
        node.create_subscription(String,'/robot_status',lambda m:self.on_motion('robot',m),10)
        node.create_subscription(String,'/camera_head/status',lambda m:self.on_motion('head',m),10)
        node.create_timer(.05,self.drain)

    def now(self):return self.node.get_clock().now().nanoseconds/1e9

    def close(self):
        self.tracker.close();self.range_executor.shutdown(wait=False,cancel_futures=True)

    def on_motion(self,kind,message):
        try:value=json.loads(message.data)
        except ValueError:return
        if not isinstance(value,dict):return
        setattr(self,kind,value)
        motors=self.robot.get('motors',{})
        moving=bool(self.head.get('moving') or self.head.get('homing') or
                    any(motors.get(k,{}).get('moving') or any(abs(float(motors.get(k,{}).get(f,0)))>1 for f in ('speed','commanded_speed')) or 'running' in motors.get(k,{}).get('state',[]) for k in ('left','right')))
        pose=(self.head.get('reference_id'),self.head.get('position'),
              motors.get('left',{}).get('generation'),motors.get('right',{}).get('generation'),
              motors.get('left',{}).get('position'),motors.get('right',{}).get('position'))
        changed=self.pose is not None and (pose[0]!=self.pose[0] or pose[2:4]!=self.pose[2:4] or
                any(type(pose[i]) in (int,float) and type(self.pose[i]) in (int,float) and abs(pose[i]-self.pose[i])>3 for i in (1,4,5)))
        if moving != self.motion or changed:
            self.tracker.invalidate_position()
            self.not_before=self.now()
        if changed or self.pose is None:self.pose=pose
        self.motion=moving

    def offer(self,kind,message):
        try:
            if kind=='faces':
                message=json.loads(message.data);key=tuple(message['frame_key'])
            else:
                key=(message.header.frame_id,message.header.stamp.sec,message.header.stamp.nanosec)
            self.cache[kind][key]=message
            while len(self.cache[kind])>48:self.cache[kind].popitem(last=False)
            self.drain()
        except (ValueError,KeyError,TypeError,AttributeError) as exc:
            self.diagnostic=type(exc).__name__

    def drain(self):
        # Local tracking does not depend on a face worker producing a result.
        # Briefly allow its same-frame answer to arrive, then use body evidence.
        now=self.now()
        ready=[key for key in self.cache['people'] if key in self.cache['image']
               and (key in self.cache['faces'] or now-(key[1]+key[2]/1e9)>=.35)]
        if not ready:return
        key=max(ready,key=lambda k:(k[1],k[2]))
        try:
            frame=self.cache['image'].pop(key);people=self.cache['people'].pop(key)
            faces=self.cache['faces'].pop(key,{'faces':[]})
            source_time=key[1]+key[2]/1e9
            if self.motion or source_time<=max(self.not_before,self.last_frame_time) or not 0<=now-source_time<=1.:return
            import numpy as np
            if frame.encoding not in ('bgr8','rgb8') or frame.step<frame.width*3:return
            image=np.frombuffer(frame.data,np.uint8).reshape(frame.height,frame.step)[:,:frame.width*3].reshape(frame.height,frame.width,3)
            if frame.encoding=='rgb8':image=image[:,:,::-1]
            boxes=[]
            for d in people.detections:
                x,y,w,h=d.bbox.center.position.x,d.bbox.center.position.y,d.bbox.size_x,d.bbox.size_y
                if w>0 and h>0:boxes.append([x-w/2,y-h/2,x+w/2,y+h/2])
            self.image=image.copy()
            self.image_frame_key=list(key)
            self.tracker.observe(image,boxes,faces['faces'],list(key),source_time)
            self.last_frame_time=source_time
            self.diagnostic=None
        except (ValueError,KeyError,TypeError,AttributeError) as exc:
            self.diagnostic=type(exc).__name__

    def range_poll(self,t,now):
        if self.range_future is not None and self.range_future.done():
            future,p=self.range_future,self.range_pending;self.range_future=self.range_pending=None
            try:
                r=future.result();current=self.tracker.tracks.get(p['binding']['track_id'])
                if (r.get('binding')==p['binding'] and current and current['epoch']==p['binding']['epoch']
                        and current.get('profile_id')==p['binding']['profile_id']
                        and current.get('revision')==p['binding']['revision']
                        and current.get('position_valid') and not self.motion
                        and 0<=now-p['observed_at']<=3 and 0<=now-current['seen_at']<=1.):
                    old = current.get('range') or {}
                    value = r.get('range') or {}
                    consistent = (0<p['observed_at']-old.get('observed_at',-100)<=3
                                  and old.get('source')==value.get('source')
                                  and old.get('distance_reference')==value.get('distance_reference')
                                  and old.get('validated')==value.get('validated')
                                  and type(old.get('distance_m')) in (int,float)
                                  and type(value.get('distance_m')) in (int,float)
                                  and abs(old['distance_m']-value['distance_m']) <= .15)
                    current['range']=dict(value,observed_at=p['observed_at'],frame_key=p['binding']['frame_key'],
                        track_id=p['binding']['track_id'],profile_id=p['binding']['profile_id'],
                        consistent_samples=min(2,old.get('consistent_samples',0)+1) if consistent else 1)
            except Exception as exc:self.diagnostic='Range: '+type(exc).__name__
        if not t or self.motion or self.image is None or self.range_future is not None or now-self.last_range_at<2:return
        if list(t.get('frame_key',[]))!=self.image_frame_key:return
        path=self.node.get_parameter('range_calibration_file').value
        try:
            mtime=os.path.getmtime(path)
            if mtime!=self.calibration_mtime:
                with open(path) as f:self.calibration=json.load(f)
                self.calibration_mtime=mtime
        except (OSError,ValueError):self.calibration={};self.calibration_mtime=None
        approximate=self.node.get_parameter('approximate_approach_enabled').value
        if not self.calibration and not (approximate and self.range_camera):return
        import cv2
        ok,jpeg=cv2.imencode('.jpg',self.image,[cv2.IMWRITE_JPEG_QUALITY,82])
        if not ok:return
        binding=dict(request_id=uuid.uuid4().hex,track_id=t['id'],epoch=t['epoch'],frame_key=t['frame_key'],
                     profile_id=t['profile_id'],revision=t['revision'],image_sha256=hashlib.sha256(jpeg.tobytes()).hexdigest())
        request=dict(binding=binding,image=base64.b64encode(jpeg).decode(),box=t['box'],
                     head=self.head,calibration=self.calibration)
        if approximate and self.range_camera and self.calibration.get('validated') is not True:
            request.update(mode='approximate',camera=self.range_camera)
        self.range_pending=dict(request,observed_at=t['seen_at'])
        self.range_future=self.range_executor.submit(post_json,self.node.get_parameter('person_range_url').value,request)
        self.last_range_at=now

    def enrich(self,payload):
        from std_msgs.msg import String
        now=self.now();self.tracker.poll(now)
        pid=self.store.selected_id();t=self.tracker.selected(pid,now) if pid else None
        self.range_poll(t,now)
        profile=self.store.load(pid) if pid else None
        enabled=self.node.get_parameter('clothing_approach_enabled').value
        observation=dict(profile_id=pid,identity_confirmed=bool(t),identity_source=t['identity_source'] if t else None,
            target_label=profile['label'] if profile else None,target_revision=profile['revision'] if profile else None,
            clothing_approach_enabled=enabled,tracking_state='Tracking' if t else 'Looking for a clearer view',
            approximate_approach_enabled=self.node.get_parameter('approximate_approach_enabled').value,
            wardrobe_error=self.tracker.last_error or self.diagnostic,
            wardrobe_result=getattr(self.tracker,'last_result',None),
            wardrobe_guidance=self.tracker.guidance(now,pid))
        if t:
            h,w=self.image.shape[:2];box=t['box']
            r=t.get('range');r=dict(r,age_seconds=now-r['observed_at']) if r and 0<=now-r.get('observed_at',0)<=3 and not self.motion else None
            observation.update(track_id=t['id'],frame_key=t['frame_key'],age_seconds=now-t['seen_at'],
                body_box=box,center_x_fraction=(box[0]+box[2])/2/w,center_y_fraction=(box[1]+box[3])/2/h,
                frame_time=t['seen_at'],visible_regions=t.get('visible_regions',[]),
                range=r,tracking_state={'face':'Recognized by face','clothing':'Matched saved clothes'}.get(t['identity_source'],'Tracking'),
                target_label=profile['label'],target_revision=profile['revision'])
            if self.intrinsics and w==self.intrinsics['width']:
                observation['bearing_radians']=math.atan(((box[0]+box[2])/2-self.intrinsics['cx'])/self.intrinsics['fx'])
        msg=String();msg.data=json.dumps(observation);self.publisher.publish(msg)
        # Existing face fields keep their original semantics during rollout.
        legacy_fields={'age_seconds','center_x_fraction','center_y_fraction','target_label','target_revision'}
        payload.update({k:v for k,v in observation.items() if enabled or k not in legacy_fields})
        if enabled:
            payload['confirmed']=bool(t and t['identity_source']=='face')
            if payload['confirmed']:
                face=t['face_box']
                payload.update(center_x_fraction=(face[0]+face[2])/2/w,
                    center_y_fraction=(face[1]+face[3])/2/h,box_height_fraction=(face[3]-face[1])/h,
                    top_fraction=face[1]/h,bottom_fraction=face[3]/h)
        return payload
