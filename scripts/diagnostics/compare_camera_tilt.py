#!/usr/bin/env python3
"""Compare image-only and depth-derived tilt. No motion or image storage."""
import argparse
import base64
import hashlib
import io
import json
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

# This short reader runs in the existing Jetson ROS environment. The JPEG is
# passed directly into the local process, never into a file or service setting.
REMOTE_CAPTURE = r'''
source /opt/ros/humble/setup.bash
cd /home/animesh/echora
python3 - <<'PY'
import base64,hashlib,json,time
import cv2,rclpy,yaml
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
from range_capture import capture_pose
rclpy.init();node=rclpy.create_node('stationary_tilt_comparison')
state={};at={};result=[];bridge=CvBridge();frames={};boxes={}
want_person=False
def key(message):
    return (message.header.frame_id,message.header.stamp.sec,message.header.stamp.nanosec)
def complete(frame_key):
    if result or frame_key not in frames:return
    if want_person and not boxes.get(frame_key):return
    value=frames[frame_key]
    if want_person:value['person_boxes']=boxes[frame_key]
    result.append(value)
def detection(message):
    frame_key=key(message);found=[]
    for d in message.detections:
        x,y,w,h=d.bbox.center.position.x,d.bbox.center.position.y,d.bbox.size_x,d.bbox.size_y
        found.append([x-w/2,y-h/2,x+w/2,y+h/2])
    boxes[frame_key]=found
    while len(boxes)>16:boxes.pop(next(iter(boxes)))
    complete(frame_key)
def status(role,message):
    try: state[role]=json.loads(message.data);at[role]=time.monotonic()
    except ValueError: pass
def image(message):
    now=time.monotonic()
    stamped=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
    if not 0<=time.time()-stamped<=1.2:return
    pose=capture_pose(state.get('head'),state.get('robot'),now-at.get('head',0),now-at.get('robot',0))
    if pose is None or result:return
    frame=bridge.imgmsg_to_cv2(message,desired_encoding='bgr8')
    ok,encoded=cv2.imencode('.jpg',frame)
    if not ok:return
    jpeg=encoded.tobytes()
    frame_key=key(message)
    frames[frame_key]=dict(image=base64.b64encode(jpeg).decode(),image_sha256=hashlib.sha256(jpeg).hexdigest(),
        pose_binding=pose,frame_key=list(frame_key))
    while len(frames)>16:frames.pop(next(iter(frames)))
    complete(frame_key)
node.create_subscription(String,'/camera_head/status',lambda m:status('head',m),1)
node.create_subscription(String,'/robot_status',lambda m:status('robot',m),1)
node.create_subscription(Image,'/camera/image_raw',image,qos_profile_sensor_data)
if want_person:node.create_subscription(Detection2DArray,'/perception/person_detections',detection,10)
deadline=time.monotonic()+8
try:
    while not result and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
finally:node.destroy_node();rclpy.shutdown()
if not result:raise SystemExit('No frame with fresh stopped hardware feedback')
with open('camera_calibration.yaml') as handle:result[0]['camera_yaml']=yaml.safe_load(handle)
print(json.dumps(result[0]))
PY
'''


def capture_live(host,person=False):
    from urllib.request import urlopen
    with urlopen('http://'+host+':8080/api/status', timeout=4) as response:
        status=json.load(response)
    if status.get('mission',{}).get('running') or status.get('manual_drive',{}).get('active'):
        raise ValueError('Finish movement before comparing camera angles')
    process=subprocess.run(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=5',
                           'animesh@'+host,'bash -s'],input=REMOTE_CAPTURE.replace('want_person=False','want_person='+repr(bool(person))),
                           text=True,capture_output=True,timeout=18)
    if process.returncode:
        raise ValueError('Stationary capture failed: '+(process.stderr.strip()[-600:] or 'Jetson reader exited'))
    capture=json.loads(process.stdout)
    jpeg=base64.b64decode(capture.pop('image'),validate=True)
    if hashlib.sha256(jpeg).hexdigest()!=capture['image_sha256']:
        raise ValueError('Image binding changed')
    return jpeg,capture


class TiltComparison:
    def __init__(self):
        import torch
        from geocalib import GeoCalib
        from robot.mac.person_range import MetricDepthBackend
        from robot.mac.route_perception import RoutePerceptionEngine
        torch.set_num_threads(4)
        self.orientation=GeoCalib().to('cpu')
        self.depth=MetricDepthBackend()
        self.segmentation=RoutePerceptionEngine()

    def compare(self,jpeg,camera_yaml):
        import cv2
        import numpy as np
        import torch
        from PIL import Image
        from robot.mac.person_range import floor_plane_orientation
        image=Image.open(io.BytesIO(jpeg)).convert('RGB')
        width,height=image.size
        if [width,height]!=[camera_yaml['image_width'],camera_yaml['image_height']]:
            raise ValueError('Image dimensions differ from calibrated intrinsics')
        matrix=np.asarray(camera_yaml['camera_matrix']['data'],dtype=float).reshape(3,3)
        distortion=np.asarray(camera_yaml['distortion_coefficients']['data'],dtype=float)
        intrinsics=dict(fx=matrix[0,0],fy=matrix[1,1],cx=matrix[0,2],cy=matrix[1,2])
        # GeoCalib assumes a centred principal point and equal focal lengths.
        # Rectify into that camera without rotating the physical optical axis.
        focal=matrix[1,1]
        centred=np.array([[focal,0,width/2],[0,focal,height/2],[0,0,1.]])
        rectified=cv2.undistort(np.array(image),matrix,distortion,None,centred)
        tensor=torch.from_numpy(np.ascontiguousarray(rectified.transpose(2,0,1))).float()/255.
        started=time.monotonic()
        value=self.orientation.calibrate(tensor,priors={'focal':torch.tensor(focal,dtype=torch.float32)})
        # GeoCalib's vector represents world up in image camera coordinates.
        up=value['gravity'].vec3d[0].cpu().numpy()
        down=-up
        geometric=dict(source='geocalib_image_gravity',pitch_degrees=float(np.degrees(np.arctan2(down[2],down[1]))),
            down_vector=down.tolist(),reported_pitch_uncertainty_degrees=float(torch.rad2deg(value['pitch_uncertainty']).item()),
            seconds=time.monotonic()-started,validated=False)
        started=time.monotonic()
        depth=self.depth.infer(image);masks=self.segmentation.infer(jpeg,return_masks=True)
        try:
            fitted=floor_plane_orientation(depth,masks['floor'],intrinsics,distortion)
            difference=geometric['pitch_degrees']-fitted['pitch_degrees']
        except ValueError as exc:
            fitted=dict(available=False,reason=str(exc));difference=None
        return dict(image_sha256=hashlib.sha256(jpeg).hexdigest(),image_size=[width,height],
            image_gravity=geometric,depth_floor=fitted,pitch_difference_degrees=difference,
            depth_and_segmentation_seconds=time.monotonic()-started,
            camera_intrinsics=intrinsics,validated=False)


def main():
    import yaml
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--live',action='store_true')
    parser.add_argument('--host',default='192.168.1.48')
    parser.add_argument('--image',action='append',default=[])
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    if not args.live and not args.image:parser.error('Choose --live or --image')
    runner=TiltComparison();reports=[]
    if args.live:
        jpeg,binding=capture_live(args.host)
        camera=binding.pop('camera_yaml')
        reports.append(dict(runner.compare(jpeg,camera),capture=binding,input='live'))
        print(json.dumps(reports[-1]),flush=True)
    camera=yaml.safe_load((ROOT/'config/camera_calibration.yaml').read_text())
    for path in args.image:
        reports.append(dict(runner.compare(Path(path).read_bytes(),camera),input=path))
        print(json.dumps(reports[-1]),flush=True)
    output=Path(args.output);output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(dict(schema=1,observation_only=True,commands_sent=False,images_saved=False,
        validated=False,comparisons=reports),indent=2)+'\n')


if __name__=='__main__':main()
