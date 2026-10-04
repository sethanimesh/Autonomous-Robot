#!/usr/bin/env python3
"""Run the production vertical scan on simulated ROS topics; no physical motors."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import sys
import threading
import time
from unittest.mock import patch

p = argparse.ArgumentParser()
p.add_argument('--scanner', required=True)
p.add_argument('--scenario', choices=('identified', 'no_identity', 'no_body', 'unreferenced', 'narrow_face', 'missed_body'), default='identified')
p.add_argument('--report', required=True)
a = p.parse_args()
assert os.environ.get('ROS_DOMAIN_ID') == '189' and os.environ.get('ROS_LOCALHOST_ONLY') == '1'
sys.path.insert(0, str(Path(a.scanner).resolve().parent))
spec = importlib.util.spec_from_file_location('scan_under_test', a.scanner)
scan = importlib.util.module_from_spec(spec);spec.loader.exec_module(scan)
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2D, Detection2DArray
rclpy.init()

class World(Node):
    def __init__(self):
        super().__init__('person_scan_simulator')
        self.head = dict(position=11, forward_position=12, up_position=-7,
                         target_position=12, moving=False, homing=False, homed=a.scenario!='unreferenced',
                         calibrated=True, reference_id='sim', approved_reference_id='sim',
                         minimum_position=-10, maximum_position=35,
                         minimum_target_position=-7, maximum_target_position=32, settle_tolerance=3)
        self.head_commands = [];self.drive = [];self.pending = None;self.frame_number = 0
        self.robot_pub = self.create_publisher(String, '/robot_status', 10)
        self.head_pub = self.create_publisher(String, '/camera_head/status', 10)
        self.camera_pub = self.create_publisher(String, '/camera/status', 10)
        self.target_pub = self.create_publisher(String, '/mission/target_observation', 10)
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 1)
        self.person_pub = self.create_publisher(Detection2DArray, '/perception/person_detections', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.create_subscription(String, '/camera_head/command', self.command, 10)
        self.create_subscription(Twist, '/cmd_vel', lambda m:self.drive.append((m.linear.x,m.angular.z)), 10)
        self.create_timer(.05, self.tick)

    def command(self, message):
        try: command = json.loads(message.data)
        except ValueError: command = dict(action=message.data)
        self.head_commands.append(command)
        if command['action']=='jog_to':
            assert -7 <= command['target'] <= 32
            assert abs(command['target']-self.head['position']) <= 15
            self.pending = (time.monotonic(), command['target'])
        elif command['action']=='look_forward':self.pending = (time.monotonic(), 12)
        elif command['action']=='stop':self.pending=None;self.head['moving']=False
        else:raise RuntimeError('Unexpected camera command '+str(command))

    def tick(self):
        if self.pending:
            since,target=self.pending
            if time.monotonic()-since > .2:self.head.update(target_position=target,moving=True)
            if time.monotonic()-since > .4:
                self.head.update(position=target+1,moving=False);self.pending=None
        found = (a.scenario in ('identified', 'missed_body') and self.head['position']<=-5
                 or a.scenario=='narrow_face' and 1 <= self.head['position'] <= 4) and not self.head['moving']
        for pub,val in ((self.robot_pub,dict(track_motion_active=False,motion_active=False)),
                        (self.head_pub,self.head),
                        (self.camera_pub,dict(state='streaming',mean_intensity=120)),
                        (self.target_pub,dict(ok=found,confirmed=found,age_seconds=.05,
                                              box_height_fraction=.3 if found else 0))):
            pub.publish(String(data=json.dumps(val)))
        self.frame_number+=1
        frame=Image();frame.width=96;frame.height=72;frame.step=288;frame.encoding='bgr8'
        frame.data=bytes(30 if (x//6+y//6+self.frame_number)%2 else 210
                         for y in range(72) for x in range(96) for c in range(3))
        self.image_pub.publish(frame)
        people=Detection2DArray()
        if a.scenario not in ('no_body', 'missed_body'):
            d=Detection2D();d.bbox.center.position.x=48.;d.bbox.center.position.y=36.
            d.bbox.size_x=92.;d.bbox.size_y=70.;people.detections=[d]
        self.person_pub.publish(people)
        odom=Odometry();odom.pose.pose.orientation.w=1.;self.odom_pub.publish(odom)

world=World();executor=SingleThreadedExecutor();executor.add_node(world)
thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
try:
    args=scan.parse_args(['--execute','--vertical-only','--try-up'])
    with patch.object(rclpy,'init'),patch.object(rclpy,'shutdown'):
        result=scan.run(args)
    assert not any(x or z for x,z in world.drive), world.drive
    if a.scenario in ('identified', 'narrow_face', 'missed_body'):
        assert result['outcome']=='target_found', result
        if a.scenario == 'missed_body':assert result.get('high_view_checks') == [0]
        else:assert len(result.get('body_guided_tilts',[]))==1
        if a.scenario == 'narrow_face':assert 1 <= world.head['position'] <= 4
        else:assert world.head['position']<=-5, world.head
        assert all(abs(c['target']-p) <= 5 for p,c in zip(
            [11]+result['camera_tilt_positions'], [c for c in world.head_commands if c['action']=='jog_to']))
    elif a.scenario=='unreferenced':
        assert result['outcome']=='failure', result
        assert not any(c['action']!='stop' for c in world.head_commands)
    else:
        assert result['outcome']=='vertical_scan_complete_no_target', result
        if a.scenario=='no_body':assert result.get('high_view_checks') == [0] and result.get('camera_tilt_positions')
    result.update(simulation=True,scenario=a.scenario,head_commands=world.head_commands,
                  nonzero_chassis_commands=sum(bool(x or z) for x,z in world.drive))
    Path(a.report).write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(scenario=a.scenario,outcome=result['outcome'],
                         elapsed=round(result['finished_at_unix']-result['started_at_unix'],1),
                         head_commands=world.head_commands)))
finally:
    executor.shutdown();thread.join(timeout=2);world.destroy_node();rclpy.shutdown()
