#!/usr/bin/env python3
"""Run the production chassis scan on simulated ROS topics; no physical motors."""
import argparse
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
from unittest.mock import patch

p = argparse.ArgumentParser()
p.add_argument('--scanner', required=True)
p.add_argument('--scenario', choices=('empty', 'target_right', 'camera_lost', 'odom_lost', 'wrong_direction', 'stuck', 'head_moving'), default='empty')
p.add_argument('--report', required=True)
a = p.parse_args()
assert os.environ.get('ROS_DOMAIN_ID') == '190' and os.environ.get('ROS_LOCALHOST_ONLY') == '1'
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
                         target_position=12, moving=False, homing=False, homed=True,
                         calibrated=True, reference_id='sim', approved_reference_id='sim',
                         minimum_position=-10, maximum_position=35,
                         minimum_target_position=-7, maximum_target_position=32, settle_tolerance=3)
        self.head_commands = [];self.drive = [];self.pending = None;self.frame_number = 0
        self.yaw = 0.0;self.angular = 0.0;self.fault_at = None;self.first_drive_at = None
        self.previous_tick = time.monotonic()
        self.robot_pub = self.create_publisher(String, '/robot_status', 10)
        self.head_pub = self.create_publisher(String, '/camera_head/status', 10)
        self.camera_pub = self.create_publisher(String, '/camera/status', 10)
        self.target_pub = self.create_publisher(String, '/mission/target_observation', 10)
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 1)
        self.person_pub = self.create_publisher(Detection2DArray, '/perception/person_detections', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.create_subscription(String, '/camera_head/command', self.command, 10)
        self.create_subscription(Twist, '/cmd_vel', self.velocity, 10)
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

    def velocity(self, message):
        now = time.monotonic()
        self.drive.append((now, message.linear.x, message.angular.z))
        self.angular = message.angular.z
        if self.angular and self.first_drive_at is None:self.first_drive_at=now

    def tick(self):
        now = time.monotonic();dt=now-self.previous_tick;self.previous_tick=now
        if self.first_drive_at is not None and now-self.first_drive_at>.25 and self.fault_at is None:
            self.fault_at=now
        fault = self.fault_at is not None
        if a.scenario != 'stuck':
            self.yaw += self.angular * dt * (-1 if a.scenario == 'wrong_direction' else 1)
        self.head['moving'] = fault and a.scenario == 'head_moving'

        if self.pending:
            since,target=self.pending
            if time.monotonic()-since > .2:self.head.update(target_position=target,moving=True)
            if time.monotonic()-since > .4:
                self.head.update(position=target+1,moving=False);self.pending=None
        found = a.scenario=='target_right' and self.yaw < -math.radians(8) and abs(self.angular)<.001
        for pub,val in ((self.robot_pub,dict(track_motion_active=bool(self.angular),motion_active=bool(self.angular))),
                        (self.head_pub,self.head),
                        (self.camera_pub,dict(state='streaming',mean_intensity=120)),
                        (self.target_pub,dict(ok=found,confirmed=found,age_seconds=.05,
                                              box_height_fraction=.3 if found else 0))):
            pub.publish(String(data=json.dumps(val)))
        self.frame_number+=1
        frame=Image();frame.width=96;frame.height=72;frame.step=288;frame.encoding='bgr8'
        frame.data=bytes(30 if (x//6+y//6+self.frame_number)%2 else 210
                         for y in range(72) for x in range(96) for c in range(3))
        if not (fault and a.scenario == 'camera_lost'):self.image_pub.publish(frame)
        people=Detection2DArray()
        self.person_pub.publish(people)
        odom=Odometry();odom.pose.pose.orientation.w=math.cos(self.yaw/2);odom.pose.pose.orientation.z=math.sin(self.yaw/2)
        if not (fault and a.scenario == 'odom_lost'):self.odom_pub.publish(odom)

world=World();executor=SingleThreadedExecutor();executor.add_node(world)
thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
try:
    args=scan.parse_args(['--execute','--cable-zero-confirmed','--no-search-up','--step-degrees','15','--sweep-limit-degrees','15','--dwell-seconds','.2'])
    with patch.object(rclpy,'init'),patch.object(rclpy,'shutdown'):
        result=scan.run(args)
    assert not any(x for _,x,_ in world.drive), world.drive
    assert world.drive[-1][2] == 0, world.drive[-1]
    assert all(c['action']=='stop' for c in world.head_commands), world.head_commands
    if a.scenario=='empty':
        assert result['outcome']=='scan_complete_no_target', result
        assert len(result['turns'])==4, result
        assert abs(math.degrees(world.yaw))<=4, world.yaw
    elif a.scenario=='target_right':
        assert result['outcome']=='target_found', result
        assert len(result['turns'])==1, result
    else:
        expected={'camera_lost':'frame heartbeat', 'odom_lost':'odometry is stale',
                  'wrong_direction':'wrong direction', 'stuck':'no progress',
                  'head_moving':'head is moving'}[a.scenario]
        assert result['outcome']=='failure' and expected in result.get('error',''), result
        assert result['final_cable_heading_degrees'] is None, result
        delay=max(t for t,x,z in world.drive if z)-world.fault_at
        assert delay <= (2.1 if a.scenario=='stuck' else 1.15 if a.scenario=='odom_lost' else .65), delay
        result['last_motion_command_after_fault_seconds']=round(delay,3)
    result.update(simulation=True,scenario=a.scenario,head_commands=world.head_commands,
                  nonzero_chassis_commands=sum(bool(z) for _,_,z in world.drive),
                  final_simulated_yaw_degrees=round(math.degrees(world.yaw),3))
    Path(a.report).write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(scenario=a.scenario,outcome=result['outcome'],
                         elapsed=round(result['finished_at_unix']-result['started_at_unix'],1),
                         head_commands=world.head_commands)))
finally:
    executor.shutdown();thread.join(timeout=2);world.destroy_node();rclpy.shutdown()
