#!/usr/bin/env python3
"""Run the production short movement against isolated simulated ROS feedback."""
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

p=argparse.ArgumentParser()
p.add_argument('--controller',required=True)
p.add_argument('--scenario',choices=('clear','blocked','blocked_after_turn','unknown_center','odom_lost','camera_lost','reverse','veering','stalled','external_motion'),default='clear')
p.add_argument('--report',required=True)
a=p.parse_args()
assert os.environ.get('ROS_DOMAIN_ID')=='191' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
sys.path.insert(0,str(Path(a.controller).resolve().parent))
spec=importlib.util.spec_from_file_location('approach_under_test',a.controller)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
rclpy.init()

class World(Node):
    def __init__(self):
        super().__init__('approach_simulator')
        self.head=dict(available=True,position=40,down_position=40,forward_position=13,up_position=-14,
                       moving=False,homing=False,homed=True,calibrated=True,
                       reference_id='sim',approved_reference_id='sim',settle_tolerance=3)
        self.x=0.;self.y=0.;self.yaw=0.;self.linear=0.;self.angular=0.
        self.drive_started=None;self.previous=time.monotonic();self.commands=[];self.route_calls=0
        self.robot_pub=self.create_publisher(String,'/robot_status',10)
        self.head_pub=self.create_publisher(String,'/camera_head/status',10)
        self.image_pub=self.create_publisher(Image,'/camera/image_raw',1)
        self.odom_pub=self.create_publisher(Odometry,'/odom',10)
        self.create_subscription(Twist,'/cmd_vel',self.command,10)
        self.create_subscription(String,'/camera_head/command',self.head_command,10)
        self.create_timer(.05,self.tick)
    def command(self,m):
        self.linear,self.angular=m.linear.x,m.angular.z
        self.commands.append((time.monotonic(),self.linear,self.angular))
        if self.linear and self.drive_started is None:self.drive_started=time.monotonic()
    def head_command(self,m):
        raise AssertionError('Camera starts at the approved floor view: '+m.data)
    def tick(self):
        now=time.monotonic();dt=now-self.previous;self.previous=now
        active=self.drive_started is not None
        self.yaw+=self.angular*dt
        if active and a.scenario=='veering' and self.linear:self.yaw+=.8*dt
        speed=0 if a.scenario=='stalled' else -self.linear if a.scenario=='reverse' else self.linear
        self.x+=speed*math.cos(self.yaw)*dt;self.y+=speed*math.sin(self.yaw)*dt
        self.robot_pub.publish(String(data=json.dumps(dict(motion_active=bool(self.linear or self.angular)))))
        self.head_pub.publish(String(data=json.dumps(self.head)))
        frame=Image();frame.width=96;frame.height=72;frame.step=288;frame.encoding='bgr8';frame.data=bytes([100])*20736
        if not(active and a.scenario=='camera_lost'):self.image_pub.publish(frame)
        odom=Odometry();odom.pose.pose.position.x=self.x;odom.pose.pose.position.y=self.y
        odom.pose.pose.orientation.w=math.cos(self.yaw/2);odom.pose.pose.orientation.z=math.sin(self.yaw/2)
        if not(active and a.scenario=='odom_lost'):self.odom_pub.publish(odom)
    def route(self,*args):
        if a.scenario == 'external_motion':
            self.linear=.02
            time.sleep(.3)
        time.sleep(.1)
        self.route_calls+=1
        blocked=a.scenario=='blocked' or a.scenario=='blocked_after_turn' and self.route_calls>1
        return dict(ok=True,result_age_seconds=.1,frame_sha256=str(self.route_calls),
                    camera_head=dict(self.head),decision=dict(blocked=blocked,heading_degrees=15 if self.route_calls==1 else 0,distance_m=.1),
                    evidence=[dict(heading_degrees=0,floor_fraction=.99,
                                   known_fraction=.1 if a.scenario=='unknown_center' else .99,sample_count=100)])

world=World();executor=SingleThreadedExecutor();executor.add_node(world)
thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
try:
    args=module.parse_args(['--route-url','http://simulated','--execute','--cable-zero-confirmed'])
    with patch.object(rclpy,'init'),patch.object(rclpy,'shutdown'),patch.object(module,'fetch_route',side_effect=world.route):
        result=module.run(args)
    assert world.commands[-1][1:]==(0.,0.),world.commands[-1]
    assert all(abs(x)<=.03 and abs(z)<=.3 for _,x,z in world.commands)
    if a.scenario=='clear':
        assert result['outcome']=='success',result
        assert .039<=result['travelled_m']<=.055,result
        assert world.route_calls==2
    else:
        expected={'blocked':'reports blocked','blocked_after_turn':'reports blocked',
                  'unknown_center':'not proven clear','odom_lost':'odometry feedback is stale',
                  'camera_lost':'camera frames are stale','reverse':'backwards',
                  'veering':'veered','stalled':'no progress','external_motion':'robot moved while checking'}[a.scenario]
        assert result['outcome']=='failure' and expected in result.get('error',''),result
        if a.scenario in ('blocked','blocked_after_turn','unknown_center'):
            assert not any(x for _,x,z in world.commands),world.commands
        if a.scenario=='blocked':assert not any(z for _,x,z in world.commands)
        if a.scenario in ('odom_lost','camera_lost'):
            delay=max(t for t,x,z in world.commands if x)-world.drive_started
            assert delay<1.15,delay
            result['last_drive_command_after_loss_seconds']=round(delay,3)
    result.update(simulation=True,scenario=a.scenario,route_calls=world.route_calls,
                  simulated_displacement_m=round(math.hypot(world.x,world.y),4),
                  nonzero_drive_commands=sum(bool(x) for _,x,z in world.commands))
    Path(a.report).write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:result.get(k) for k in ('scenario','outcome','error','travelled_m','route_calls')}))
finally:
    executor.shutdown();thread.join(timeout=2);world.destroy_node();rclpy.shutdown()
