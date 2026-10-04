#!/usr/bin/env python3
"""Exercise real mission child processes and observer on isolated ROS feedback."""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
import signal
from pathlib import Path
import subprocess
import sys
import threading
import time

parser = argparse.ArgumentParser()
parser.add_argument('--runtime', required=True)
parser.add_argument('--scenario', choices=('clear', 'blocked', 'already_close', 'centering_loss'), default='clear')
parser.add_argument('--report', required=True)
args = parser.parse_args()
assert os.environ.get('ROS_DOMAIN_ID') == '192' and os.environ.get('ROS_LOCALHOST_ONLY') == '1'
runtime = Path(args.runtime).resolve()
report_path = Path(args.report).resolve()
report_path.parent.mkdir(parents=True, exist_ok=True)

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from vision_msgs.msg import Detection2D, Detection2DArray

rclpy.init()


class World(Node):
    def __init__(self):
        super().__init__('integrated_mission_simulator')
        self.head = dict(available=True, position=13, target_position=13,
                         down_position=40, forward_position=13, up_position=-14,
                         minimum_position=-17, maximum_position=43,
                         minimum_target_position=-14, maximum_target_position=40,
                         moving=False, homing=False, homed=True, calibrated=True,
                         manual_override=False, require_approved_reference=True,
                         reference_id='sim', approved_reference_id='sim',
                         settle_tolerance=3, calibration_source='operator_limits',
                         saved_limits=dict(version=1, reference_id='sim', lower=43, upper=-17))
        self.x = self.y = self.yaw = self.linear = self.angular = 0.
        self.previous = time.monotonic()
        self.pending = None
        self.frame_number = self.route_calls = 0
        self.commands = []
        self.head_commands = []
        self.first_drive_x_fraction = None
        self.faults = []
        self.robot_pub = self.create_publisher(String, '/robot_status', 10)
        self.head_pub = self.create_publisher(String, '/camera_head/status', 10)
        self.camera_pub = self.create_publisher(String, '/camera/status', 10)
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 1)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.matches_pub = self.create_publisher(Detection2DArray, '/perception/target_matches', 10)
        self.people_pub = self.create_publisher(Detection2DArray, '/perception/person_detections', 10)
        self.recognition_pub = self.create_publisher(String, '/perception/recognition_status', 10)
        self.create_subscription(Twist, '/cmd_vel', self.velocity, 10)
        self.create_subscription(String, '/camera_head/command', self.head_command, 10)
        self.create_timer(.05, self.tick)

    def target_x(self):
        # ROS yaw is left-positive. The target starts to the right.
        return .95 + math.degrees(self.yaw) / 60.

    def velocity(self, message):
        self.linear, self.angular = message.linear.x, message.angular.z
        self.commands.append((time.monotonic(), self.linear, self.angular))
        if self.linear and self.first_drive_x_fraction is None:
            self.first_drive_x_fraction = self.target_x()
        if self.linear and (abs(self.head['position']-40) > 3 or self.route_calls < 2):
            self.faults.append('Forward command before floor pose and two route checks')

    def head_command(self, message):
        try:
            command = json.loads(message.data)
        except ValueError:
            command = dict(action=message.data)
        self.head_commands.append(command)
        action = command['action']
        if action == 'use_saved_limits':
            self.head.update(calibrated=True, calibration_source='operator_limits')
        elif action == 'invalidate_calibration':
            self.head['calibrated'] = False
        elif action == 'stop':
            self.pending = None
            self.head['moving'] = False
        else:
            target = {'look_forward': 13, 'look_down': 40}.get(action, command.get('target'))
            if target is None or not -14 <= target <= 40:
                self.faults.append('Invalid head target: '+str(command))
                return
            self.pending = (time.monotonic(), target)

    def tick(self):
        now = time.monotonic()
        dt = now-self.previous
        self.previous = now
        self.yaw += self.angular*dt
        self.x += self.linear*math.cos(self.yaw)*dt
        self.y += self.linear*math.sin(self.yaw)*dt
        if self.pending:
            since, target = self.pending
            if now-since > .15:
                self.head.update(target_position=target, moving=True)
            if now-since > .35:
                self.head.update(position=target, moving=False)
                self.pending = None
        self.frame_number += 1
        stamp = self.get_clock().now().to_msg()
        self.robot_pub.publish(String(data=json.dumps(dict(
            track_motion_active=bool(self.linear or self.angular),
            motion_active=bool(self.linear or self.angular)))))
        self.head_pub.publish(String(data=json.dumps(self.head)))
        self.camera_pub.publish(String(data=json.dumps(dict(state='streaming', mean_intensity=120))))
        frame = Image()
        frame.header.stamp = stamp
        frame.width, frame.height, frame.step, frame.encoding = 96, 72, 288, 'bgr8'
        frame.data = bytes(30 if (x//6+y//6+self.frame_number)%2 else 210
                           for y in range(72) for x in range(96) for c in range(3))
        self.image_pub.publish(frame)
        info = CameraInfo()
        info.header.stamp = stamp
        info.width, info.height = 96, 72
        self.info_pub.publish(info)
        odom = Odometry()
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation.w = math.cos(self.yaw/2)
        odom.pose.pose.orientation.z = math.sin(self.yaw/2)
        self.odom_pub.publish(odom)
        visible = (self.head['position'] <= 8 and not self.head['moving']
                   and abs(self.angular) < .001 and 0 < self.target_x() < 1)
        if args.scenario == 'centering_loss' and abs(self.yaw) > .01:
            visible = False
        matches = Detection2DArray()
        matches.header.stamp = stamp
        if visible:
            detection = Detection2D()
            detection.bbox.center.position.x = self.target_x()*96
            detection.bbox.center.position.y = 30.
            detection.bbox.size_x = 8.
            detection.bbox.size_y = (.2 if args.scenario == 'already_close' else .08)*72
            matches.detections = [detection]
        self.matches_pub.publish(matches)
        # Exercise the fallback that raises the head even if YOLO misses a body.
        self.people_pub.publish(Detection2DArray())
        self.recognition_pub.publish(String(data=json.dumps(dict(
            state='target_confirmed' if visible else 'searching', confirmation=dict(confirmed=visible)))))


world = World()


class RouteHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        world.route_calls += 1
        payload = dict(ok=True, result_age_seconds=.05, frame_sha256=str(world.route_calls),
                       camera_head=dict(world.head),
                       decision=dict(blocked=args.scenario == 'blocked', heading_degrees=0, distance_m=.05),
                       evidence=[dict(heading_degrees=0, floor_fraction=.99, known_fraction=.99, sample_count=100)])
        body = json.dumps(payload).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *unused):
        pass


server = ThreadingHTTPServer(('127.0.0.1', 0), RouteHandler)
server_thread = threading.Thread(target=server.serve_forever, daemon=True)
server_thread.start()
executor = SingleThreadedExecutor()
executor.add_node(world)
ros_thread = threading.Thread(target=executor.spin, daemon=True)
ros_thread.start()
observer = subprocess.Popen([sys.executable, str(runtime/'target_observer.py')], stdout=subprocess.DEVNULL)
try:
    command = [sys.executable, str(runtime/'autonomous_find.py'), '--execute', '--one-step-test',
               '--cable-zero-confirmed', '--report', str(report_path),
               '--scan-script', str(runtime/'bounded_target_scan.py'),
               '--calibration-script', str(runtime/'camera_head_calibration.py'),
               '--approach-script', str(runtime/'closed_loop_detour.py'),
               '--route-url', 'http://127.0.0.1:'+str(server.server_port)+'/route']
    completed = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=180)
    assert report_path.exists(), completed.stdout[-2000:]
    result = json.loads(report_path.read_text())
    expected = {'clear': 'step_complete_target_reacquired', 'blocked': 'failure',
                'already_close': 'target_found_at_standoff', 'centering_loss': 'failure'}[args.scenario]
    assert result['outcome'] == expected, result
    assert observer.poll() is None, 'Observer exited'
    assert not world.faults, world.faults
    assert world.commands and world.commands[-1][1:] == (0., 0.), world.commands[-1:]
    assert all(abs(x) <= .03 and abs(z) <= .3 for _, x, z in world.commands)
    if args.scenario == 'clear':
        assert .039 <= math.hypot(world.x, world.y) <= .055, (world.x, world.y)
        assert .35 <= world.first_drive_x_fraction <= .65, world.first_drive_x_fraction
        assert world.route_calls == 2, world.route_calls
        assert [s['kind'] for s in result['steps']] == ['camera_head_calibration', 'scan', 'target_approach', 'scan']
        assert result['steps'][1]['report']['target_centered'] is True
        assert result['steps'][3]['report']['target_centered'] is True
    else:
        assert not any(x for _, x, _ in world.commands), world.commands
        if args.scenario == 'centering_loss':
            assert 'Target lost while centering' in result.get('error', ''), result
        if args.scenario == 'blocked':
            assert 'reports blocked' in result.get('error', ''), result
    result.update(simulation=True, scenario=args.scenario, route_calls=world.route_calls,
                  displacement_m=math.hypot(world.x, world.y),
                  target_x_fraction_before_drive=world.first_drive_x_fraction)
    report_path.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({key: result.get(key) for key in ('scenario', 'outcome', 'error', 'displacement_m', 'route_calls')}))
finally:
    observer.send_signal(signal.SIGINT)
    observer.wait(timeout=5)
    server.shutdown()
    executor.shutdown()
    ros_thread.join(timeout=2)
    world.destroy_node()
    rclpy.shutdown()
