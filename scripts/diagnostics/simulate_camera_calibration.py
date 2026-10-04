#!/usr/bin/env python3
"""Exercise the production ROS calibrator with simulated feedback, no hardware/cloud.
Run on the Jetson with ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1, after sourcing ROS.
This tests message timing and orchestration, not camera mechanics or recognition.
"""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import sys
import threading
import time
from unittest.mock import patch


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--calibrator', required=True)
    parser.add_argument('--scenario', choices=('normal', 'stall', 'reference_change', 'manual'), default='normal')
    parser.add_argument('--report', required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '187' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('Run in isolated localhost ROS domain 187')
    sys.path.insert(0, str(Path(args.calibrator).resolve().parent))
    spec = importlib.util.spec_from_file_location('calibration_under_test', args.calibrator)
    calibration = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(calibration)
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import Image

    rclpy.init()

    class Simulation(Node):
        def __init__(self):
            super().__init__('camera_calibration_simulator')
            self.head = dict(position=-18, target_position=-18, minimum_position=-44,
                            maximum_position=0, minimum_target_position=-41,
                            maximum_target_position=-3, forward_position=-18,
                            down_position=-3, up_position=-41, moving=False, homing=False,
                            homed=True, reference_id='simulated-reference',
                            approved_reference_id='simulated-reference',
                            require_approved_reference=True, calibrated=False,
                            manual_override=False, speed=300, stall_retry_limit=1,
                            stall_retry_count=0)
            if args.scenario == 'manual':
                self.head['saved_limits'] = dict(version=1, reference_id='simulated-reference', lower=0, upper=-44)
            self.commands = []
            self.cloud_calls = 0
            self.pending = None
            self.status_pub = self.create_publisher(String, '/robot_status', 10)
            self.head_pub = self.create_publisher(String, '/camera_head/status', 10)
            self.camera_pub = self.create_publisher(String, '/camera/status', 10)
            self.image_pub = self.create_publisher(Image, '/camera/image_raw', 1)
            self.create_subscription(String, '/camera_head/command', self.command, 10)
            self.create_subscription(Twist, '/cmd_vel', self.velocity, 10)
            self.create_timer(.05, self.tick)
            self.drive_commands = 0

        def velocity(self, message):
            if any((message.linear.x, message.linear.y, message.angular.z)):
                self.drive_commands += 1

        def command(self, message):
            try:
                command = json.loads(message.data)
            except ValueError:
                command = {'action': message.data}
            self.commands.append(command)
            action = command['action']
            if action == 'use_saved_limits':
                self.head.update(calibrated=True, calibration_source='operator_limits')
            elif action == 'invalidate_calibration':
                self.head['calibrated'] = False
            elif action == 'stop':
                self.pending = None
                self.head['moving'] = False
            elif action == 'jog_to':
                # Deliver old stopped feedback before acknowledging the new target.
                self.pending = (time.monotonic(), command['target'])
            elif action == 'set_runtime_positions':
                assert command['reference_id'] == self.head['reference_id']
                for key in ('forward', 'down', 'up'):
                    self.head[key + '_position'] = command[key]
                self.head['calibrated'] = True
            else:
                raise RuntimeError('Unexpected command: ' + str(command))

        def tick(self):
            if self.pending:
                started, target = self.pending
                elapsed = time.monotonic() - started
                if elapsed > .25:
                    self.head.update(target_position=target, moving=True)
                if elapsed > .5:
                    if args.scenario == 'stall':
                        self.head.update(moving=False, stall_retry_count=1)
                    else:
                        self.head.update(position=target + 1, moving=False)
                    if args.scenario == 'reference_change':
                        self.head['reference_id'] = 'different-reference'
                    self.pending = None
            for publisher, value in (
                (self.status_pub, dict(track_motion_active=False, motion_active=False)),
                (self.head_pub, self.head),
                (self.camera_pub, dict(state='streaming'))):
                publisher.publish(String(data=json.dumps(value)))
            frame = Image()
            frame.height, frame.width, frame.step = 72, 96, 288
            frame.encoding = 'bgr8'
            frame.data = bytes((30 if (x//6 + y//6) % 2 else 210)
                               for y in range(72) for x in range(96) for channel in range(3))
            self.image_pub.publish(frame)

        def scene(self, *unused):
            head = dict(self.head, available=True)
            # Deliberately ambiguous local semantics; only the mocked cloud labels settle roles.
            return dict(ok=True, result_age_seconds=0, camera_head=head,
                        scene=dict(floor_fraction=.1, ceiling_fraction=0, known_fraction=.8),
                        frame_sha256=str(head['position']))

        def cloud(self, url, samples, reference):
            self.cloud_calls += 1
            return dict(ok=True, provider='groq', reference_id=reference, advice_id='sim-advice',
                        frame_sha256=[s['frame_sha256'] for s in samples],
                        positions=[s['position'] for s in samples], upper_verified=True,
                        observations=[dict(view='floor_room', floor_visible='yes', ceiling_visible='no'),
                                      dict(view='room', floor_visible='no', ceiling_visible='no'),
                                      dict(view='ceiling', floor_visible='no', ceiling_visible='yes')])

    node = Simulation()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        options = calibration.parse_args(['--execute'])
        with patch.object(rclpy, 'init'), patch.object(rclpy, 'shutdown'), \
             patch.object(calibration, 'fetch_route', side_effect=node.scene), \
             patch.object(calibration, 'fetch_cloud_advice', side_effect=node.cloud):
            result = calibration.run(options)
        result.update(simulation=True, scenario=args.scenario,
                      issued_commands=node.commands, cloud_calls=node.cloud_calls,
                      nonzero_chassis_commands=node.drive_commands)
        expected = 'calibrated' if args.scenario in ('normal', 'manual') else 'failure'
        assert result['outcome'] == expected, result.get('error')
        assert node.drive_commands == 0
        assert all(-41 <= c['target'] <= -3 for c in node.commands if c['action'] == 'jog_to')
        if args.scenario == 'manual':
            assert result['outcome'] == 'calibrated'
            assert node.cloud_calls == 0
            assert node.commands == [{'action': 'use_saved_limits'}], node.commands
        elif args.scenario == 'normal':
            assert node.cloud_calls == 1
            assert len(result['samples']) == 3
            assert abs(result['final_position'] - result['calibration']['forward_position']) <= 3
        else:
            assert node.cloud_calls == 0
            assert not node.head['calibrated']
        Path(args.report).write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(dict(scenario=args.scenario, outcome=result['outcome'],
                             error=result.get('error'), cloud_calls=node.cloud_calls,
                             elapsed=round(result['finished_at_unix']-result['started_at_unix'], 2))))
    finally:
        executor.shutdown()
        thread.join(timeout=2)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
