#!/usr/bin/env python3
"""Perform one cable-safe in-place scan and stop on target confirmation."""

import argparse
import json
import math
import time

try:
    from robot.jetson.navigation.local_planner import cable_safe_scan_headings
except ImportError:
    from local_planner import cable_safe_scan_headings


class ScanError(RuntimeError):
    pass


def target_is_confirmed(payload, maximum_age_seconds=0.75):
    if not isinstance(payload, dict) or not payload.get("ok", False):
        return False
    try:
        age = float(payload["age_seconds"])
    except (KeyError, TypeError, ValueError):
        return False
    return (
        math.isfinite(age)
        and 0.0 <= age <= maximum_age_seconds
        and payload.get("confirmed") is True
        and float(payload.get("box_height_fraction", 0.0)) > 0.0
    )


def incremental_scan_turns(headings):
    values = list(headings)
    if not values or values[0] != 0:
        raise ValueError("scan headings must start at zero")
    return tuple(values[index] - values[index - 1] for index in range(1, len(values)))


def run(args):
    headings = cable_safe_scan_headings(args.step_degrees, args.sweep_limit_degrees)
    report = {
        "outcome": "failure",
        "started_at_unix": time.time(),
        "requested_headings": headings,
        "observed_headings": [0],
        "turns": [],
    }
    if not args.execute:
        report["outcome"] = "dry_run_success"
        report["incremental_turns"] = incremental_scan_turns(headings)
        report["finished_at_unix"] = time.time()
        return report

    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from std_msgs.msg import String

    class ScanNode(Node):
        def __init__(self):
            super().__init__("echora_bounded_target_scan")
            self.robot = None
            self.robot_at = None
            self.camera = None
            self.camera_at = None
            self.head_status = None
            self.head_at = None
            self.target = None
            self.target_at = None
            self.yaw = None
            self.cmd = self.create_publisher(Twist, "/cmd_vel", 1)
            self.head = self.create_publisher(String, "/camera_head/command", 1)
            self.create_subscription(String, "/robot_status", self.on_robot, 10)
            self.create_subscription(String, "/camera/status", self.on_camera, 10)
            self.create_subscription(String, "/camera_head/status", self.on_head, 10)
            self.create_subscription(
                String, "/mission/target_observation", self.on_target, 10
            )
            self.create_subscription(Odometry, "/odom", self.on_odom, 10)

        def parse(self, message):
            try:
                value = json.loads(message.data)
            except ValueError:
                return None
            return value if isinstance(value, dict) else None

        def on_robot(self, message):
            value = self.parse(message)
            if value is not None:
                self.robot, self.robot_at = value, time.monotonic()

        def on_camera(self, message):
            value = self.parse(message)
            if value is not None:
                self.camera, self.camera_at = value, time.monotonic()

        def on_head(self, message):
            value = self.parse(message)
            if value is not None:
                self.head_status, self.head_at = value, time.monotonic()

        def on_target(self, message):
            value = self.parse(message)
            if value is not None:
                self.target, self.target_at = value, time.monotonic()

        def on_odom(self, message):
            q = message.pose.pose.orientation
            self.yaw = math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)

        def spin_until(self, predicate, timeout, reason):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                if predicate():
                    return
            raise ScanError(reason)

        def velocity(self, angular=0.0):
            message = Twist()
            message.angular.z = float(angular)
            self.cmd.publish(message)

        def stop(self):
            previous = self.robot_at or 0.0
            for _ in range(5):
                self.velocity()
                rclpy.spin_once(self, timeout_sec=0.05)
            self.spin_until(
                lambda: self.robot_at is not None
                and self.robot_at > previous
                and not self.robot.get("motion_active", True),
                2.0,
                "stopped status was not confirmed",
            )

        def safety_ready(self):
            now = time.monotonic()
            return (
                self.robot_at is not None
                and now - self.robot_at <= 1.0
                and not self.robot.get("motion_active", True)
                and self.camera_at is not None
                and now - self.camera_at <= 2.0
                and self.camera.get("state") == "streaming"
                and float(self.camera.get("seconds_since_frame", 999.0)) <= 0.5
                and float(self.camera.get("mean_intensity", 0.0)) >= 15.0
                and self.head_at is not None
                and now - self.head_at <= 1.0
                and self.head_status.get("homed", False)
                and not self.head_status.get("moving", True)
                and not self.head_status.get("homing", True)
                and self.yaw is not None
            )

        def prepare(self):
            self.spin_until(self.safety_ready, 5.0, "scan sensors are unavailable")
            self.stop()
            previous = self.head_at or 0.0
            message = String()
            message.data = "look_forward"
            self.head.publish(message)
            self.spin_until(
                lambda: self.head_at is not None
                and self.head_at > previous
                and self.head_status.get("homed", False)
                and not self.head_status.get("moving", True)
                and abs(
                    int(self.head_status.get("position", -999))
                    - int(self.head_status.get("forward_position", 999))
                )
                <= 5,
                4.0,
                "camera head did not settle at the forward position",
            )

        def turn_relative(self, image_degrees):
            if not self.safety_ready():
                raise ScanError("sensor state became unsafe before turn")
            # Scan headings use image convention: positive is right.
            target = math.radians(-float(image_degrees))
            start = self.yaw
            direction = 1.0 if target > 0 else -1.0
            deadline = time.monotonic() + args.turn_timeout
            try:
                while time.monotonic() < deadline:
                    rclpy.spin_once(self, timeout_sec=0.04)
                    moved = math.atan2(
                        math.sin(self.yaw - start), math.cos(self.yaw - start)
                    )
                    remaining = abs(target) - abs(moved)
                    if remaining <= math.radians(args.yaw_tolerance_degrees):
                        return math.degrees(moved)
                    speed = max(0.16, min(args.turn_speed, remaining * 1.4))
                    self.velocity(direction * speed)
                raise ScanError("scan turn timed out")
            finally:
                self.stop()

        def wait_for_target(self):
            deadline = time.monotonic() + args.dwell_seconds
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                if not self.safety_ready():
                    raise ScanError("sensor state became unsafe while observing")
                if target_is_confirmed(self.target):
                    return True
            return False

    rclpy.init()
    node = ScanNode()
    try:
        node.prepare()
        if node.wait_for_target():
            report["outcome"] = "target_found"
            return report
        for target_heading, turn in zip(headings[1:], incremental_scan_turns(headings)):
            actual = node.turn_relative(turn)
            report["turns"].append(
                {"requested_degrees": turn, "actual_degrees": actual}
            )
            report["observed_headings"].append(target_heading)
            if node.wait_for_target():
                report["target_heading_degrees"] = target_heading
                report["outcome"] = "target_found"
                return report
        report["outcome"] = "scan_complete_no_target"
    except Exception as exc:
        report["error"] = str(exc)
        try:
            node.stop()
        except Exception as stop_exc:
            report["stop_error"] = str(stop_exc)
    finally:
        report["finished_at_unix"] = time.time()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return report


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--step-degrees", type=int, default=30)
    parser.add_argument("--sweep-limit-degrees", type=int, default=180)
    parser.add_argument("--dwell-seconds", type=float, default=1.0)
    parser.add_argument("--turn-speed", type=float, default=0.30)
    parser.add_argument("--turn-timeout", type=float, default=8.0)
    parser.add_argument("--yaw-tolerance-degrees", type=float, default=3.0)
    parser.add_argument("--report", default="/home/animesh/echora/logs/latest_scan.json")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    report = run(args)
    print(json.dumps(report, indent=2, sort_keys=True))
    if args.execute:
        import os
        os.makedirs(os.path.dirname(args.report), exist_ok=True)
        with open(args.report, "w", encoding="utf-8") as handle:
            json.dump(report, handle, indent=2, sort_keys=True)
            handle.write("\n")
    return 0 if report["outcome"] != "failure" else 2


if __name__ == "__main__":
    raise SystemExit(main())
