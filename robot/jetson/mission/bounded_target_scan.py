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
    if args.first_direction == "left":
        headings = tuple(-heading for heading in headings)
    report = {
        "outcome": "failure",
        "started_at_unix": time.time(),
        "requested_headings": headings,
        "observed_headings": [args.initial_cable_heading_degrees],
        "turns": [],
    }
    if not args.execute:
        report["outcome"] = "dry_run_success"
        report["incremental_turns"] = (
            (-args.initial_cable_heading_degrees,)
            + incremental_scan_turns(headings)
            if args.initial_cable_heading_degrees
            else incremental_scan_turns(headings)
        )
        report["finished_at_unix"] = time.time()
        return report

    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
    from sensor_msgs.msg import Image
    from std_msgs.msg import String

    class ScanNode(Node):
        def __init__(self):
            super().__init__("echora_bounded_target_scan")
            self.robot = None
            self.robot_at = None
            self.camera = None
            self.camera_at = None
            self.camera_frame_at = None
            self.head_status = None
            self.head_at = None
            self.target = None
            self.target_at = None
            self.yaw = None
            self.cmd = self.create_publisher(Twist, "/cmd_vel", 1)
            self.head = self.create_publisher(String, "/camera_head/command", 1)
            self.create_subscription(String, "/robot_status", self.on_robot, 10)
            self.create_subscription(String, "/camera/status", self.on_camera, 10)
            sensor_qos = QoSProfile(
                depth=1,
                history=QoSHistoryPolicy.KEEP_LAST,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
            )
            self.create_subscription(
                Image, "/camera/image_raw", self.on_camera_frame, sensor_qos
            )
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

        def on_camera_frame(self, _message):
            self.camera_frame_at = time.monotonic()

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
            return self.safety_reason() is None

        def safety_reason(self):
            now = time.monotonic()
            motion_reason = self.motion_reason()
            if motion_reason:
                return motion_reason
            if self.camera_at is None or now - self.camera_at > 6.5:
                return "camera health status is stale"
            if self.camera.get("state") != "streaming":
                return "camera is not streaming"
            if self.camera_frame_at is None or now - self.camera_frame_at > 0.5:
                return "live camera frame heartbeat is stale"
            if float(self.camera.get("mean_intensity", 0.0)) < 15.0:
                return "camera image is too dark"
            return None

        def motion_ready(self):
            return self.motion_reason() is None

        def motion_reason(self):
            now = time.monotonic()
            if self.robot_at is None or now - self.robot_at > 1.0:
                return "robot status is stale"
            if self.robot.get("motion_active", True):
                return "robot unexpectedly reports motion"
            if self.head_at is None or now - self.head_at > 1.0:
                return "camera-head status is stale"
            if not self.head_status.get("homed", False):
                return "camera head is not homed"
            if self.head_status.get("moving", True) or self.head_status.get(
                "homing", True
            ):
                return "camera head is moving"
            if self.yaw is None:
                return "odometry is unavailable"
            return None

        def look_forward(self):
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
            self.wait_for_camera_after_head_move()

        def wait_for_camera_after_head_move(self, timeout=8.0):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                motion_problem = self.motion_reason()
                if motion_problem:
                    raise ScanError(motion_problem)
                now = time.monotonic()
                if (
                    self.camera_at is not None
                    and now - self.camera_at <= 6.5
                    and self.camera.get("state") == "streaming"
                    and self.camera_frame_at is not None
                    and now - self.camera_frame_at <= 0.5
                ):
                    return
            raise ScanError("camera did not recover after camera-head movement")

        def prepare(self):
            self.spin_until(self.safety_ready, 10.0, "scan sensors are unavailable")
            self.stop()
            self.look_forward()

        def turn_relative(self, image_degrees, camera_required=True):
            ready = self.safety_ready if camera_required else self.motion_ready
            if not ready():
                reason = self.safety_reason() if camera_required else self.motion_reason()
                raise ScanError(reason or "sensor state became unsafe before turn")
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

        def wait_for_target(self, duration=None):
            dwell = args.dwell_seconds if duration is None else float(duration)
            deadline = time.monotonic() + dwell
            recovered_once = False
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                if not self.safety_ready():
                    motion_problem = self.motion_reason()
                    if motion_problem:
                        raise ScanError(motion_problem)
                    if recovered_once:
                        raise ScanError(
                            self.safety_reason()
                            or "camera became unsafe again while observing"
                        )
                    self.wait_for_camera_after_head_move()
                    recovered_once = True
                    deadline = time.monotonic() + dwell
                if target_is_confirmed(self.target):
                    return True
            return False

        def look_fully_up(self):
            while True:
                position = int(self.head_status.get("position", -999))
                minimum = int(self.head_status.get("minimum_position", -999))
                if position <= minimum + 2:
                    break
                step = max(-15, minimum - position)
                previous = self.head_at or 0.0
                message = String()
                message.data = json.dumps(
                    {"action": "jog", "degrees": step}, separators=(",", ":")
                )
                self.head.publish(message)
                self.spin_until(
                    lambda: self.head_at is not None
                    and self.head_at > previous
                    and not self.head_status.get("moving", True)
                    and not self.head_status.get("homing", True),
                    4.0,
                    "camera head did not settle during upward reacquisition",
                )
            self.wait_for_camera_after_head_move()

    rclpy.init()
    node = ScanNode()
    try:
        if args.unwind_only:
            if not args.initial_cable_heading_degrees:
                raise ScanError("unwind-only requires the current cable heading")
            node.spin_until(
                node.motion_ready, 5.0, "motion state is unavailable for unwind"
            )
            node.stop()
            actual = node.turn_relative(
                -args.initial_cable_heading_degrees, camera_required=False
            )
            report["turns"].append(
                {
                    "requested_degrees": -args.initial_cable_heading_degrees,
                    "actual_degrees": actual,
                    "reason": "camera-independent_cable_unwind",
                }
            )
            report["observed_headings"].append(0)
            report["outcome"] = "cable_unwound"
            return report
        node.prepare()
        if args.initial_cable_heading_degrees:
            actual = node.turn_relative(-args.initial_cable_heading_degrees)
            report["turns"].append(
                {
                    "requested_degrees": -args.initial_cable_heading_degrees,
                    "actual_degrees": actual,
                    "reason": "initial_unwind",
                }
            )
            report["observed_headings"].append(0)
        if node.wait_for_target():
            report["target_heading_degrees"] = 0
            report["target_observation"] = node.target
            report["outcome"] = "target_found"
            return report
        if args.try_up:
            node.look_fully_up()
            time.sleep(0.5)
            if node.wait_for_target(args.up_dwell_seconds):
                report["target_heading_degrees"] = 0
                report["target_observation"] = node.target
                report["outcome"] = "target_found"
                return report
            node.look_forward()
        for target_heading, turn in zip(headings[1:], incremental_scan_turns(headings)):
            actual = node.turn_relative(turn)
            report["turns"].append(
                {"requested_degrees": turn, "actual_degrees": actual}
            )
            report["observed_headings"].append(target_heading)
            if node.wait_for_target():
                report["target_heading_degrees"] = target_heading
                report["target_observation"] = node.target
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
    parser.add_argument(
        "--first-direction", choices=("right", "left"), default="right"
    )
    parser.add_argument("--initial-cable-heading-degrees", type=int, default=0)
    parser.add_argument("--unwind-only", action="store_true")
    parser.add_argument(
        "--try-up",
        action="store_true",
        help="try the camera's upper limit before a close-range chassis scan",
    )
    parser.add_argument("--dwell-seconds", type=float, default=1.0)
    parser.add_argument("--up-dwell-seconds", type=float, default=3.0)
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
