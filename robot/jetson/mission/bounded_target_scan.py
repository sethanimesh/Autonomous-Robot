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


def largest_body_observation(boxes, image_width, image_height):
    """Normalize the largest valid person box for vertical camera guidance."""
    if image_width <= 0 or image_height <= 0:
        return None
    valid = []
    for center_x, center_y, width, height in boxes:
        values = tuple(float(value) for value in (center_x, center_y, width, height))
        if not all(math.isfinite(value) for value in values):
            continue
        if width <= 0 or height <= 0:
            continue
        valid.append(values)
    if not valid:
        return None
    center_x, center_y, width, height = max(
        valid, key=lambda value: value[2] * value[3]
    )
    return {
        "center_x_fraction": center_x / float(image_width),
        "center_y_fraction": center_y / float(image_height),
        "height_fraction": height / float(image_height),
        "top_fraction": max(0.0, center_y - height / 2.0) / float(image_height),
        "bottom_fraction": min(float(image_height), center_y + height / 2.0)
        / float(image_height),
    }


def chassis_motion_active(status):
    """Separate track motion from the EV3 server's all-motor activity flag."""
    if not isinstance(status, dict):
        return True
    if "track_motion_active" in status:
        return bool(status["track_motion_active"])
    return bool(status.get("motion_active", True)) and not bool(
        status.get("tool_motion_active", False)
    )


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
    from vision_msgs.msg import Detection2DArray

    class ScanNode(Node):
        def __init__(self):
            super().__init__("echora_bounded_target_scan")
            self.robot = None
            self.robot_at = None
            self.camera = None
            self.camera_at = None
            self.camera_frame_at = None
            self.image_width = 640
            self.image_height = 480
            self.body = None
            self.body_at = None
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
            self.create_subscription(
                Detection2DArray,
                "/perception/person_detections",
                self.on_people,
                10,
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

        def on_camera_frame(self, message):
            self.camera_frame_at = time.monotonic()
            self.image_width = int(message.width)
            self.image_height = int(message.height)

        def on_people(self, message):
            boxes = [
                (
                    detection.bbox.center.position.x,
                    detection.bbox.center.position.y,
                    detection.bbox.size_x,
                    detection.bbox.size_y,
                )
                for detection in message.detections
            ]
            self.body = largest_body_observation(
                boxes, self.image_width, self.image_height
            )
            self.body_at = time.monotonic()

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
            if chassis_motion_active(self.robot):
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
            if (
                self.head_status.get("homed", False)
                and not self.head_status.get("moving", True)
                and not self.head_status.get("homing", True)
                and abs(
                    int(self.head_status.get("position", -999))
                    - int(self.head_status.get("forward_position", 999))
                )
                <= 5
            ):
                self.wait_for_camera_after_head_move()
                return
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

        def body_is_visible(self):
            return (
                self.body is not None
                and self.body_at is not None
                and time.monotonic() - self.body_at <= 1.0
            )

        def seek_target_upward(self, requested_limit=None):
            """Stop the chassis and inspect progressively higher camera views."""
            self.stop()
            while True:
                if target_is_confirmed(self.target):
                    return True
                position = int(self.head_status.get("position", -999))
                minimum = int(self.head_status.get("minimum_position", -999))
                limit = minimum
                if requested_limit is not None:
                    limit = max(minimum, int(requested_limit))
                if position <= limit + 2:
                    return self.wait_for_target(args.up_dwell_seconds)
                step = max(-abs(args.tilt_step_degrees), limit - position)
                target_position = position + step
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
                    and not self.head_status.get("homing", True)
                    and abs(
                        int(self.head_status.get("position", -999))
                        - target_position
                    )
                    <= 2,
                    8.0,
                    "camera head did not settle during upward reacquisition",
                )
                self.wait_for_camera_after_head_move()
                report.setdefault("camera_tilt_positions", []).append(
                    int(self.head_status.get("position", target_position))
                )
                if self.wait_for_target(args.up_dwell_seconds):
                    return True

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
        def target_seen_at(heading):
            report["target_heading_degrees"] = heading
            report["target_observation"] = node.target
            report["outcome"] = "target_found"
            return True

        def observe_heading(heading):
            if node.wait_for_target():
                return target_seen_at(heading)
            if args.try_up and node.body_is_visible():
                report.setdefault("body_guided_tilts", []).append(
                    {"heading_degrees": heading, "body": dict(node.body)}
                )
                if node.seek_target_upward(args.face_search_position):
                    return target_seen_at(heading)
                if not args.vertical_only:
                    node.look_forward()
            elif args.search_up:
                report.setdefault("high_view_checks", []).append(heading)
                if node.seek_target_upward(args.high_search_position):
                    return target_seen_at(heading)
                if node.body_is_visible():
                    report.setdefault("body_guided_tilts", []).append(
                        {"heading_degrees": heading, "body": dict(node.body)}
                    )
                    if node.seek_target_upward(args.face_search_position):
                        return target_seen_at(heading)
                if not args.vertical_only:
                    node.look_forward()
            return False

        if observe_heading(0):
            return report
        if args.vertical_only:
            report["outcome"] = "vertical_scan_complete_no_target"
            return report
        for target_heading, turn in zip(headings[1:], incremental_scan_turns(headings)):
            actual = node.turn_relative(turn)
            report["turns"].append(
                {"requested_degrees": turn, "actual_degrees": actual}
            )
            report["observed_headings"].append(target_heading)
            if observe_heading(target_heading):
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
        help="tilt upward in steps when a person body is seen without a face",
    )
    parser.add_argument(
        "--search-up",
        action="store_true",
        help="check one higher view before rotating when the low view sees nobody",
    )
    parser.add_argument("--high-search-position", type=int, default=-90)
    parser.add_argument("--dwell-seconds", type=float, default=1.0)
    parser.add_argument("--up-dwell-seconds", type=float, default=3.0)
    parser.add_argument("--tilt-step-degrees", type=int, default=15)
    parser.add_argument("--face-search-position", type=int, default=-120)
    parser.add_argument(
        "--vertical-only",
        action="store_true",
        help="search camera height at the current chassis heading without rotating",
    )
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
