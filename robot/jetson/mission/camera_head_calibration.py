#!/usr/bin/env python3
"""Calibrate the camera head from a fresh mechanical zero and live vision."""

import argparse
import json
import math
import os
import time
from urllib.request import Request, urlopen


class CameraCalibrationError(RuntimeError):
    pass


def next_upward_position(current, upper_limit=-150, step_degrees=15):
    current = int(current)
    upper_limit = int(upper_limit)
    step_degrees = abs(int(step_degrees))
    if step_degrees <= 0:
        raise ValueError("camera calibration step must be positive")
    if current <= upper_limit:
        return None
    return max(upper_limit, current - step_degrees)


def track_motion_active(status):
    if not isinstance(status, dict):
        return True
    if "track_motion_active" in status:
        return bool(status["track_motion_active"])
    return bool(status.get("motion_active", True)) and not bool(
        status.get("tool_motion_active", False)
    )


def center_floor_fraction(route):
    if not isinstance(route, dict):
        return None
    values = []
    for item in route.get("evidence", []):
        try:
            heading = float(item["heading_degrees"])
            fraction = float(item["floor_fraction"])
        except (KeyError, TypeError, ValueError):
            continue
        if math.isfinite(heading) and math.isfinite(fraction) and abs(heading) <= 10:
            values.append((abs(heading), max(0.0, min(1.0, fraction))))
    return None if not values else min(values)[1]


def choose_runtime_positions(samples, fallback_forward=-30):
    """Choose positions discovered during one upward vision sweep."""

    clean = []
    for sample in samples:
        try:
            position = int(sample["position"])
        except (KeyError, TypeError, ValueError):
            continue
        clean.append((position, sample))
    if not clean:
        raise ValueError("camera calibration produced no settled positions")

    positions = [item[0] for item in clean]
    down = max(positions)
    upper = min(positions)
    face_samples = [
        item
        for item in clean
        if item[1].get("target_confirmed")
        or item[1].get("target_score") is not None
    ]
    if face_samples:
        forward = max(
            face_samples,
            key=lambda item: (
                bool(item[1].get("target_confirmed")),
                float(item[1].get("target_score") or -1.0),
            ),
        )[0]
        reason = "target_face"
    else:
        body_samples = [
            item for item in clean if item[1].get("body_height_fraction") is not None
        ]
        if body_samples:
            forward = max(
                body_samples,
                key=lambda item: float(item[1]["body_height_fraction"]),
            )[0]
            reason = "person_body"
        else:
            floor_samples = [
                item for item in clean if item[1].get("floor_fraction") is not None
            ]
            if floor_samples:
                forward = min(
                    floor_samples,
                    key=lambda item: abs(float(item[1]["floor_fraction"]) - 0.45),
                )[0]
                reason = "floor_transition"
            else:
                forward = min(positions, key=lambda value: abs(value - fallback_forward))
                reason = "bounded_fallback"

    if forward >= down and upper < down:
        forward = next(value for value in sorted(positions, reverse=True) if value < down)
    return {
        "down_position": down,
        "forward_position": forward,
        "observed_upper_position": upper,
        "selection_reason": reason,
    }


def fetch_route(url, timeout_seconds=8.0):
    if not url:
        return None
    request = Request(url, headers={"Cache-Control": "no-cache"})
    with urlopen(request, timeout=timeout_seconds) as response:
        value = json.loads(response.read().decode("utf-8"))
    if not isinstance(value, dict) or not value.get("ok", False):
        raise CameraCalibrationError("route perception did not return a valid result")
    return value


def dry_run_report(args):
    positions = [0]
    while True:
        value = next_upward_position(
            positions[-1], args.upper_limit, args.step_degrees
        )
        if value is None:
            break
        positions.append(value)
    return {
        "outcome": "dry_run_success",
        "chassis_locked": True,
        "home_position": 0,
        "planned_positions": positions,
        "verification": {"down_then_up_degrees": args.step_degrees},
    }


def run(args):
    if not args.execute:
        return dry_run_report(args)

    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from vision_msgs.msg import Detection2DArray

    class CalibrationNode(Node):
        def __init__(self):
            super().__init__("echora_camera_head_calibration")
            self.robot = None
            self.robot_at = None
            self.camera = None
            self.camera_at = None
            self.frame_at = None
            self.head = None
            self.head_at = None
            self.body = None
            self.body_at = None
            self.target_score = None
            self.target_at = None
            self.target_confirmed = False
            self.confirmed_at = None
            self.velocity_publisher = self.create_publisher(Twist, "/cmd_vel", 1)
            self.head_publisher = self.create_publisher(
                String, "/camera_head/command", 1
            )
            self.create_subscription(String, "/robot_status", self.on_robot, 10)
            self.create_subscription(String, "/camera/status", self.on_camera, 10)
            self.create_subscription(String, "/camera_head/status", self.on_head, 10)
            self.create_subscription(
                String, "/mission/target_observation", self.on_confirmation, 10
            )
            qos = QoSProfile(
                depth=1,
                history=QoSHistoryPolicy.KEEP_LAST,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
            )
            self.create_subscription(Image, "/camera/image_raw", self.on_frame, qos)
            self.create_subscription(
                Detection2DArray,
                "/perception/person_detections",
                self.on_people,
                10,
            )
            self.create_subscription(
                Detection2DArray,
                "/perception/target_matches",
                self.on_matches,
                10,
            )

        @staticmethod
        def parse(message):
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
                self.head, self.head_at = value, time.monotonic()

        def on_frame(self, _message):
            self.frame_at = time.monotonic()

        def on_people(self, message):
            heights = [float(item.bbox.size_y) / 480.0 for item in message.detections]
            self.body = max(heights) if heights else None
            self.body_at = time.monotonic()

        def on_matches(self, message):
            scores = [
                float(result.hypothesis.score)
                for item in message.detections
                for result in item.results
            ]
            self.target_score = max(scores) if scores else None
            self.target_at = time.monotonic()

        def on_confirmation(self, message):
            value = self.parse(message)
            if value is None:
                return
            self.target_confirmed = bool(value.get("confirmed", False))
            self.confirmed_at = time.monotonic()

        def spin_until(self, predicate, timeout, reason):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                if predicate():
                    return
            raise CameraCalibrationError(reason)

        def ready(self):
            now = time.monotonic()
            return (
                self.robot_at is not None
                and now - self.robot_at <= 1.0
                and not track_motion_active(self.robot)
                and self.head_at is not None
                and now - self.head_at <= 1.0
                and self.camera_at is not None
                and now - self.camera_at <= 6.5
                and self.camera.get("state") == "streaming"
                and self.frame_at is not None
                and now - self.frame_at <= 0.75
            )

        def stop_chassis(self):
            message = Twist()
            for _ in range(5):
                self.velocity_publisher.publish(message)
                rclpy.spin_once(self, timeout_sec=0.05)
            self.spin_until(
                lambda: self.robot is not None and not track_motion_active(self.robot),
                2.0,
                "chassis stop was not confirmed",
            )

        def command_head(self, value):
            message = String()
            message.data = (
                value
                if isinstance(value, str)
                else json.dumps(value, separators=(",", ":"))
            )
            self.head_publisher.publish(message)

        def wait_camera(self):
            self.spin_until(
                lambda: self.camera is not None
                and self.camera.get("state") == "streaming"
                and self.frame_at is not None
                and time.monotonic() - self.frame_at <= 0.75,
                8.0,
                "camera did not recover after head movement",
            )

        def home(self):
            previous = self.head_at or 0.0
            self.command_head("home")
            self.spin_until(
                lambda: self.head_at is not None
                and self.head_at > previous
                and self.head.get("homed", False)
                and not self.head.get("moving", True)
                and not self.head.get("homing", True)
                and abs(int(self.head.get("position", 999))) <= args.tolerance,
                args.move_timeout,
                "camera head did not establish its downward mechanical zero",
            )
            self.wait_camera()

        def move_to(self, target):
            current = int(self.head["position"])
            while abs(current - target) > args.tolerance:
                delta = target - current
                step = max(-args.step_degrees, min(args.step_degrees, delta))
                current = self.jog_once(step)
            return current

        def jog_once(self, step):
            current = int(self.head["position"])
            minimum = int(self.head.get("minimum_position", args.upper_limit))
            maximum = int(self.head.get("maximum_position", 0))
            expected = min(maximum, max(minimum, current + int(step)))
            if expected == current:
                raise CameraCalibrationError("camera head is already at its limit")
            previous = self.head_at or 0.0
            self.command_head({"action": "jog", "degrees": int(step)})
            self.spin_until(
                lambda: self.head_at is not None
                and self.head_at > previous
                and self.head.get("homed", False)
                and not self.head.get("moving", True)
                and not self.head.get("homing", True)
                and abs(int(self.head.get("position", 999)) - expected)
                <= args.tolerance,
                args.move_timeout,
                "camera head did not settle at {0}".format(expected),
            )
            self.wait_camera()
            return int(self.head["position"])

        def observe(self):
            deadline = time.monotonic() + args.dwell_seconds
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
            now = time.monotonic()
            return {
                "position": int(self.head["position"]),
                "body_height_fraction": (
                    round(self.body, 4)
                    if self.body is not None
                    and self.body_at is not None
                    and now - self.body_at <= 1.0
                    else None
                ),
                "target_score": (
                    round(self.target_score, 4)
                    if self.target_score is not None
                    and self.target_at is not None
                    and now - self.target_at <= 1.0
                    else None
                ),
                "target_confirmed": bool(
                    self.target_confirmed
                    and self.confirmed_at is not None
                    and now - self.confirmed_at <= 1.0
                ),
            }

    report = {
        "outcome": "failure",
        "started_at_unix": time.time(),
        "chassis_locked": True,
        "samples": [],
    }
    rclpy.init()
    node = CalibrationNode()
    try:
        node.spin_until(node.ready, 10.0, "camera or EV3 state is unavailable")
        node.stop_chassis()
        if not args.reuse_existing_home or not node.head.get("homed", False):
            node.home()
            report["fresh_mechanical_home"] = True
        else:
            report["fresh_mechanical_home"] = False

        while True:
            sample = node.observe()
            # One semantic check at the mechanical down reference is enough;
            # person/face topics guide the fast upward sweep without repeatedly
            # waiting for the slower remote segmentation model.
            if args.route_url and not report["samples"]:
                try:
                    route = fetch_route(args.route_url, args.route_timeout)
                    sample["floor_fraction"] = center_floor_fraction(route)
                    sample["floor_horizon_y"] = route.get("floor_horizon_y")
                except Exception as exc:
                    sample["route_error"] = str(exc)
            report["samples"].append(sample)
            if (
                len(report["samples"]) > 1
                and (
                    sample["target_confirmed"]
                    or (
                        sample["target_score"] is not None
                        and sample["target_score"] >= args.match_score
                    )
                )
            ):
                break
            target = next_upward_position(
                sample["position"], args.upper_limit, args.step_degrees
            )
            if target is None:
                break
            node.move_to(target)

        calibration = choose_runtime_positions(
            report["samples"], args.fallback_forward
        )
        report["calibration"] = calibration

        verification_start = int(node.head["position"])
        if verification_start >= 0:
            raise CameraCalibrationError("up/down verification has no travel range")
        reached_down = node.jog_once(args.step_degrees)
        reached_up = node.jog_once(-args.step_degrees)
        report["up_down_verification"] = {
            "start": verification_start,
            "down": reached_down,
            "back_up": reached_up,
        }

        node.command_head(
            {
                "action": "set_runtime_positions",
                "forward": calibration["forward_position"],
                "down": calibration["down_position"],
            }
        )
        node.spin_until(
            lambda: node.head is not None
            and int(node.head.get("forward_position", 999))
            == calibration["forward_position"]
            and int(node.head.get("down_position", 999))
            == calibration["down_position"],
            3.0,
            "bridge did not accept runtime camera positions",
        )
        node.move_to(calibration["forward_position"])
        report["final_position"] = int(node.head["position"])
        report["outcome"] = "calibrated"
    except Exception as exc:
        report["error"] = str(exc)
        try:
            node.command_head("stop")
            node.stop_chassis()
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
    parser.add_argument("--reuse-existing-home", action="store_true")
    parser.add_argument("--upper-limit", type=int, default=-150)
    parser.add_argument("--step-degrees", type=int, default=15)
    parser.add_argument("--tolerance", type=int, default=8)
    parser.add_argument("--move-timeout", type=float, default=10.0)
    parser.add_argument("--dwell-seconds", type=float, default=1.0)
    parser.add_argument("--match-score", type=float, default=0.45)
    parser.add_argument("--fallback-forward", type=int, default=-30)
    parser.add_argument(
        "--route-url", default="http://192.168.1.26:8091/route"
    )
    parser.add_argument("--route-timeout", type=float, default=8.0)
    parser.add_argument(
        "--report",
        default="/home/animesh/echora/logs/camera_head_calibration.json",
    )
    args = parser.parse_args(argv)
    if args.upper_limit >= 0:
        parser.error("upper-limit must be negative")
    if args.step_degrees <= 0 or args.step_degrees > 15:
        parser.error("step-degrees must be between 1 and 15")
    if args.tolerance < 1 or args.tolerance >= args.step_degrees:
        parser.error("tolerance must be positive and smaller than the step")
    if args.move_timeout <= 0 or args.dwell_seconds <= 0:
        parser.error("timeouts and dwell must be positive")
    return args


def main(argv=None):
    args = parse_args(argv)
    report = run(args)
    print(json.dumps(report, indent=2, sort_keys=True))
    if args.execute:
        os.makedirs(os.path.dirname(args.report), exist_ok=True)
        with open(args.report, "w", encoding="utf-8") as handle:
            json.dump(report, handle, indent=2, sort_keys=True)
            handle.write("\n")
    return 0 if report["outcome"] in ("calibrated", "dry_run_success") else 2


if __name__ == "__main__":
    raise SystemExit(main())
