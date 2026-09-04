#!/usr/bin/env python3
"""Execute one stop-look-turn-look-move camera-only detour primitive."""

import argparse
import json
import math
import time
from urllib.request import Request, urlopen


class DetourError(RuntimeError):
    pass


def center_floor_fraction(route_result):
    for item in route_result.get("evidence", []):
        if abs(float(item.get("heading_degrees", 999))) < 0.1:
            return float(item.get("floor_fraction", 0.0))
    return 0.0


def validate_route_result(result, maximum_age_seconds=1.0):
    if not isinstance(result, dict) or not result.get("ok"):
        raise DetourError("route perception did not return an okay result")
    age = float(result.get("result_age_seconds", 999.0))
    if not math.isfinite(age) or age > maximum_age_seconds:
        raise DetourError("route result is stale")
    head = result.get("camera_head")
    if (
        not isinstance(head, dict)
        or not head.get("available", False)
        or not head.get("homed", False)
        or not head.get("calibrated", False)
        or head.get("moving", True)
        or head.get("homing", True)
    ):
        raise DetourError("camera head is not in a stable known state")
    try:
        if abs(int(head["position"]) - int(head["down_position"])) > 5:
            raise DetourError("camera head is outside the route-view range")
    except (KeyError, TypeError, ValueError):
        raise DetourError("camera head route-view position is unavailable")
    decision = result.get("decision")
    if not isinstance(decision, dict) or decision.get("blocked", True):
        raise DetourError("route perception reports blocked")
    heading = float(decision.get("heading_degrees", 0.0))
    distance = float(decision.get("distance_m", 0.0))
    if not math.isfinite(heading) or abs(heading) > 45.0:
        raise DetourError("route heading exceeds the development limit")
    if not math.isfinite(distance) or not 0.05 <= distance <= 0.10:
        raise DetourError("route distance exceeds the development limit")
    return heading, distance


def fetch_route(url, timeout_seconds=15.0):
    request = Request(url, headers={"Cache-Control": "no-cache"})
    with urlopen(request, timeout=timeout_seconds) as response:
        return json.loads(response.read().decode("utf-8"))


def angle_delta(current, start):
    return math.atan2(math.sin(current - start), math.cos(current - start))


def run(args):
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from std_msgs.msg import String

    class DetourNode(Node):
        def __init__(self):
            super().__init__("echora_closed_loop_detour")
            self.robot_status = None
            self.robot_status_at = None
            self.head_status = None
            self.head_status_at = None
            self.yaw = None
            self.x = None
            self.y = None
            self.cmd = self.create_publisher(Twist, "/cmd_vel", 1)
            self.head = self.create_publisher(String, "/camera_head/command", 1)
            self.create_subscription(String, "/robot_status", self.on_robot, 10)
            self.create_subscription(String, "/camera_head/status", self.on_head, 10)
            self.create_subscription(Odometry, "/odom", self.on_odom, 10)

        def on_robot(self, message):
            try:
                self.robot_status = json.loads(message.data)
                self.robot_status_at = time.monotonic()
            except ValueError:
                pass

        def on_head(self, message):
            try:
                self.head_status = json.loads(message.data)
                self.head_status_at = time.monotonic()
            except ValueError:
                pass

        def on_odom(self, message):
            q = message.pose.pose.orientation
            self.yaw = math.atan2(
                2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z
            )
            self.x = float(message.pose.pose.position.x)
            self.y = float(message.pose.pose.position.y)

        def spin_until(self, predicate, timeout, message):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                if predicate():
                    return
            raise DetourError(message)

        def publish_velocity(self, linear=0.0, angular=0.0):
            value = Twist()
            value.linear.x = float(linear)
            value.angular.z = float(angular)
            self.cmd.publish(value)

        def stop(self):
            previous_status_at = self.robot_status_at or 0.0
            for _ in range(5):
                self.publish_velocity()
                rclpy.spin_once(self, timeout_sec=0.05)
            self.spin_until(
                lambda: self.robot_status_at is not None
                and self.robot_status_at > previous_status_at
                and not self.robot_status.get("motion_active", True),
                2.0,
                "stopped status was not confirmed",
            )

        def prepare(self):
            self.spin_until(
                lambda: self.robot_status_at is not None
                and self.head_status_at is not None
                and self.yaw is not None,
                5.0,
                "robot state is unavailable",
            )
            now = time.monotonic()
            if now - self.robot_status_at > 1.0 or now - self.head_status_at > 1.0:
                raise DetourError("robot state is stale")
            self.stop()
            if (
                self.head_status.get("homed", False)
                and not self.head_status.get("moving", True)
                and not self.head_status.get("homing", True)
                and abs(
                    int(self.head_status.get("position", -999))
                    - int(self.head_status.get("down_position", 999))
                )
                <= 5
            ):
                time.sleep(0.3)
                return
            previous_head_at = self.head_status_at or 0.0
            request = String()
            request.data = "look_down"
            self.head.publish(request)
            self.spin_until(
                lambda: self.head_status is not None
                and self.head_status_at > previous_head_at
                and self.head_status.get("homed", False)
                and not self.head_status.get("moving", True)
                and abs(
                    int(self.head_status.get("position", -999))
                    - int(self.head_status.get("down_position", 999))
                )
                <= 5,
                4.0,
                "camera head did not settle at the down position",
            )
            time.sleep(0.7)

        def turn_image_heading(self, heading_degrees):
            # Image-space corridor convention is negative-left/positive-right;
            # ROS yaw is positive-left, hence the sign inversion.
            target = math.radians(-heading_degrees)
            if abs(target) < math.radians(2.0):
                return 0.0
            start = self.yaw
            direction = 1.0 if target > 0 else -1.0
            deadline = time.monotonic() + args.turn_timeout
            try:
                while time.monotonic() < deadline:
                    rclpy.spin_once(self, timeout_sec=0.04)
                    moved = angle_delta(self.yaw, start)
                    remaining = abs(target) - abs(moved)
                    if remaining <= math.radians(args.yaw_tolerance_degrees):
                        return math.degrees(moved)
                    speed = max(0.16, min(args.turn_speed, remaining * 1.4))
                    self.publish_velocity(angular=direction * speed)
                raise DetourError("turn timed out")
            finally:
                self.stop()

        def drive_distance(self, distance):
            start_x, start_y = self.x, self.y
            deadline = time.monotonic() + args.drive_timeout
            try:
                while time.monotonic() < deadline:
                    rclpy.spin_once(self, timeout_sec=0.04)
                    travelled = math.hypot(self.x - start_x, self.y - start_y)
                    if travelled >= max(0.0, distance - args.distance_tolerance):
                        return travelled
                    self.publish_velocity(linear=args.drive_speed)
                raise DetourError("drive timed out")
            finally:
                self.stop()

    report = {
        "outcome": "failure",
        "started_at_unix": time.time(),
        "route_url": args.route_url,
        "events": [],
    }
    rclpy.init()
    node = DetourNode()
    try:
        node.prepare()
        first = fetch_route(args.route_url)
        heading, distance = validate_route_result(first, args.maximum_result_age)
        report["initial_route"] = first
        report["events"].append("initial_route_approved")
        if not args.execute:
            report["outcome"] = "dry_run_success"
            return report

        turned = node.turn_image_heading(heading)
        report["turned_degrees"] = turned
        report["events"].append("turn_complete_and_stopped")
        time.sleep(0.8)

        second = None
        for _ in range(4):
            candidate = fetch_route(args.route_url)
            if candidate.get("frame_sha256") != first.get("frame_sha256"):
                second = candidate
                break
            time.sleep(0.3)
        if second is None:
            raise DetourError("no new camera frame arrived after the turn")
        validate_route_result(second, args.maximum_result_age)
        report["post_turn_route"] = second
        center_fraction = center_floor_fraction(second)
        report["post_turn_center_floor_fraction"] = center_fraction
        if center_fraction < args.minimum_center_floor:
            raise DetourError("new straight corridor is not proven clear")
        report["events"].append("new_straight_view_approved")

        travelled = node.drive_distance(min(distance, 0.10))
        report["travelled_m"] = travelled
        report["events"].append("short_drive_complete_and_stopped")
        report["outcome"] = "success"
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
    parser.add_argument("--route-url", required=True)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--turn-speed", type=float, default=0.30)
    parser.add_argument("--drive-speed", type=float, default=0.05)
    parser.add_argument("--turn-timeout", type=float, default=8.0)
    parser.add_argument("--drive-timeout", type=float, default=5.0)
    parser.add_argument("--yaw-tolerance-degrees", type=float, default=3.0)
    parser.add_argument("--distance-tolerance", type=float, default=0.01)
    parser.add_argument("--minimum-center-floor", type=float, default=0.95)
    parser.add_argument("--maximum-result-age", type=float, default=1.0)
    parser.add_argument("--report", default="/home/animesh/echora/logs/latest_detour.json")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    report = run(args)
    print(json.dumps(report, indent=2, sort_keys=True))
    try:
        import os
        os.makedirs(os.path.dirname(args.report), exist_ok=True)
        with open(args.report, "w", encoding="utf-8") as handle:
            json.dump(report, handle, indent=2, sort_keys=True)
            handle.write("\n")
    except OSError as exc:
        print("warning: could not write report: {0}".format(exc))
    return 0 if report["outcome"] in ("success", "dry_run_success") else 2


if __name__ == "__main__":
    raise SystemExit(main())
