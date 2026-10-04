#!/usr/bin/env python3
"""Execute one stop-look-turn-look-move camera-only detour primitive."""

import argparse
import json
import math
import time
from urllib.request import Request, urlopen
from urllib.error import HTTPError

try:
    from robot.jetson.mission.recovery_policy import same_motor_references
    from robot.jetson.mission.bounded_target_scan import chassis_motion_active
except ImportError:
    from recovery_policy import same_motor_references
    from bounded_target_scan import chassis_motion_active


try:
    from robot.jetson.navigation.cable_guard import project_cable_turn
    from robot.jetson.navigation.cable_guard import record_cable_turn
    from robot.jetson.navigation.cable_guard import validate_measured_cable_heading
    from robot.jetson.navigation.navigation_reasoning import validate_route_advice
except ImportError:
    from cable_guard import project_cable_turn
    from cable_guard import record_cable_turn
    from cable_guard import validate_measured_cable_heading
    from navigation_reasoning import validate_route_advice


try:
    from robot.jetson.mission.camera_control_lease import camera_control_lease
except ImportError:
    from camera_control_lease import camera_control_lease


class DetourError(RuntimeError):
    pass


def validate_route_reasoning(result):
    """Reject a stale/mixed deployment before any chassis primitive is issued."""
    reasoning = result.get('route_reasoning', {})
    if (reasoning.get('provider') != 'gemini' or reasoning.get('advisory_only') is not True
            or not result.get('frame_sha256')
            or reasoning.get('fresh_frame_sha256') != result['frame_sha256']
            or not reasoning.get('source_frame_sha256')
            or reasoning.get('source_frame_sha256') == result['frame_sha256']
            or reasoning.get('scene_recheck', {}).get('stable') is not True
            or not isinstance(reasoning.get('approved_headings'), list)):
        raise DetourError('bound Gemini and local route checks are unavailable')
    try:
        validate_route_advice(reasoning.get('interpretation'))
    except (ValueError, TypeError):
        raise DetourError('invalid Gemini route interpretation') from None
    headings = reasoning['approved_headings']
    if (any(type(h) not in (int, float) or h not in (-30, 0, 30) for h in headings)
            or len(set(headings)) != len(headings)):
        raise DetourError('invalid approved corridor list')
    decision = result.get('decision', {})
    if not decision.get('blocked', True) and decision.get('heading_degrees') not in reasoning['approved_headings']:
        raise DetourError('selected corridor did not pass the combined route checks')


def center_floor_fraction(route_result):
    for item in route_result.get("evidence", []):
        if abs(float(item.get("heading_degrees", 999))) < 0.1:
            return float(item.get("floor_fraction", 0.0))
    return 0.0


def validate_route_result(result, maximum_age_seconds=1.0):
    if not isinstance(result, dict) or not result.get("ok"):
        raise DetourError("route perception did not return an okay result")
    age = float(result.get("result_age_seconds", 999.0))
    if not math.isfinite(age) or not 0 <= age <= maximum_age_seconds:
        raise DetourError("route result is stale")
    head = result.get("camera_head")
    if (
        not isinstance(head, dict)
        or not head.get("available", False)
        or not head.get("homed", False)
        or not head.get("calibrated", False)
        or head.get("moving", True)
        or head.get("homing", True)
        or head.get("manual_override")
        or not head.get("reference_id")
        or head.get("reference_id") != head.get("approved_reference_id")
    ):
        raise DetourError("camera head is not in a stable known state")
    try:
        if abs(int(head["position"]) - int(head["down_position"])) > int(head.get("settle_tolerance", 3)):
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


def fetch_route(url, timeout_seconds=20.0):
    request = Request(url, headers={"Cache-Control": "no-cache"})
    previous_error = None
    for attempt in range(2):
        try:
            with urlopen(request, timeout=timeout_seconds) as response:
                result = json.loads(response.read().decode("utf-8"))
            if previous_error:
                result['service_recheck'] = previous_error
            return result
        except HTTPError as exc:
            try:
                detail = json.loads(exc.read(4096).decode('utf-8')).get('error', str(exc))
            except (ValueError, UnicodeError, AttributeError):
                detail = str(exc)
            finally:
                exc.close()
            previous_error = dict(status=exc.code, error=str(detail)[:500])
            if exc.code != 503 or attempt:
                raise DetourError('route service: {0}'.format(previous_error['error'])) from exc


def recheck_uncertain_floor(fetch_checked, maximum_age):
    """Allow one new stopped frame when floor coverage alone is uncertain."""
    first = fetch_checked()
    decision = first.get('decision', {})
    retry = False
    if (first.get('ok') and decision.get('blocked')
            and decision.get('reason') == 'no proven footprint-wide route'):
        for item in first.get('evidence', []):
            if (abs(float(item.get('heading_degrees', 999))) < .1
                    and .95 <= float(item.get('floor_fraction', 0)) <= 1
                    and .60 <= float(item.get('known_fraction', 0)) < .75):
                retry = True
    if not retry:
        return first
    second = fetch_checked()
    second['uncertainty_recheck'] = first
    if (not first.get('frame_sha256') or not second.get('frame_sha256')
            or first['frame_sha256'] == second['frame_sha256']):
        raise DetourError('floor recheck did not receive a new image')
    # The retry must independently pass the unchanged route/head/age limits.
    validate_route_result(second, maximum_age)
    return second


def angle_delta(current, start):
    return math.atan2(math.sin(current - start), math.cos(current - start))


def reusable_straight_route(result, heading, turned, maximum_age, now):
    if abs(heading) >= .1 or abs(turned) >= .1:
        return None
    captured = result.get('camera_received_at_unix')
    if not isinstance(captured, (int, float)) or not math.isfinite(captured):
        return None
    age = now - captured
    if not 0 <= age <= maximum_age:
        return None
    updated = dict(result, result_age_seconds=max(age, float(result.get('result_age_seconds', 999))))
    try:
        validate_route_result(updated, maximum_age)
    except DetourError:
        return None
    return updated


def straight_step_distance(result, maximum_age, minimum_floor, maximum_step):
    """Require known center-corridor pixels, then bound this movement segment."""
    _, distance = validate_route_result(result, maximum_age)
    if 'route_reasoning' in result:
        validate_route_reasoning(result)
        if 0 not in result['route_reasoning']['approved_headings']:
            raise DetourError('Gemini or local evidence vetoed the new straight corridor')
    for evidence in result.get('evidence', []):
        if abs(float(evidence.get('heading_degrees', 999))) < .1:
            floor = float(evidence.get('floor_fraction', 0))
            known = float(evidence.get('known_fraction', 0))
            count = int(evidence.get('sample_count', 0))
            if (math.isfinite(floor) and math.isfinite(known)
                    and minimum_floor <= floor <= 1 and .75 <= known <= 1 and count > 0):
                return min(distance, maximum_step)
    raise DetourError('new straight corridor is not proven clear')


def run(args):
    with camera_control_lease(exclusive=True):
        return _run(args)


def _run(args):
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from std_msgs.msg import String
    from sensor_msgs.msg import Image
    from rclpy.qos import qos_profile_sensor_data
    from concurrent.futures import ThreadPoolExecutor

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
            self.odom_at = None
            self.frame_at = None
            self.frame_usable = False
            self.cmd = self.create_publisher(Twist, "/cmd_vel", 1)
            self.head = self.create_publisher(String, "/camera_head/command", 1)
            self.create_subscription(String, "/robot_status", self.on_robot, 1)
            self.create_subscription(String, "/camera_head/status", self.on_head, 1)
            self.create_subscription(Odometry, "/odom", self.on_odom, 1)
            self.create_subscription(Image, "/camera/image_raw", self.on_frame, qos_profile_sensor_data)

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

        def on_frame(self, message):
            if message.width > 0 and message.height > 0 and len(message.data):
                self.frame_at = time.monotonic()
                sample = message.data[::max(1,len(message.data)//512)]
                self.frame_usable = sum(sample)/len(sample) >= 15

        def on_odom(self, message):
            q = message.pose.pose.orientation
            self.yaw = math.atan2(
                2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z
            )
            self.x = float(message.pose.pose.position.x)
            self.y = float(message.pose.pose.position.y)
            self.odom_at = time.monotonic()

        def spin_until(self, predicate, timeout, message):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                if predicate():
                    return
            raise DetourError(message() if callable(message) else message)

        def publish_velocity(self, linear=0.0, angular=0.0):
            value = Twist()
            value.linear.x = float(linear)
            value.angular.z = float(angular)
            self.cmd.publish(value)

        def stop(self):
            previous_status_at = self.robot_status_at or 0.0
            self.head.publish(String(data='stop'))
            deadline, next_zero = time.monotonic()+8., 0.
            while time.monotonic() < deadline:
                if time.monotonic() >= next_zero:
                    self.publish_velocity()
                    next_zero = time.monotonic()+.2
                rclpy.spin_once(self, timeout_sec=.05)
                if (self.robot_status_at is not None and self.robot_status_at > previous_status_at
                        and not chassis_motion_active(self.robot_status)):
                    return
            raise DetourError('stopped status was not confirmed')

        def motion_reason(self, moving=False):
            now = time.monotonic()
            for name, stamp in [('robot', self.robot_status_at), ('camera head', self.head_status_at), ('odometry', self.odom_at)]:
                if stamp is None or now - stamp > (1.5 if moving else 2.5):
                    return name + ' feedback is stale'
            if self.frame_at is None or now - self.frame_at > .75:
                return 'camera frames are stale'
            if not self.frame_usable:return 'camera view is dark'
            h = self.head_status
            if (not h.get('homed') or not h.get('calibrated') or h.get('moving')
                    or h.get('homing') or h.get('manual_override')
                    or not h.get('reference_id') or h.get('reference_id') != h.get('approved_reference_id')):
                return h.get('calibration_error') or 'camera head is not in its approved stopped state'
            if abs(int(h['position'])-int(h['down_position'])) > int(h.get('settle_tolerance',3)):
                return 'camera left its floor view'
            if not all(v is not None and math.isfinite(v) for v in (self.x,self.y,self.yaw)):
                return 'odometry is unavailable'
            return None

        def checked_route(self):
            try:
                return recheck_uncertain_floor(self.checked_route_once, args.maximum_result_age)
            except DetourError as exc:
                if str(exc) != 'route result is stale after delivery':
                    raise
                # A stopped feedback recovery can outlast the image. Refresh it
                # once, without discarding the target or measured heading.
                return self.checked_route_once()

        def wait_route_feedback(self):
            """Let a stopped observation survive brief telemetry gaps."""
            reason = self.motion_reason()
            if reason is None:
                return
            if not (reason.endswith('feedback is stale') or reason == 'camera frames are stale'):
                raise DetourError(reason)
            started = time.monotonic()
            next_zero = 0.
            while time.monotonic() - started < 8.:
                if time.monotonic() >= next_zero:
                    self.publish_velocity()
                    next_zero = time.monotonic() + .2
                rclpy.spin_once(self, timeout_sec=.05)
                reason = self.motion_reason()
                if reason is None:
                    if chassis_motion_active(self.robot_status):
                        raise DetourError('robot moved while checking the route')
                    return
                if not (reason.endswith('feedback is stale') or reason == 'camera frames are stale'):
                    raise DetourError(reason)
            raise DetourError(reason)

        def checked_route_once(self):
            # Keep ROS feedback current while the Mac segments a stopped view.
            origin = (self.x, self.y, self.yaw)
            origin_status = self.robot_status
            with ThreadPoolExecutor(max_workers=1) as pool:
                future = pool.submit(fetch_route, args.route_url)
                while not future.done():
                    rclpy.spin_once(self, timeout_sec=.05)
                    self.wait_route_feedback()
                    if chassis_motion_active(self.robot_status):
                        self.publish_velocity()
                        raise DetourError('robot moved while checking the route')
                result = future.result()
            self.wait_route_feedback()
            if (chassis_motion_active(self.robot_status)
                    or not same_motor_references(origin_status, self.robot_status)
                    or math.hypot(self.x - origin[0], self.y - origin[1]) > .005
                    or abs(angle_delta(self.yaw, origin[2])) > math.radians(3)):
                self.publish_velocity()
                raise DetourError('robot pose changed during route interpretation')
            if result.get('camera_head', {}).get('reference_id') != self.head_status.get('reference_id'):
                raise DetourError('route belongs to a different camera reference')
            validate_route_reasoning(result)
            # HTTP transit time is included; old server-reported age alone is
            # insufficient after a delayed response or tunnel reconnect.
            captured = result.get('camera_received_at_unix')
            if not isinstance(captured, (int, float)) or not math.isfinite(captured):
                raise DetourError('route capture time is unavailable')
            age = time.time() - captured
            server_age = float(result.get('result_age_seconds', 999.))
            if (not math.isfinite(server_age) or not 0 <= server_age <= args.maximum_result_age
                    or not 0 <= age <= args.maximum_result_age):
                raise DetourError('route result is stale after delivery')
            result['result_age_seconds'] = max(age, server_age)
            return result

        def prepare(self):
            self.spin_until(
                lambda: self.robot_status_at is not None
                and self.head_status_at is not None
                and self.yaw is not None
                and self.frame_at is not None,
                5.0,
                lambda: "Startup feedback missing: " + ", ".join(
                    name for name, value in (("robot", self.robot_status_at),
                        ("camera head", self.head_status_at), ("odometry", self.yaw),
                        ("camera frames", self.frame_at)) if value is None),
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
                <= int(self.head_status.get("settle_tolerance",3))
            ):
                self.spin_until(lambda: self.motion_reason() is None, 3.0,
                                lambda: self.motion_reason() or "floor view feedback is unavailable")
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
                <= int(self.head_status.get("settle_tolerance",3)),
                15.0,
                lambda: self.head_status.get('calibration_error')
                        or "camera head did not settle at the down position",
            )
            self.spin_until(lambda: self.motion_reason() is None, 3.0,
                            lambda: self.motion_reason() or "floor view feedback is unavailable")

        def turn_image_heading(self, heading_degrees):
            # Image-space corridor convention is negative-left/positive-right;
            # ROS yaw is positive-left, hence the sign inversion.
            target = math.radians(-heading_degrees)
            if abs(target) < math.radians(2.0):
                return 0.0
            start = self.yaw
            direction = 1.0 if target > 0 else -1.0
            deadline = time.monotonic() + args.turn_timeout
            progress_at = time.monotonic();best_progress = 0.0
            try:
                while time.monotonic() < deadline:
                    rclpy.spin_once(self, timeout_sec=0.04)
                    reason = self.motion_reason(moving=True)
                    if reason:raise DetourError(reason)
                    moved = angle_delta(self.yaw, start)
                    progress = direction * moved
                    if progress < -math.radians(3):raise DetourError('turn moved in the wrong direction')
                    if progress > best_progress + math.radians(1):
                        best_progress, progress_at = progress, time.monotonic()
                    if time.monotonic()-progress_at > 2:raise DetourError('turn made no progress')
                    remaining = abs(target) - progress
                    if remaining <= math.radians(args.yaw_tolerance_degrees):
                        break
                    speed = min(args.turn_speed, max(0.16, remaining * 1.4))
                    self.publish_velocity(angular=direction * speed)
                else:
                    raise DetourError("turn timed out")
            finally:
                self.stop()
            stopped_at = time.monotonic()
            self.spin_until(lambda: self.odom_at is not None and self.odom_at > stopped_at,
                            2.5, 'stopped odometry is unavailable')
            return -math.degrees(angle_delta(self.yaw, start))

        def drive_distance(self, distance):
            start_x, start_y, start_yaw = self.x, self.y, self.yaw
            best_progress = 0.0;progress_at = time.monotonic()
            deadline = time.monotonic() + args.drive_timeout
            try:
                while time.monotonic() < deadline:
                    rclpy.spin_once(self, timeout_sec=0.04)
                    reason = self.motion_reason(moving=True)
                    if reason:raise DetourError(reason)
                    dx,dy = self.x-start_x,self.y-start_y
                    travelled = dx*math.cos(start_yaw)+dy*math.sin(start_yaw)
                    if travelled < -.01:raise DetourError('drive moved backwards')
                    if abs(angle_delta(self.yaw,start_yaw)) > math.radians(8):raise DetourError('drive veered off the checked corridor')
                    if travelled > best_progress + .002:
                        best_progress,progress_at = travelled,time.monotonic()
                    if time.monotonic()-progress_at > 2:raise DetourError('drive made no progress')
                    if travelled >= max(0.0, distance - args.distance_tolerance):
                        break
                    self.publish_velocity(linear=args.drive_speed)
                else:
                    raise DetourError("drive timed out")
            finally:
                self.stop()
            stopped_at = time.monotonic()
            self.spin_until(lambda: self.odom_at is not None and self.odom_at > stopped_at,
                            2.5, 'stopped odometry is unavailable')
            return (self.x-start_x)*math.cos(start_yaw)+(self.y-start_y)*math.sin(start_yaw)

    report = {
        "outcome": "failure",
        "started_at_unix": time.time(),
        "route_url": args.route_url,
        "events": [],
        "initial_cable_heading_degrees": args.initial_cable_heading_degrees,
        "cable_limit_degrees": args.cable_limit_degrees,
    }
    cable_heading = validate_measured_cable_heading(
        args.initial_cable_heading_degrees,
        args.cable_limit_degrees,
        args.cable_margin_degrees,
    )
    rclpy.init()
    node = DetourNode()
    origin_yaw = origin_status = None
    try:
        node.prepare()
        origin_yaw, origin_status = node.yaw, node.robot_status
        first = node.checked_route()
        report["initial_route"] = first
        heading, distance = validate_route_result(first, args.maximum_result_age)
        projected_heading = project_cable_turn(
            cable_heading,
            heading,
            args.cable_limit_degrees,
            args.cable_margin_degrees,
        )
        report["projected_cable_heading_degrees"] = projected_heading
        report["events"].append("initial_route_approved")
        if not args.execute:
            report["outcome"] = "dry_run_success"
            return report

        try:
            turned = node.turn_image_heading(heading)
        except Exception:
            report["cable_heading_known"] = False
            raise
        try:
            cable_heading = record_cable_turn(
                cable_heading,
                heading,
                turned,
                args.cable_limit_degrees,
                args.maximum_turn_error_degrees,
                args.cable_margin_degrees,
            )
        except Exception:
            report['cable_heading_known'] = False
            raise
        report["turned_degrees"] = turned
        report["cable_heading_degrees"] = cable_heading
        report["events"].append("turn_complete_and_stopped")
        node.spin_until(lambda: node.motion_reason() is None, 3.0, "feedback unavailable after turn")

        # A straight command does not change the viewpoint. Reuse its still
        # fresh route instead of running identical inference a second time.
        second = reusable_straight_route(first, heading, turned, args.maximum_result_age, time.time())
        reused_straight_route = second is not None
        for _ in range(0 if second is not None else 4):
            candidate = node.checked_route()
            if candidate.get("frame_sha256") != first.get("frame_sha256"):
                second = candidate
                break
            time.sleep(0.3)
        if second is None:
            raise DetourError("no new camera frame arrived after the turn")
        report["post_turn_route"] = second
        if reused_straight_route:
            report['events'].append('fresh_straight_route_reused')
        approved_distance = straight_step_distance(second, args.maximum_result_age,
                                                   args.minimum_center_floor, args.maximum_step_distance)
        center_fraction = center_floor_fraction(second)
        report["post_turn_center_floor_fraction"] = center_fraction
        if center_fraction < args.minimum_center_floor:
            raise DetourError("new straight corridor is not proven clear")
        report["events"].append("new_straight_view_approved")

        drive_start_yaw = node.yaw
        report['drive_started'] = True
        try:
            travelled = node.drive_distance(min(distance, approved_distance))
        finally:
            if node.odom_at is not None and time.monotonic()-node.odom_at <= 1:
                cable_heading -= math.degrees(angle_delta(node.yaw, drive_start_yaw))
            else:
                report['cable_heading_known'] = False
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
        if origin_status is not None and not report.get('stop_error'):
            try:
                node.stop()
                stopped_at = time.monotonic()
                node.spin_until(lambda: node.odom_at is not None and node.odom_at > stopped_at,
                                5.0, 'stopped odometry is unavailable')
                if not same_motor_references(origin_status, node.robot_status):
                    raise DetourError('motor reference changed during route movement')
                cable_heading = validate_measured_cable_heading(args.initial_cable_heading_degrees
                    - math.degrees(angle_delta(node.yaw, origin_yaw)), args.cable_limit_degrees, args.cable_margin_degrees)
                report['cable_heading_known'] = True
                report['stopped_motors'] = node.robot_status.get('motors')
            except Exception as recovery_error:
                report['cable_heading_known'] = False
                report['heading_recovery_error'] = str(recovery_error)
        report["final_cable_heading_degrees"] = round(cable_heading, 3) if report.get("cable_heading_known", True) else None
        report["finished_at_unix"] = time.time()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return report


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--route-url", required=True)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--turn-speed", type=float, default=0.60)
    parser.add_argument("--drive-speed", type=float, default=0.06)
    parser.add_argument("--maximum-step-distance", type=float, default=0.05)
    parser.add_argument("--turn-timeout", type=float, default=8.0)
    parser.add_argument("--drive-timeout", type=float, default=5.0)
    parser.add_argument("--yaw-tolerance-degrees", type=float, default=3.0)
    parser.add_argument("--distance-tolerance", type=float, default=0.01)
    parser.add_argument("--minimum-center-floor", type=float, default=0.95)
    parser.add_argument("--maximum-result-age", type=float, default=1.0)
    parser.add_argument("--initial-cable-heading-degrees", type=float, default=0.0)
    parser.add_argument("--cable-limit-degrees", type=float, default=120.0)
    parser.add_argument("--cable-margin-degrees", type=float, default=5.0)
    parser.add_argument(
        "--cable-zero-confirmed",
        action="store_true",
        help="confirm this run belongs to a mission started at marked cable neutral",
    )
    parser.add_argument("--maximum-turn-error-degrees", type=float, default=25.0)
    parser.add_argument("--report", default="/home/animesh/echora/logs/latest_detour.json")
    args = parser.parse_args(argv)
    try:
        validate_measured_cable_heading(
            args.initial_cable_heading_degrees,
            args.cable_limit_degrees,
            args.cable_margin_degrees,
        )
    except Exception as exc:
        parser.error(str(exc))
    for name, value, maximum in [('drive speed', args.drive_speed, .06),
                                  ('turn speed', args.turn_speed, .6),
                                  ('step distance', args.maximum_step_distance, .10)]:
        if not math.isfinite(value) or not 0 < value <= maximum:
            parser.error(name + ' exceeds the development limit')
    if not 0 <= args.distance_tolerance < args.maximum_step_distance:
        parser.error('distance tolerance must be smaller than the step distance')
    if args.execute and not args.cable_zero_confirmed:
        parser.error("chassis movement requires --cable-zero-confirmed")
    return args


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
