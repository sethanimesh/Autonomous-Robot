#!/usr/bin/env python3
"""Perform one cable-safe in-place scan and stop on target confirmation."""

import argparse
import json
import math
import threading
import time
from uuid import uuid4
from concurrent.futures import Future

try:
    from robot.jetson.mission.search_view_policy import search_scene_action, upward_budget_reached, human_framing_plan, combine_framing_plans
    from robot.jetson.mission.camera_visual_setup import request_setup_view
    from robot.jetson.mission.occlusion_advice import request_occlusion_advice
    from robot.jetson.navigation.navigation_reasoning import occlusion_recovery
except ImportError:
    from search_view_policy import search_scene_action, upward_budget_reached, human_framing_plan, combine_framing_plans
    from camera_visual_setup import request_setup_view
    from occlusion_advice import request_occlusion_advice
    from navigation_reasoning import occlusion_recovery

try:
    from robot.jetson.mission.recovery_policy import same_motor_references
except ImportError:
    from recovery_policy import same_motor_references

try:
    from robot.jetson.perception.person_continuity_stream import PersonContinuityStream
except ImportError:
    from person_continuity_stream import PersonContinuityStream

try:
    from robot.jetson.mission.camera_motion_feedback import (
        frame_fingerprint,
        fingerprint_quality,
    )
except ImportError:
    from camera_motion_feedback import (
        frame_fingerprint,
        fingerprint_quality,
    )

try:
    from robot.jetson.navigation.cable_guard import project_cable_turn
    from robot.jetson.navigation.cable_guard import record_cable_turn
    from robot.jetson.navigation.cable_guard import (
        relative_scan_headings_within_cable_limit,
    )
    from robot.jetson.navigation.cable_guard import validate_measured_cable_heading
    from robot.jetson.navigation.local_planner import cable_safe_scan_headings
except ImportError:
    from cable_guard import project_cable_turn
    from cable_guard import record_cable_turn
    from cable_guard import relative_scan_headings_within_cable_limit
    from cable_guard import validate_measured_cable_heading
    from local_planner import cable_safe_scan_headings


try:
    from robot.jetson.mission.camera_control_lease import camera_control_lease
except ImportError:
    from camera_control_lease import camera_control_lease


try:
    from robot.jetson.mission.person_approach import identity_ready, approach_decision, wardrobe_view_target
except ImportError:
    from person_approach import identity_ready, approach_decision, wardrobe_view_target

class ScanError(RuntimeError):
    pass


def target_is_confirmed(payload, maximum_age_seconds=0.75):
    if isinstance(payload,dict) and payload.get('clothing_approach_enabled'):
        return identity_ready(payload, maximum_age_seconds)
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


def target_centering_step(observation, left=.35, right=.65):
    value = observation.get('center_x_fraction')
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 <= value <= 1:
        raise ScanError('Confirmed target position is unavailable')
    # The chassis stops within three degrees: eight requested gives about five
    # degrees of correction, including enough reach for a target at the edge.
    return -8 if value < left else 8 if value > right else 0


def search_view_usable(quality):
    # A plain wall is a valid search view. Texture cannot prove obstruction;
    # route segmentation separately authorizes every forward movement.
    brightness = quality.get('mean_luma')
    return isinstance(brightness, (int, float)) and math.isfinite(brightness) and brightness >= 12


def fast_search_positions(head):
    # Calibration endpoints describe travel, not where a person must be.
    # Sweep room-facing views; raise incrementally only on person evidence.
    return list(dict.fromkeys((int(head['forward_position']),
                               int(head.get('down_position', head['forward_position'])))))


def person_tilt_target(body, head):
    position = int(head['position'])
    face_y = body['top_fraction'] + .1 * body['height_fraction']
    step = -5 if face_y < .18 else 5 if face_y > .55 else 0
    return max(int(head['up_position']), min(int(head['forward_position']), position + step))


def small_lower_person_candidate(body):
    """A small lower-image fragment merits a higher look before chassis turns."""
    if not isinstance(body, dict):
        return False
    height, top = body.get('height_fraction'), body.get('top_fraction')
    return (all(isinstance(v, (int, float)) and not isinstance(v, bool)
                and math.isfinite(v) for v in (height, top))
            and 0 < height < .20 and .55 <= top <= 1)


def face_tilt_target(face, head):
    """Improve edge framing without requiring a complete face for identity."""
    position = int(head['position'])
    y, height = face.get('center_y_fraction'), face.get('box_height_fraction')
    if not all(isinstance(v, (int, float)) and not isinstance(v, bool)
               and math.isfinite(v) for v in (y, height)):
        return position
    step = -8 if y - height / 2 < .04 or y < .18 else 8 if y > .75 else 0
    return max(int(head['up_position']), min(int(head['down_position']), position + step))


def person_investigation_plan(body, face, head):
    """Choose the next view from fresh evidence, never from assumed body height."""
    current = int(head['position'])
    center = False
    if face:
        y = face.get('center_y_fraction', (face['top_fraction'] + face['bottom_fraction']) / 2.)
        if face['top_fraction'] < .04 or y < .18:
            scenario = 'face_above_view'
            options = [current - 8]
        elif face['bottom_fraction'] > .96 or y > .75:
            scenario = 'face_below_view'
            options = [current + 8]
        elif target_centering_step(face):
            scenario, options, center = 'face_at_side', [], True
        else:
            # The face is already framed. Keep its view while the recognition
            # worker gathers independent matches, including partial profiles.
            scenario, options = 'face_visible_identity_pending', []
    elif body and body['top_fraction'] < .04:
        scenario = 'body_cropped_above'
        # A torso can fill the view. Raise a little, then recheck the person
        # instead of jumping past their head to the ceiling.
        options = [current - 8]
    elif small_lower_person_candidate(body):
        scenario, options = 'small_lower_body_fragment', [current - 8]
    elif body:
        face_y = body['top_fraction'] + .1 * body['height_fraction']
        if face_y < .18:
            scenario, options = 'body_suggests_higher_face', [current - 8]
        elif face_y > .55:
            scenario, options = 'body_suggests_lower_face', [current + 8]
        elif target_centering_step(body):
            scenario, options, center = 'body_at_side', [], True
        else:
            scenario, options = 'body_only_check_upper_view', [current - 8]
    else:
        scenario, options = 'person_temporarily_missing', []
    positions = list(dict.fromkeys(max(int(head['minimum_target_position']),
        min(int(head['maximum_target_position']), p)) for p in options))
    return dict(scenario=scenario, positions=positions, center_person=center)


def preferred_search_positions(head, position=None, reference=None):
    positions = fast_search_positions(head)
    if (position is not None and reference == head.get('reference_id')
            and head['minimum_target_position'] <= position <= head['maximum_target_position']):
        positions.insert(0, position)
    return list(dict.fromkeys(positions))


def continuity_pose(robot, head):
    """Stationary image tracking is invalid after camera or track movement."""
    motors = (robot or {}).get('motors') or {}
    head = head or {}
    values = (head.get('position'), motors.get('left', {}).get('position'),
              motors.get('right', {}).get('position'))
    if not all(isinstance(v, (int, float)) and not isinstance(v, bool)
               and math.isfinite(v) for v in values):
        return None
    return (head.get('reference_id'),) + values


def continuity_pose_changed(previous, current):
    return (previous is not None and current is not None
            and (previous[0] != current[0]
                 or any(abs(a-b) > 3 for a, b in zip(previous[1:], current[1:]))))


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
    for name in ('left', 'right'):
        motor = status.get('motors', {}).get(name, {})
        for field in ('speed', 'commanded_speed'):
            if field in motor:
                value = motor[field]
                if (not isinstance(value, (int, float)) or not math.isfinite(value)
                        or abs(value) > (5 if field == 'speed' else 0)):
                    return True
    if "track_motion_active" in status:
        return bool(status["track_motion_active"])
    return bool(status.get("motion_active", True)) and not bool(
        status.get("tool_motion_active", False)
    )


def next_tilt_toward(current, target, maximum_step=5, tolerance=3):
    current = int(current)
    target = int(target)
    if abs(target - current) <= int(tolerance):
        return None
    delta = target - current
    return max(-abs(int(maximum_step)), min(abs(int(maximum_step)), delta))


def background_scene_request(callback, *arguments):
    """A read-only RPC must not hold the scan process open after local success."""
    future = Future()
    def work():
        if not future.set_running_or_notify_cancel():
            return
        try:
            future.set_result(callback(*arguments))
        except Exception as exc:
            future.set_exception(exc)
    threading.Thread(target=work, name='scene-advice', daemon=True).start()
    return future


def run(args):
    if not args.execute:
        return _run(args)
    with camera_control_lease(exclusive=True):
        return _run(args)


def _run(args):
    headings = cable_safe_scan_headings(args.step_degrees, args.sweep_limit_degrees)
    if args.first_direction == "left":
        headings = tuple(-heading for heading in headings)
    initial_cable_heading = validate_measured_cable_heading(
        args.initial_cable_heading_degrees,
        args.cable_limit_degrees,
        args.cable_margin_degrees,
    )
    scan_origin_heading = (
        initial_cable_heading if args.preserve_initial_heading else 0.0
    )
    headings = relative_scan_headings_within_cable_limit(
        headings,
        scan_origin_heading,
        args.cable_limit_degrees,
        args.cable_margin_degrees,
    )
    planned_turns = incremental_scan_turns(headings)
    if initial_cable_heading and not args.preserve_initial_heading:
        planned_turns = (-initial_cable_heading,) + planned_turns
    report = {
        "outcome": "failure",
        "started_at_unix": time.time(),
        "requested_headings": headings,
        "observed_headings": [initial_cable_heading],
        "cable_limit_degrees": args.cable_limit_degrees,
        "cable_margin_degrees": args.cable_margin_degrees,
        "scan_origin_heading_degrees": scan_origin_heading,
        "turns": [],
    }
    if not args.execute:
        report["outcome"] = "dry_run_success"
        report["incremental_turns"] = planned_turns
        report["final_cable_heading_degrees"] = (
            initial_cable_heading + sum(planned_turns)
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
            self.profile_id=args.profile_id
            self.profile_revision=args.profile_revision
            self.robot = None
            self.robot_at = None
            self.camera = None
            self.camera_at = None
            self.camera_frame_at = None
            self.frame_sequence = 0
            self.frame_fingerprint = None
            self.frame_error = None
            self.source_frame = None
            self.scene_future = None
            self.search_scene_memory = None
            self.search_height_limits = {}
            self.image_width = 640
            self.image_height = 480
            self.body = None
            self.body_at = None
            self.face = None
            self.face_at = None
            self.face_source_age = 999.0
            self.head_status = None
            self.head_at = None
            self.target = None
            self.target_at = None
            self.continuity = PersonContinuityStream(required_face_hits=2)
            self.continuity_pose_baseline = None
            self.continuity_identity = None
            self.yaw = None
            self.yaw_at = None
            self.cmd = self.create_publisher(Twist, "/cmd_vel", 1)
            self.head = self.create_publisher(String, "/camera_head/command", 1)
            self.create_subscription(String, "/robot_status", self.on_robot, 1)
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
            self.create_subscription(String, "/camera_head/status", self.on_head, 1)
            self.create_subscription(Detection2DArray, '/perception/target_matches',
                                     lambda m: self.offer_continuity('matches', m), 10)
            self.create_subscription(Detection2DArray, '/perception/face_detections', self.on_faces, 10)
            self.create_subscription(
                String, "/mission/target_observation", self.on_target, 10
            )
            self.create_subscription(Odometry, "/odom", self.on_odom, 1)

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
                self.update_continuity_pose()

        def on_camera(self, message):
            value = self.parse(message)
            if value is not None:
                self.camera, self.camera_at = value, time.monotonic()

        def on_camera_frame(self, message):
            self.source_frame = message
            self.camera_frame_at = time.monotonic()
            self.image_width = int(message.width)
            self.image_height = int(message.height)
            try:
                self.frame_fingerprint = frame_fingerprint(
                    message.data,
                    message.width,
                    message.height,
                    message.step,
                    message.encoding,
                )
                self.frame_error = None
            except (TypeError, ValueError) as exc:
                self.frame_fingerprint = None
                self.frame_error = str(exc)
            self.frame_sequence += 1
            if self.frame_error is None:
                self.offer_continuity('image', message)
            else:
                self.reset_continuity()

        def on_faces(self, message):
            self.face = largest_body_observation([
                (d.bbox.center.position.x, d.bbox.center.position.y, d.bbox.size_x, d.bbox.size_y)
                for d in message.detections
            ], self.image_width, self.image_height)
            self.face_at = time.monotonic()
            source = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
            self.face_source_age = max(0.0, self.get_clock().now().nanoseconds / 1e9 - source)

        def face_is_visible(self):
            return (self.face is not None and self.face_at is not None
                    and time.monotonic() - self.face_at + self.face_source_age <= .75)

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
            self.offer_continuity('people', message)

        def on_head(self, message):
            value = self.parse(message)
            if value is not None:
                self.head_status, self.head_at = value, time.monotonic()
                self.update_continuity_pose()

        def on_target(self, message):
            value = self.parse(message)
            if value is not None:
                if ((getattr(self,'profile_id',None) and value.get('profile_id')!=self.profile_id)
                        or (getattr(self,'profile_revision',None) and value.get('target_revision')!=self.profile_revision)):
                    self.target=None
                    self.target_at=None
                    return
                identity = (value.get('profile_id') or value.get('target_label'), value.get('target_revision'))
                if identity != self.continuity_identity:
                    self.reset_continuity()
                    self.continuity_identity = identity
                self.target, self.target_at = value, time.monotonic()

        def reset_continuity(self):
            self.continuity.reset(self.get_clock().now().nanoseconds / 1e9)

        def update_continuity_pose(self):
            current = continuity_pose(self.robot, self.head_status)
            if (chassis_motion_active(self.robot) or (self.head_status or {}).get('moving', True)
                    or (self.head_status or {}).get('homing', True)
                    or continuity_pose_changed(self.continuity_pose_baseline, current)):
                self.reset_continuity()
                self.continuity_pose_baseline = current
            elif self.continuity_pose_baseline is None:
                self.continuity_pose_baseline = current

        def offer_continuity(self, name, message):
            if self.motion_reason() is not None:
                self.reset_continuity()
                return
            self.continuity.offer(name, message, self.get_clock().now().nanoseconds / 1e9)

        def wait_for_tracked_face(self):
            """Hold the current view only while a recent identified person remains.

            Every successful return requires a fresh face confirmation. Body
            continuity never creates target_found or a face-size distance cue.
            """
            started = time.monotonic()
            hint = self.continuity.retained(self.get_clock().now().nanoseconds / 1e9)
            if hint is None:
                return self.target_is_visible()
            deadline = started + max(0., self.continuity.tracker.maximum_face_age_seconds
                                     - hint['face_age_seconds'])
            while time.monotonic() < deadline:
                if not self.safety_ready():
                    if self.recover_feedback(self.motion_reason()):
                        continue
                    raise ScanError(self.safety_reason() or 'View unavailable during person retention')
                if self.target_is_visible():
                    report.setdefault('tracking_holds', []).append({
                        'seconds': round(time.monotonic()-started, 3), 'result': 'face_returned'})
                    return True
                hint = self.continuity.retained(self.get_clock().now().nanoseconds / 1e9)
                if hint is None:
                    break
                remaining = deadline - time.monotonic()
                if remaining > 0:
                    rclpy.spin_once(self, timeout_sec=min(.05, remaining))
            report.setdefault('tracking_holds', []).append({
                'seconds': round(time.monotonic()-started, 3), 'result': 'face_not_returned'})
            return False

        def on_odom(self, message):
            q = message.pose.pose.orientation
            self.yaw = math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)
            self.yaw_at = time.monotonic()

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
            # Clear the bridge's active velocity and request a direct stop.
            # Zero velocity alone previously lost its stopped acknowledgement.
            self.head.publish(String(data='stop'))
            deadline = time.monotonic() + 3.0
            next_zero = 0.0
            while time.monotonic() < deadline:
                if time.monotonic() >= next_zero:
                    self.velocity()
                    next_zero = time.monotonic() + .2
                rclpy.spin_once(self, timeout_sec=0.05)
                if (self.robot_at is not None and self.robot_at > previous
                        and not chassis_motion_active(self.robot)):
                    return
            raise ScanError('stopped status was not confirmed')

        def startup_ready(self):
            now = time.monotonic()
            return (
                self.robot_at is not None
                and now - self.robot_at <= 1.0
                and not chassis_motion_active(self.robot)
                and self.head_at is not None
                and now - self.head_at <= 1.0
                and not self.head_status.get("moving", True)
                and not self.head_status.get("homing", True)
                and self.camera_at is not None
                and now - self.camera_at <= 6.5
                and self.camera.get("state") == "streaming"
                and self.camera_frame_at is not None
                and now - self.camera_frame_at <= 0.5
                and self.frame_fingerprint is not None
                and self.yaw is not None
            )

        def safety_ready(self):
            return self.safety_reason() is None

        def safety_reason(self, allow_chassis_motion=False):
            now = time.monotonic()
            motion_reason = self.motion_reason(allow_chassis_motion)
            if motion_reason:
                return motion_reason
            h = self.head_status
            if (not h.get('calibrated') or h.get('manual_override')
                    or not h.get('reference_id') or h.get('reference_id') != h.get('approved_reference_id')):
                return "camera reference or saved limits became unavailable"
            if self.camera_at is None or now - self.camera_at > 6.5:
                return "camera health status is stale"
            if self.camera.get("state") != "streaming":
                return "camera is not streaming"
            if self.camera_frame_at is None or now - self.camera_frame_at > (1.5 if allow_chassis_motion else .75):
                return "live camera frame heartbeat is stale"
            if float(self.camera.get("mean_intensity", 0.0)) < 15.0:
                return "camera image is too dark"
            return None

        def motion_ready(self):
            return self.motion_reason() is None

        def motion_reason(self, allow_chassis_motion=False):
            now = time.monotonic()
            # Idle EV3 feedback arrives about twice a second, with measured
            # gaps near one second. Leave room for callback scheduling.
            maximum_age = 1.5 if allow_chassis_motion else 2.5
            if self.robot_at is None or now - self.robot_at > maximum_age:
                return "robot status is stale"
            if not allow_chassis_motion and chassis_motion_active(self.robot):
                return "robot unexpectedly reports motion"
            if self.head_at is None or now - self.head_at > maximum_age:
                return "camera-head status is stale"
            if not self.head_status.get("homed", False):
                return "camera head is not homed"
            if self.head_status.get("moving", True) or self.head_status.get(
                "homing", True
            ):
                return "camera head is moving"
            if self.yaw is None or not math.isfinite(self.yaw):
                return "odometry is unavailable"
            if self.yaw_at is None or now - self.yaw_at > maximum_age:
                return "odometry is stale"
            return None

        def recover_feedback(self, reason):
            """Keep the current search/turn after a brief telemetry delay."""
            if reason not in ('robot status is stale', 'camera-head status is stale', 'odometry is stale', 'robot unexpectedly reports motion'):
                return False
            recoveries = report.setdefault('feedback_recoveries', [])
            if len(recoveries) >= 3:
                return False
            started = time.monotonic()
            reference = (self.head_status or {}).get('reference_id')
            generations = {name: (self.robot or {}).get('motors', {}).get(name, {}).get('generation')
                           for name in ('left', 'right')}
            self.stop()
            self.spin_until(lambda: self.motion_ready()
                            and all(stamp is not None and stamp > started
                                    for stamp in (self.robot_at, self.head_at, self.yaw_at)),
                            5.0, 'Robot feedback did not return after waiting')
            if (not reference or self.head_status.get('reference_id') != reference
                    or any(not generation or self.robot.get('motors', {}).get(name, {}).get('generation') != generation
                           for name, generation in generations.items())):
                raise ScanError('Motor reference changed during feedback recovery')
            recoveries.append({'reason': reason, 'wait_seconds': round(time.monotonic()-started, 3)})
            return True

        def look_forward(self):
            self.look_at_search_position(int(self.head_status['forward_position']))

        def ensure_homed(self):
            h = self.head_status
            saved = h.get('saved_limits') or {}
            if (not h.get('calibrated') and h.get('homed') and not h.get('moving')
                    and not h.get('homing') and not h.get('manual_override')
                    and h.get('reference_id')
                    and h['reference_id'] == h.get('approved_reference_id') == saved.get('reference_id')
                    and saved.get('upper') is not None and saved.get('lower') is not None):
                previous = self.head_at or 0.
                reference = h['reference_id']
                self.head.publish(String(data='use_saved_limits'))
                self.spin_until(lambda: self.head_at > previous
                                and self.head_status.get('calibrated')
                                and self.head_status.get('reference_id') == reference,
                                4., 'Saved camera limits could not be revalidated')
                h = self.head_status
            if (not h.get('homed') or not h.get('calibrated') or h.get('manual_override')
                    or not h.get('reference_id') or h['reference_id'] != h.get('approved_reference_id')):
                raise ScanError('Set the camera limits in the UI and complete view calibration before scanning')

        def recover_camera(self, reason):
            if reason not in ('camera health status is stale', 'camera is not streaming',
                              'live camera frame heartbeat is stale', 'camera image is too dark'):
                return False
            started = time.monotonic()
            before_robot = self.robot
            reference = self.head_status.get('reference_id')
            self.stop()
            self.reset_continuity()
            sequence = self.frame_sequence
            self.wait_for_camera_after_head_move()
            self.spin_until(lambda: self.frame_sequence >= sequence + 3 and self.safety_ready(),
                            args.camera_recovery_seconds, 'fresh camera frames unavailable during recovery')
            if (not reference or reference != self.head_status.get('reference_id')
                    or not same_motor_references(before_robot, self.robot)):
                raise ScanError('Motor reference changed during camera recovery')
            report.setdefault('camera_recoveries', []).append(dict(
                reason=reason, wait_seconds=round(time.monotonic()-started, 3),
                continued_same_step=True))
            return True

        def wait_for_camera_after_head_move(self, timeout=None):
            timeout = (
                args.camera_recovery_seconds if timeout is None else float(timeout)
            )
            started_at = time.monotonic()
            recovery_needed = not (
                self.camera_at is not None
                and started_at - self.camera_at <= 6.5
                and self.camera.get("state") == "streaming"
                and self.camera_frame_at is not None
                and started_at - self.camera_frame_at <= 0.5
            )
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                motion_problem = self.motion_reason()
                if motion_problem and not self.recover_feedback(motion_problem):
                    raise ScanError(motion_problem)
                now = time.monotonic()
                if (
                    self.camera_at is not None
                    and now - self.camera_at <= 6.5
                    and self.camera.get("state") == "streaming"
                    and self.camera_frame_at is not None
                    and now - self.camera_frame_at <= 0.5
                ):
                    if recovery_needed:
                        report.setdefault("camera_recoveries", []).append(
                            {
                                "wait_seconds": round(now - started_at, 3),
                                "state": self.camera.get("state"),
                            }
                        )
                    return
            raise ScanError("camera did not recover after camera-head movement")

        def capture_frame_after(self, sequence, observed_after, timeout=3.0):
            self.spin_until(
                lambda: self.frame_sequence > sequence
                and self.camera_frame_at is not None
                and self.camera_frame_at >= observed_after
                and self.frame_fingerprint is not None,
                timeout,
                self.frame_error or "no fresh frame arrived for camera feedback",
            )
            return (
                self.frame_sequence,
                self.camera_frame_at,
                self.frame_fingerprint,
            )

        def verify_head_view_change(self, before_sequence, before_frame):
            """Accept a fresh usable frame after settling; people may move."""

            self.wait_for_camera_after_head_move()
            stopped_at = time.monotonic()
            # Allow linkage flex to settle; do not compare pictures of a moving person.
            settle_deadline = stopped_at + args.visual_settle_seconds
            while time.monotonic() < settle_deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
            after_sequence, after_at, after_frame = self.capture_frame_after(
                before_sequence, settle_deadline
            )
            hold_deadline = after_at + args.visual_hold_seconds
            while time.monotonic() < hold_deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
            _, _, settled_frame = self.capture_frame_after(
                after_sequence, hold_deadline
            )
            quality = fingerprint_quality(settled_frame)
            visible = search_view_usable(quality)
            feedback = dict(verified=visible, method='fresh_lit_search_view', quality=quality)
            report.setdefault('camera_motion_feedback', []).append(feedback)
            if not visible:
                raise ScanError('Camera view is too dark after movement')
            return feedback

        def prepare(self):
            self.spin_until(
                self.startup_ready,
                max(10.0, args.camera_recovery_seconds),
                "scan sensors are unavailable",
            )
            self.stop()
            self.ensure_homed()
            if not (args.fast_search and not args.vertical_only):
                self.look_forward()

        def turn_relative(self, image_degrees, camera_required=True):
            ready = self.safety_ready if camera_required else self.motion_ready
            if not ready():
                # A stopped transition may receive robot and head messages in
                # separate callbacks. Give fresh feedback a chance to arrive.
                self.spin_until(ready, 2.0, "sensor state unavailable before turn")
            self.reset_continuity()
            # Scan headings use image convention: positive is right.
            target = math.radians(-float(image_degrees))
            start = self.yaw
            start_status = self.robot
            direction = 1.0 if target > 0 else -1.0
            deadline = time.monotonic() + args.turn_timeout
            progress_at = time.monotonic()
            best_progress = 0.0
            retried = False
            reference = self.head_status.get('reference_id')
            generations = {name: self.robot.get('motors', {}).get(name, {}).get('generation')
                           for name in ('left', 'right')}
            try:
                while time.monotonic() < deadline:
                    rclpy.spin_once(self, timeout_sec=0.04)
                    reason = (self.safety_reason(allow_chassis_motion=True) if camera_required
                              else self.motion_reason(allow_chassis_motion=True))
                    if reason:
                        recovery_started = time.monotonic()
                        if self.recover_feedback(reason) or self.recover_camera(reason):
                            deadline += time.monotonic() - recovery_started
                            progress_at = time.monotonic()
                            continue
                        raise ScanError(reason)
                    moved = math.atan2(
                        math.sin(self.yaw - start), math.cos(self.yaw - start)
                    )
                    progress = direction * moved
                    if progress < -math.radians(3):
                        raise ScanError("chassis turned in the wrong direction")
                    if progress > best_progress + math.radians(1):
                        best_progress, progress_at = progress, time.monotonic()
                    if time.monotonic() - progress_at > 2.0:
                        if retried:
                            raise ScanError("chassis turn made no progress after retry")
                        self.stop()
                        self.spin_until(ready, 2.0, 'fresh stopped feedback unavailable for turn retry')
                        if (not reference or self.head_status.get('reference_id') != reference
                                or any(not generation or self.robot.get('motors', {}).get(name, {}).get('generation') != generation
                                       for name, generation in generations.items())):
                            raise ScanError('motor reference changed before turn retry')
                        report.setdefault('turn_retries', []).append({
                            'reason': 'no_progress', 'requested_degrees': image_degrees,
                            'remaining_degrees': math.degrees(max(0., abs(target)-progress))})
                        retried = True
                        progress_at = time.monotonic()
                        deadline += 2.0
                        continue
                    remaining = abs(target) - progress
                    if remaining <= math.radians(args.yaw_tolerance_degrees):
                        break
                    speed = min(args.turn_speed, max(0.16, remaining * 1.4))
                    self.velocity(direction * speed)
                else:
                    raise ScanError("scan turn timed out")
            finally:
                self.stop()
                self.reset_continuity()
            stopped_at = time.monotonic()
            self.spin_until(lambda: self.yaw_at is not None and self.yaw_at > stopped_at,
                            2.5, 'stopped odometry is unavailable')
            if not same_motor_references(start_status, self.robot):
                raise ScanError('motor reference changed while turning')
            return -math.degrees(math.atan2(math.sin(self.yaw-start), math.cos(self.yaw-start)))

        def wait_for_target(self, duration=None):
            dwell = args.dwell_seconds if duration is None else float(duration)
            deadline = time.monotonic() + dwell
            recovered_once = False
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                if not self.safety_ready():
                    motion_problem = self.motion_reason()
                    if motion_problem and not self.recover_feedback(motion_problem):
                        raise ScanError(motion_problem)
                    if recovered_once:
                        raise ScanError(
                            self.safety_reason()
                            or "camera became unsafe again while observing"
                        )
                    self.wait_for_camera_after_head_move()
                    recovered_once = True
                    deadline = time.monotonic() + dwell
                if self.target_is_visible():
                    return True
            return False

        def target_is_visible(self):
            if self.target_at is None:
                return False
            elapsed = time.monotonic() - self.target_at
            return (0.0 <= elapsed <= 0.5
                    and target_is_confirmed(self.target, maximum_age_seconds=.75-elapsed))

        def body_is_visible(self):
            return (
                self.body is not None
                and self.body_at is not None
                and time.monotonic() - self.body_at <= 1.0
            )

        def seek_target_upward(self, requested_limit=None):
            """Inspect bounded higher views using the same scene/person rules."""
            self.stop()
            moves = 0
            origin = int(self.head_status['position'])
            while True:
                if self.target_is_visible():
                    return True
                advice = self.assess_search_scene()
                if self.target_is_visible():
                    return True
                face_visible = self.face_is_visible()
                if search_scene_action(advice, face_visible) == 'return_from_ceiling':
                    report.setdefault('view_returns', []).append('ceiling_view')
                    return False
                if self.body_is_visible() or face_visible or human_framing_plan(advice, self.head_status):
                    return self.investigate_person(initial_advice=advice)
                position = int(self.head_status.get("position", -999))
                limit = int(
                    self.head_status.get(
                        "up_position",
                        self.head_status.get("minimum_position", -999),
                    )
                )
                if requested_limit is not None:
                    candidate = int(requested_limit)
                    if min(position, limit) <= candidate <= max(position, limit):
                        limit = candidate
                step = next_tilt_toward(
                    position, limit, args.tilt_step_degrees, int(self.head_status.get("settle_tolerance", 3))
                )
                if step is None or moves >= 3 or upward_budget_reached(origin, position):
                    report.setdefault('view_returns', []).append('empty_higher_views')
                    return False
                moves += 1
                self.look_at_search_position(position + step)
                if self.wait_for_target(args.up_dwell_seconds):
                    return True

        def look_at_search_position(self, target):
            self.ensure_homed()
            if not self.head_status['minimum_target_position'] <= target <= self.head_status['maximum_target_position']:
                raise ScanError('Search view is outside saved camera limits')
            if (not self.head_status.get('moving', True)
                    and not self.head_status.get('homing', True)
                    and abs(int(self.head_status['position']) - target)
                    <= int(self.head_status.get('settle_tolerance', 4))):
                return
            previous = self.head_at or 0.0
            sequence, frame = self.frame_sequence, self.frame_fingerprint
            self.reset_continuity()
            # The bridge owns the hardware-limited chunks. Inspect the image
            # only at the requested final view, not after every motor chunk.
            request_id = uuid4().hex
            self.head.publish(String(data=json.dumps(dict(action='move_to', target=target, request_id=request_id))))
            self.spin_until(lambda: self.head_at > previous
                            and (self.head_status.get('move_result') or {}).get('request_id') == request_id,
                            15.0, 'Camera search command did not finish')
            result = self.head_status['move_result']
            if not result.get('ok'):
                raise ScanError(result.get('error') or 'Camera search command failed')
            self.spin_until(lambda: self.head_at > previous
                            and self.head_status.get('target_position') == target
                            and not self.head_status.get('moving', True)
                            and not self.head_status.get('homing', True)
                            and self.head_status['minimum_position'] <= self.head_status['position'] <= self.head_status['maximum_position']
                            and abs(int(self.head_status['position']) - target) <= int(self.head_status.get('settle_tolerance', 4)),
                            15.0, 'Camera did not reach the search view')
            self.reset_continuity()
            self.verify_head_view_change(sequence, frame)
            report.setdefault('camera_tilt_positions', []).append(int(self.head_status['position']))

        def assess_search_scene(self):
            """Ask Gemini about a stopped view; keep ROS and Stop responsive."""
            if self.face_is_visible() or (self.scene_future and not self.scene_future.done()):
                return None
            if not self.safety_ready() or self.source_frame is None:
                return None
            frame = self.source_frame
            age = (self.get_clock().now().nanoseconds -
                   (frame.header.stamp.sec * 1_000_000_000 + frame.header.stamp.nanosec)) / 1e9
            if not -.25 <= age <= 1.:
                return None
            reference, position, yaw = self.head_status['reference_id'], self.head_status['position'], self.yaw
            try:
                import cv2
                from cv_bridge import CvBridge
                pixels = CvBridge().imgmsg_to_cv2(frame, desired_encoding='bgr8')
                scale = min(1., 448. / max(pixels.shape[:2]))
                if scale < 1:
                    pixels = cv2.resize(pixels, None, fx=scale, fy=scale)
                ok, jpeg = cv2.imencode('.jpg', pixels, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if not ok:
                    return None
                jpeg = jpeg.tobytes()
                previous = self.search_scene_memory
                captured_at = time.monotonic() - max(0., age)
                compare = (previous is not None and not self.body_is_visible()
                           and previous['reference'] == reference
                           and 0 <= captured_at - previous['captured_at'] <= 30
                           and abs(math.atan2(math.sin(yaw - previous['yaw']),
                                               math.cos(yaw - previous['yaw']))) <= math.radians(3)
                           and same_motor_references(previous['robot'], self.robot))
                if compare:
                    self.scene_future = background_scene_request(request_occlusion_advice,
                        previous['jpeg'], jpeg, args.search_view_url,
                        abs(position - previous['position']) > 3, 11.)
                else:
                    self.scene_future = background_scene_request(
                        request_setup_view, jpeg, args.search_view_url, args.search_view_timeout_seconds)
                local_person = self.body_is_visible()
                capture_robot = self.robot
                deadline = time.monotonic() + (11. if compare else args.search_view_timeout_seconds)
                while not self.scene_future.done() and time.monotonic() < deadline:
                    rclpy.spin_once(self, timeout_sec=.05)
                    if not self.safety_ready() or chassis_motion_active(self.robot):
                        self.stop()
                        return None
                    if self.target_is_visible() or self.face_is_visible():
                        return None
                if not self.scene_future.done():
                    raise TimeoutError('scene advice deadline')
                result = self.scene_future.result()
                # Bind the answer to the actual stopped pose, not a later view.
                observed_after = time.monotonic()
                self.spin_until(lambda: self.head_at > observed_after and self.robot_at > observed_after,
                                1., 'Fresh pose unavailable after scene advice')
                if (not self.safety_ready() or self.head_status['reference_id'] != reference
                        or abs(self.head_status['position'] - position) > 4
                        or abs(math.atan2(math.sin(self.yaw - yaw), math.cos(self.yaw - yaw))) > math.radians(3)
                        or not same_motor_references(capture_robot, self.robot)):
                    return None
                if compare:
                    result['occlusion_recovery'] = occlusion_recovery(result.get('interpretation'),
                        previous['position'], position, memory_age=time.monotonic() - previous['captured_at'],
                        same_reference=previous['reference'] == reference, face_visible=self.face_is_visible())
                    report.setdefault('occlusion_checks', []).append(dict(
                        interpretation=result.get('interpretation'), recovery=result['occlusion_recovery'],
                        frame_sha256=result.get('frame_sha256'), model=result.get('model')))
                    # Consume this pair. Repeated missing observations cannot
                    # keep the candidate alive or spend unbounded cloud time.
                    self.search_scene_memory = None
                    return result
                observations = result.get('observations') or []
                if local_person or (observations and observations[0].get('human_visible') == 'yes'):
                    self.search_scene_memory = dict(jpeg=jpeg, position=position, reference=reference,
                        yaw=yaw, robot=capture_robot, captured_at=captured_at)
                report.setdefault('search_scene_checks', []).append(dict(
                    position=position, model=result.get('model'), provider=result.get('provider'),
                    observations=result.get('observations'), elapsed_seconds=result.get('elapsed_seconds')))
                return result
            except Exception as exc:
                report.setdefault('search_scene_fallbacks', []).append(type(exc).__name__)
                report.setdefault('search_scene_errors', []).append(str(exc)[:180])
                return None

        def investigate_person(self, initial_advice=None):
            # Explore useful nearby views without requiring a frontal face or
            # complete body. Only fresh identity can finish identification.
            deadline = time.monotonic() + args.candidate_seconds
            hard_deadline = time.monotonic() + 60.
            origin = int(self.head_status['position'])
            entry_view = dict(position=origin, reference=self.head_status['reference_id'])
            heading_key = (entry_view['reference'], round(math.degrees(self.yaw or 0.) / 10))
            ceiling_limit = self.search_height_limits.get(heading_key)
            if ceiling_limit and time.monotonic() - ceiling_limit[1] > 60:
                ceiling_limit = None
            cloud_checks = 0
            upward_checks = 0
            guided_human = False
            moves = 0
            previous_view = origin
            visited = set()
            best_face_view = None
            last_body_view = None
            while time.monotonic() < min(deadline, hard_deadline):
                visited.add(int(self.head_status['position']))
                if self.wait_for_target(1.2):
                    return True
                family_observation=getattr(self,'target',None) or {}
                guidance=family_observation.get('wardrobe_guidance') or {}
                if family_observation.get('clothing_approach_enabled') and guidance.get('pending'):
                    # The observer keeps local tracking while its single cloud
                    # comparison runs; moving now would discard that result.
                    deadline=min(hard_deadline,max(deadline,time.monotonic()+1.5))
                    continue
                if self.continuity.retained(self.get_clock().now().nanoseconds / 1e9) is not None:
                    if self.wait_for_tracked_face():
                        return True
                if self.face_is_visible():
                    report['face_observation'] = dict(self.face)
                    observed_face = self.face
                    score = (min(observed_face['top_fraction'], 1. - observed_face['bottom_fraction']),
                             observed_face['height_fraction'])
                    if best_face_view is None or score > best_face_view['score']:
                        best_face_view = dict(score=score, position=int(self.head_status['position']),
                                              reference=self.head_status['reference_id'])
                    if self.wait_for_target(1.8):
                        return True
                    # The wait may outlast this face observation. Do not use
                    # a stale box to direct the next movement.
                face = dict(self.face) if self.face_is_visible() else None
                body = dict(self.body) if self.body_is_visible() else None
                if body and last_body_view is None:
                    last_body_view = dict(position=int(self.head_status['position']),
                                          reference=self.head_status['reference_id'])
                current = int(self.head_status['position'])
                advice = None
                if not face and cloud_checks < 4 and (initial_advice is not None or
                        hard_deadline - time.monotonic() >= args.search_view_timeout_seconds):
                    cloud_checks += 1
                    started = time.monotonic()
                    advice = initial_advice if initial_advice is not None else self.assess_search_scene()
                    initial_advice = None
                    # Network latency is not time spent observing the person.
                    deadline += time.monotonic() - started
                    if self.target_is_visible():
                        return True
                    if self.face_is_visible():
                        face = dict(self.face)
                        best_face_view = dict(position=current, reference=entry_view['reference'],
                            score=(min(face['top_fraction'], 1. - face['bottom_fraction']), face['height_fraction']))
                    body = dict(self.body) if self.body_is_visible() else None
                    if search_scene_action(advice, bool(face)) == 'return_from_ceiling':
                        self.search_height_limits[heading_key] = (previous_view, time.monotonic())
                        report.setdefault('view_returns', []).append('ceiling_view')
                        break
                cloud_plan = human_framing_plan(advice, self.head_status, bool(face))
                if cloud_plan:
                    guided_human = True
                    report['cloud_person_candidate'] = dict(position=current,
                        reference=entry_view['reference'], visible_parts=cloud_plan['visible_parts'])
                    if last_body_view is None or any(p in ('head', 'face') for p in cloud_plan['visible_parts']):
                        last_body_view = dict(position=current, reference=entry_view['reference'])
                if not face and not body and not cloud_plan:
                    recovery = (advice or {}).get('occlusion_recovery', {})
                    if recovery.get('action') == 'stop':
                        raise ScanError(recovery['reason'])
                    if recovery.get('action') == 'restore_previous_view':
                        last_body_view = dict(position=recovery['position'], reference=entry_view['reference'])
                    elif recovery.get('action') == 'wait_then_rescan':
                        if self.wait_for_target(recovery['wait_seconds']):
                            return True
                    report.setdefault('view_returns', []).append('person_lost')
                    if current < previous_view:
                        self.search_height_limits[heading_key] = (previous_view, time.monotonic())
                    break
                if moves >= 6 or (not guided_human and not face and
                        (upward_checks >= 3 or upward_budget_reached(origin, current))):
                    self.search_height_limits[heading_key] = (origin, time.monotonic())
                    report.setdefault('view_returns', []).append('upward_checks_exhausted')
                    break
                local_plan = person_investigation_plan(body, face, self.head_status) if body or face else None
                plan = combine_framing_plans(cloud_plan, local_plan, current, bool(face))
                wardrobe_target=wardrobe_view_target(guidance,self.head_status) if family_observation.get('clothing_approach_enabled') else None
                if wardrobe_target is not None and not face:
                    plan=dict(plan,positions=[wardrobe_target]+list(plan['positions']),center_person=False)
                decision = dict(plan, position=int(self.head_status['position']))
                report.setdefault('candidate_decisions', []).append(decision)
                if plan['center_person']:
                    # The caller owns bounded chassis turns. Let it center now
                    # instead of spending the candidate timeout on unrelated tilts.
                    decision['action'] = 'center_person'
                    return False
                options = plan['positions']
                target = next((p for p in options if all(abs(p-old)>3 for old in visited)
                               and (face or not ceiling_limit or p >= ceiling_limit[0])), None)
                if target is None:
                    decision['action'] = 'retain_useful_view' if best_face_view or last_body_view else 'continue_scan'
                    break
                decision.update(action='tilt_camera', target_position=target)
                previous_view = current
                upward_checks += int(target < current)
                started = time.monotonic()
                self.look_at_search_position(target)
                deadline += time.monotonic() - started
                moves += 1
            retained = best_face_view or last_body_view or entry_view
            if retained and retained['reference'] == self.head_status.get('reference_id'):
                # Actual stopped position can differ slightly from a target.
                # Restore within the commandable range, including at a limit.
                restored_position = max(int(self.head_status['minimum_target_position']),
                    min(int(self.head_status['maximum_target_position']), retained['position']))
                self.look_at_search_position(restored_position)
                report.setdefault('retained_face_views' if best_face_view else 'retained_body_views', []).append(restored_position)
                return self.wait_for_target(1.2)
            return self.target_is_visible()

    rclpy.init()
    node = ScanNode()
    cable_heading = initial_cable_heading

    def guarded_turn(requested_degrees, reason=None, camera_required=True):
        nonlocal cable_heading
        checkpoint = {'requested_degrees': requested_degrees,
                      'initial_cable_heading_degrees': cable_heading,
                      'before': dict((name, dict((node.robot or {}).get('motors', {}).get(name, {})))
                                     for name in ('left', 'right'))}
        report.setdefault('turn_encoder_checkpoints', []).append(checkpoint)
        project_cable_turn(
            cable_heading,
            requested_degrees,
            args.cable_limit_degrees,
            args.cable_margin_degrees,
        )
        turn_start_yaw, turn_start_status = node.yaw, node.robot
        try:
            actual = node.turn_relative(
                requested_degrees, camera_required=camera_required
            )
        except Exception as turn_error:
            report['cable_heading_known'] = False
            report['last_known_cable_heading_degrees'] = cable_heading
            report['interrupted_turn_degrees'] = requested_degrees
            try:
                node.stop()
                stopped_at = time.monotonic()
                node.spin_until(lambda: node.yaw_at is not None and node.yaw_at > stopped_at,
                                2.5, 'stopped odometry is unavailable')
                if same_motor_references(turn_start_status, node.robot):
                    delta = math.degrees(math.atan2(math.sin(node.yaw-turn_start_yaw), math.cos(node.yaw-turn_start_yaw)))
                    cable_heading = validate_measured_cable_heading(cable_heading-delta, args.cable_limit_degrees, args.cable_margin_degrees)
                    report['cable_heading_known'] = True
                    report.setdefault('events', []).append('interrupted_turn_position_recovered')
            except Exception as recovery_error:
                report['heading_recovery_error'] = str(recovery_error)
            checkpoint['after'] = dict((name, dict((node.robot or {}).get('motors', {}).get(name, {})))
                                       for name in ('left', 'right'))
            if (str(turn_error) != 'stopped status was not confirmed'
                    or not report['cable_heading_known']):
                raise
            # The retry above already confirmed stopped feedback and recovered
            # this turn's measured position. Continue this scan step instead of
            # discarding a successful recovery and restarting the whole scan.
            actual = -delta
            cable_heading = checkpoint['initial_cable_heading_degrees']
            report.setdefault('stop_confirmation_recoveries', []).append({
                'requested_degrees': requested_degrees,
                'actual_degrees': actual,
                'continued_same_step': True,
            })
        checkpoint['after'] = dict((name, dict((node.robot or {}).get('motors', {}).get(name, {})))
                                   for name in ('left', 'right'))
        try:
            cable_heading = record_cable_turn(
                cable_heading,
                requested_degrees,
                actual,
                args.cable_limit_degrees,
                args.maximum_turn_error_degrees,
                args.cable_margin_degrees,
            )
        except Exception:
            report['last_known_cable_heading_degrees'] = cable_heading
            report['interrupted_turn_degrees'] = requested_degrees
            report['measured_turn_degrees'] = actual
            # A precision miss does not erase a same-reference stopped position.
            try:
                cable_heading = validate_measured_cable_heading(cable_heading + actual,
                    args.cable_limit_degrees, args.cable_margin_degrees)
                report['cable_heading_known'] = True
            except Exception:
                report['cable_heading_known'] = False
            raise
        turn_report = {
            "requested_degrees": requested_degrees,
            "actual_degrees": actual,
            "turn_error_degrees": actual - requested_degrees,
            "cable_heading_degrees": cable_heading,
            "cable_reserve_used": abs(cable_heading) > args.cable_limit_degrees - args.cable_margin_degrees,
        }
        if reason:
            turn_report["reason"] = reason
        report["turns"].append(turn_report)
        report["observed_headings"].append(cable_heading)
        return actual

    try:
        if args.unwind_only:
            if not initial_cable_heading:
                raise ScanError("unwind-only requires the current cable heading")
            node.spin_until(
                node.motion_ready, 5.0, "motion state is unavailable for unwind"
            )
            node.stop()
            guarded_turn(
                -initial_cable_heading,
                reason="camera-independent_cable_unwind",
                camera_required=False,
            )
            report["outcome"] = "cable_unwound"
            return report
        node.prepare()
        if initial_cable_heading and not args.preserve_initial_heading:
            guarded_turn(-initial_cable_heading, reason="initial_unwind")

        def target_seen_at(heading):
            if args.center_target:
                for attempt in range(7):
                    if not node.target_is_visible() and not node.wait_for_tracked_face():
                        raise ScanError('Target disappeared before centering')
                    step = target_centering_step(node.target)
                    if step == 0:
                        report['target_centered'] = True
                        break
                    if attempt == 6:
                        raise ScanError('Target could not be centered in the bounded turns')
                    before_sequence, before_frame = node.frame_sequence, node.frame_fingerprint
                    guarded_turn(step, reason='center_confirmed_target')
                    node.verify_head_view_change(before_sequence, before_frame)
                    if not node.wait_for_target(args.up_dwell_seconds) and not node.wait_for_tracked_face():
                        raise ScanError('Target lost while centering')
            for _ in range(2):
                if args.range_lower_view:break
                if node.target.get('clothing_approach_enabled') and node.target.get('identity_source')!='face':
                    break
                target = face_tilt_target(node.target, node.head_status)
                if abs(target - int(node.head_status['position'])) <= 2:
                    break
                node.look_at_search_position(target)
                if not node.wait_for_target(1.5) and not node.wait_for_tracked_face():
                    raise ScanError('Target lost while improving face framing')
            if node.target.get('clothing_approach_enabled'):
                # Let the long-lived observer finish range work in this view.
                # No face refresh is required and cloud work cannot block Stop.
                deadline=time.monotonic()+7.
                while time.monotonic()<deadline:
                    if node.target_is_visible():
                        decision=approach_decision(node.target)
                        if decision['action'] in ('approach','arrived','arrived_estimate','close','look_lower'):
                            break
                    if not node.safety_ready():
                        raise ScanError(node.safety_reason() or 'View unavailable during ranging')
                    rclpy.spin_once(node,timeout_sec=.05)
            if not node.target_is_visible():
                report.setdefault('events',[]).append('target_lost_during_view_check')
                return False
            report['target_head_position'] = int(node.head_status['position'])
            report['target_head_reference'] = node.head_status['reference_id']
            report["target_heading_degrees"] = cable_heading - scan_origin_heading
            report["target_relative_heading_degrees"] = (
                cable_heading - scan_origin_heading
            )
            report["target_observation"] = dict(node.target)
            report["outcome"] = "target_found"
            return True

        def observe_heading(heading):
            if node.wait_for_target():
                return target_seen_at(heading)
            if args.try_up and node.body_is_visible():
                report.setdefault("body_guided_tilts", []).append(
                    {"heading_degrees": heading, "body": dict(node.body)}
                )
                if node.seek_target_upward():
                    return target_seen_at(heading)
                node.look_forward()
            elif args.search_up:
                report.setdefault("high_view_checks", []).append(heading)
                if node.seek_target_upward():
                    return target_seen_at(heading)
                if node.body_is_visible():
                    report.setdefault("body_guided_tilts", []).append(
                        {"heading_degrees": heading, "body": dict(node.body)}
                    )
                    if node.seek_target_upward():
                        return target_seen_at(heading)
                node.look_forward()
            return False

        if args.fast_search and not args.vertical_only:
            positions=([int(node.head_status['down_position'])] if args.range_lower_view else
                       preferred_search_positions(node.head_status,args.preferred_head_position,args.preferred_head_reference))
            for position in positions:
                node.look_at_search_position(position)
                report.setdefault('search_passes', []).append({'position': position, 'headings': []})
                visited = set()
                for heading in ([0.] if args.range_lower_view else headings):
                    requested_turn = scan_origin_heading + heading - cable_heading
                    if abs(requested_turn) > args.yaw_tolerance_degrees:
                        guarded_turn(requested_turn)
                    if heading in visited:
                        continue
                    visited.add(heading)
                    report['search_passes'][-1]['headings'].append(heading)
                    if node.wait_for_target() or node.wait_for_tracked_face():
                        if target_seen_at(heading):return report
                    scene = None
                    if not args.range_lower_view and not node.body_is_visible() and not node.face_is_visible():
                        scene = node.assess_search_scene()
                        if node.target_is_visible():
                            if target_seen_at(heading):return report
                    cloud_candidate = bool(human_framing_plan(scene, node.head_status))
                    if args.try_up and not args.range_lower_view and (node.body_is_visible() or node.face_is_visible() or cloud_candidate):
                        report.setdefault('body_guided_tilts', []).append(
                            {'heading_degrees': heading, 'body': dict(node.body or {})})
                        # Inspect body/face at this heading before chasing its
                        # horizontal position. A partial profile remains valid.
                        if node.investigate_person(initial_advice=scene):
                            if target_seen_at(heading):return report
                        if (not node.face_is_visible()
                                and (not node.body_is_visible() or small_lower_person_candidate(node.body))):
                            node.look_at_search_position(position)
                            continue
                        # Keep this person as the candidate. Missing identity
                        # is not evidence that we should turn past them.
                        for attempt in range(7):
                            if not (node.body_is_visible() or node.face_is_visible()):
                                node.spin_until(lambda: node.body_is_visible() or node.face_is_visible(),
                                                1.5, 'Person lost while aligning')
                            candidate = node.face if node.face_is_visible() else node.body
                            correction = target_centering_step(candidate)
                            if correction == 0:
                                break
                            if attempt == 6:
                                raise ScanError('Person could not be centered in the bounded turns')
                            guarded_turn(correction, reason='center_person_candidate')
                            node.wait_for_target(.8)
                        if node.investigate_person():
                            if target_seen_at(heading):return report
                        report['outcome'] = 'person_found_unidentified'
                        report['person_observation'] = node.body
                        report['target_head_position'] = int(node.head_status['position'])
                        report['target_head_reference'] = node.head_status['reference_id']
                        report.setdefault('events', []).extend(['person_seen', 'identity_not_confirmed'])
                        if args.continue_past_unidentified:
                            report.setdefault('unidentified_candidates', []).append(dict(heading=cable_heading, observation=node.body))
                            report['outcome'] = 'failure'  # Incomplete if the next view fails.
                            node.look_at_search_position(position)
                            continue
                        report['events'].append('stopped_with_person_candidate')
                        return report
            report['outcome'] = 'scan_complete_no_target'
            return report

        if observe_heading(0):
            return report
        if args.vertical_only:
            report["outcome"] = "vertical_scan_complete_no_target"
            return report
        for target_heading, turn in zip(headings[1:], incremental_scan_turns(headings)):
            guarded_turn(turn)
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
        report["final_cable_heading_degrees"] = (round(cable_heading, 3)
            if report.get("cable_heading_known", True) else None)
        report["finished_at_unix"] = time.time()
        if node.scene_future is not None:
            node.scene_future.cancel()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return report


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile-id')
    parser.add_argument('--profile-revision')
    parser.add_argument('--range-lower-view',action='store_true',help='Inspect the saved lower view for metric person range')
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--candidate-seconds", type=float, default=12.0)
    parser.add_argument("--continue-past-unidentified", action="store_true")
    parser.add_argument("--center-target", action="store_true",
                        help="center the confirmed target using small chassis turns")
    parser.add_argument("--step-degrees", type=int, default=30)
    parser.add_argument("--sweep-limit-degrees", type=int, default=90)
    parser.add_argument(
        "--first-direction", choices=("right", "left"), default="right"
    )
    parser.add_argument("--initial-cable-heading-degrees", type=float, default=0.0)
    parser.add_argument("--cable-limit-degrees", type=float, default=120.0)
    parser.add_argument("--cable-margin-degrees", type=float, default=5.0)
    parser.add_argument(
        "--cable-zero-confirmed",
        action="store_true",
        help="confirm this run belongs to a mission started at marked cable neutral",
    )
    parser.add_argument("--maximum-turn-error-degrees", type=float, default=25.0)
    parser.add_argument(
        "--preserve-initial-heading",
        action="store_true",
        help="scan around the current tether heading and return to it",
    )
    parser.add_argument("--unwind-only", action="store_true")
    parser.add_argument(
        "--try-up",
        action="store_true",
        help="tilt upward in steps when a person body is seen without a face",
    )
    parser.add_argument(
        "--search-up",
        action="store_true", default=True,
        help="search higher views even when the initial person detection misses",
    )
    parser.add_argument("--no-search-up", dest="search_up", action="store_false",
                        help="inspect only the current forward height at each heading")
    parser.add_argument("--high-search-position", type=int, default=-90)
    parser.add_argument("--dwell-seconds", type=float, default=1.0)
    parser.add_argument('--fast-search', action='store_true',
                        help='scan horizontally first; change height only for a person or another room pass')
    parser.add_argument("--up-dwell-seconds", type=float, default=3.0)
    parser.add_argument("--visual-settle-seconds", type=float, default=1.5)
    parser.add_argument("--visual-hold-seconds", type=float, default=0.75)
    parser.add_argument("--search-view-url", default="http://127.0.0.1:18091/search-view")
    parser.add_argument('--search-view-timeout-seconds', type=float, default=22.,
                        help='Bounded cloud-view wait; local detection and Stop keep running')
    parser.add_argument("--camera-recovery-seconds", type=float, default=30.0)
    parser.add_argument("--tilt-step-degrees", type=int, default=5)
    parser.add_argument('--preferred-head-position', type=int)
    parser.add_argument('--preferred-head-reference')
    parser.add_argument("--face-search-position", type=int, default=-120)
    parser.add_argument(
        "--vertical-only",
        action="store_true",
        help="search camera height at the current chassis heading without rotating",
    )
    parser.add_argument("--turn-speed", type=float, default=0.60)
    parser.add_argument("--turn-timeout", type=float, default=8.0)
    parser.add_argument("--yaw-tolerance-degrees", type=float, default=3.0)
    parser.add_argument("--report", default="/home/animesh/echora/logs/latest_scan.json")
    args = parser.parse_args(argv)
    if not math.isfinite(args.candidate_seconds) or not 0 < args.candidate_seconds <= 60:
        parser.error("candidate seconds must be in (0, 60]")
    if args.fast_search:
        args.visual_settle_seconds = .25
        args.visual_hold_seconds = .1
        args.up_dwell_seconds = .8
        args.tilt_step_degrees = 15
    try:
        validate_measured_cable_heading(
            args.initial_cable_heading_degrees,
            args.cable_limit_degrees,
            args.cable_margin_degrees,
        )
    except Exception as exc:
        parser.error(str(exc))
    if args.center_target and args.vertical_only:
        parser.error("target centering requires chassis turns")
    if not 4 <= args.tilt_step_degrees <= 15:
        parser.error("camera search steps must be between 4 and 15 counts")
    if not math.isfinite(args.turn_speed) or not 0 < args.turn_speed <= 0.6:
        parser.error("scan turn speed must be positive and at most 0.6 radians per second")
    if not math.isfinite(args.turn_timeout) or args.turn_timeout <= 0:
        parser.error("scan turn timeout must be positive and finite")
    if not math.isfinite(args.maximum_turn_error_degrees) or args.maximum_turn_error_degrees < 0:
        parser.error("maximum turn error cannot be negative")
    if not math.isfinite(args.search_view_timeout_seconds) or not 1 <= args.search_view_timeout_seconds <= 30:
        parser.error('search-view timeout must be between 1 and 30 seconds')
    if (
        args.visual_settle_seconds <= 0
        or args.visual_hold_seconds <= 0
        or args.camera_recovery_seconds <= 0
    ):
        parser.error("visual settle, hold, and camera recovery times must be positive")
    if (
        args.execute
        and not args.vertical_only
        and not args.unwind_only
        and not args.cable_zero_confirmed
    ):
        parser.error("chassis scan requires --cable-zero-confirmed")
    return args


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
