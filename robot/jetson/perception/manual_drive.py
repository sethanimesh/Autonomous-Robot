"""Fail-closed state for the browser's short, dead-man drive controls."""

import math
import secrets
import threading
import time


class ManualDriveError(RuntimeError):
    """Raised when a browser drive command is stale or unsafe."""


def normalize_angle(angle):
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


def yaw_from_quaternion(x, y, z, w):
    """Return ROS yaw from a quaternion without another dependency."""

    sin_yaw = 2.0 * (float(w) * float(z) + float(x) * float(y))
    cos_yaw = 1.0 - 2.0 * (float(y) ** 2 + float(z) ** 2)
    return math.atan2(sin_yaw, cos_yaw)


class ManualDriveController(object):
    """Authorize low-speed bursts and account for tether rotation from odometry."""

    DIRECTIONS = ("forward", "back", "left", "right", "stop")

    def __init__(
        self,
        linear_speed=0.03,
        reverse_speed=0.02,
        angular_speed=0.25,
        cable_limit_degrees=90.0,
        cable_margin_degrees=10.0,
        turn_buffer_degrees=3.0,
        command_timeout_seconds=0.25,
        session_timeout_seconds=120.0,
        maximum_odometry_age_seconds=1.5,
        clock=None,
        token_factory=None,
    ):
        values = (
            linear_speed,
            reverse_speed,
            angular_speed,
            cable_limit_degrees,
            command_timeout_seconds,
            session_timeout_seconds,
            maximum_odometry_age_seconds,
        )
        if any(not math.isfinite(float(value)) or float(value) <= 0 for value in values):
            raise ValueError("manual-drive speeds, limits, and timeouts must be positive")
        if cable_margin_degrees < 0 or cable_margin_degrees >= cable_limit_degrees:
            raise ValueError("manual-drive cable margin is invalid")
        usable_limit = float(cable_limit_degrees) - float(cable_margin_degrees)
        if turn_buffer_degrees < 0 or turn_buffer_degrees >= usable_limit:
            raise ValueError("manual-drive turn buffer is invalid")
        self.linear_speed = float(linear_speed)
        self.reverse_speed = float(reverse_speed)
        self.angular_speed = float(angular_speed)
        self.cable_limit_degrees = float(cable_limit_degrees)
        self.cable_margin_degrees = float(cable_margin_degrees)
        self.turn_buffer_degrees = float(turn_buffer_degrees)
        self.command_timeout_seconds = float(command_timeout_seconds)
        self.session_timeout_seconds = float(session_timeout_seconds)
        self.maximum_odometry_age_seconds = float(maximum_odometry_age_seconds)
        self.clock = clock or time.monotonic
        self.token_factory = token_factory or (lambda: secrets.token_urlsafe(24))
        self.lock = threading.RLock()
        self.latest_yaw = None
        self.latest_odometry_at = None
        self.enabled = False
        self.token = None
        self.baseline_yaw = None
        self.last_sequence = -1
        self.active_direction = None
        self.active_since = None
        self.last_command_at = None
        self.last_activity_at = None
        self.message = "Driving is locked. Confirm cable-neutral before enabling."

    @property
    def usable_cable_limit_degrees(self):
        return self.cable_limit_degrees - self.cable_margin_degrees

    def update_odometry(self, yaw, observed_at=None):
        yaw = float(yaw)
        if not math.isfinite(yaw):
            return
        with self.lock:
            self.latest_yaw = normalize_angle(yaw)
            self.latest_odometry_at = (
                self.clock() if observed_at is None else float(observed_at)
            )

    def _odometry_ready_locked(self, now):
        return bool(
            self.latest_yaw is not None
            and self.latest_odometry_at is not None
            and now - self.latest_odometry_at
            <= self.maximum_odometry_age_seconds
        )

    def _heading_locked(self):
        if self.baseline_yaw is None or self.latest_yaw is None:
            return None
        # ROS yaw is positive left; tether/image heading is positive right.
        return -math.degrees(normalize_angle(self.latest_yaw - self.baseline_yaw))

    def _clear_motion_locked(self):
        self.active_direction = None
        self.active_since = None
        self.last_command_at = None

    def _snapshot_locked(self, now=None):
        now = self.clock() if now is None else float(now)
        heading = self._heading_locked()
        return {
            "enabled": self.enabled,
            "active": self.active_direction is not None,
            "direction": self.active_direction,
            "message": self.message,
            "cable_heading_degrees": (
                None if heading is None else round(heading, 1)
            ),
            "usable_cable_limit_degrees": self.usable_cable_limit_degrees,
            "odometry_ready": self._odometry_ready_locked(now),
        }

    def status(self):
        with self.lock:
            return self._snapshot_locked()

    def enable(
        self,
        cable_zero_confirmed,
        camera_ready,
        robot_ready,
        robot_moving,
        mission_running,
        enrollment_running=False,
    ):
        now = self.clock()
        with self.lock:
            if self.enabled:
                raise ManualDriveError(
                    "Disable the existing manual-drive session before re-enabling."
                )
            if cable_zero_confirmed is not True:
                raise ManualDriveError(
                    "Place the robot at the marked cable-neutral heading and "
                    "confirm the cable is clear first."
                )
            if mission_running or enrollment_running:
                raise ManualDriveError("Stop the current mission or enrollment first.")
            if not camera_ready:
                raise ManualDriveError("The live camera must be fresh before driving.")
            if not robot_ready:
                raise ManualDriveError("Fresh EV3 status is required before driving.")
            if robot_moving:
                raise ManualDriveError("Wait for the robot to stop before enabling driving.")
            if not self._odometry_ready_locked(now):
                raise ManualDriveError("Fresh wheel odometry is required before driving.")
            self.enabled = True
            self.token = str(self.token_factory())
            self.baseline_yaw = self.latest_yaw
            self.last_sequence = -1
            self._clear_motion_locked()
            self.last_activity_at = now
            self.message = "Driving enabled. Hold a direction; release to stop."
            result = self._snapshot_locked(now)
            result["control_token"] = self.token
            return result

    def _fail_motion_locked(self, message):
        self._clear_motion_locked()
        self.message = message
        raise ManualDriveError(message)

    def command(
        self,
        direction,
        token,
        sequence,
        camera_ready,
        robot_ready,
        mission_running,
    ):
        now = self.clock()
        direction = str(direction or "").strip().lower()
        if direction not in self.DIRECTIONS:
            raise ManualDriveError("Unknown drive direction.")
        if isinstance(sequence, bool) or not isinstance(sequence, int):
            raise ManualDriveError("Drive command sequence is invalid.")
        with self.lock:
            if not self.enabled or not self.token:
                raise ManualDriveError("Enable manual driving first.")
            if not isinstance(token, str) or not secrets.compare_digest(
                token, self.token
            ):
                raise ManualDriveError("This drive control session is no longer valid.")
            if sequence <= self.last_sequence:
                raise ManualDriveError("A stale drive command was rejected.")
            self.last_sequence = sequence
            self.last_activity_at = now
            if direction == "stop":
                self._clear_motion_locked()
                self.message = "Manual drive stopped."
                return dict(self._snapshot_locked(now), linear=0.0, angular=0.0)
            if mission_running:
                self._fail_motion_locked("Manual drive stopped because a mission is active.")
            if not camera_ready:
                self._fail_motion_locked("Manual drive stopped because the camera is stale.")
            if not robot_ready:
                self._fail_motion_locked("Manual drive stopped because EV3 status is stale.")
            if not self._odometry_ready_locked(now):
                self._fail_motion_locked("Manual drive stopped because odometry is stale.")
            if (
                self.active_direction is not None
                and self.active_direction != direction
            ):
                self._fail_motion_locked(
                    "Release the current direction before choosing another."
                )
            heading = self._heading_locked()
            turn_cutoff = (
                self.usable_cable_limit_degrees - self.turn_buffer_degrees
            )
            if direction == "left" and heading <= -turn_cutoff:
                self._fail_motion_locked(
                    "Left turn blocked at the tether safety limit. Turn right."
                )
            if direction == "right" and heading >= turn_cutoff:
                self._fail_motion_locked(
                    "Right turn blocked at the tether safety limit. Turn left."
                )
            if self.active_since is None:
                self.active_since = now
            self.active_direction = direction
            self.last_command_at = now
            twists = {
                "forward": (self.linear_speed, 0.0),
                "back": (-self.reverse_speed, 0.0),
                "left": (0.0, self.angular_speed),
                "right": (0.0, -self.angular_speed),
            }
            linear, angular = twists[direction]
            self.message = "Moving {0}; release to stop.".format(direction)
            return dict(
                self._snapshot_locked(now), linear=linear, angular=angular
            )

    def stop(self, disable=False, message=None):
        now = self.clock()
        with self.lock:
            self._clear_motion_locked()
            if disable:
                self.enabled = False
                self.token = None
                self.baseline_yaw = None
                self.last_sequence = -1
            self.last_activity_at = now
            self.message = message or (
                "Driving disabled."
                if disable
                else "Manual drive stopped."
            )
            return self._snapshot_locked(now)

    def watchdog(self, camera_ready, robot_ready, mission_running):
        """Return True once when the node must publish a zero velocity."""

        now = self.clock()
        with self.lock:
            should_stop = False
            if self.enabled and self.last_activity_at is not None and (
                now - self.last_activity_at >= self.session_timeout_seconds
            ):
                should_stop = self.active_direction is not None
                self.enabled = False
                self.token = None
                self.baseline_yaw = None
                self.last_sequence = -1
                self._clear_motion_locked()
                self.message = "Driving disabled after inactivity."
                return should_stop
            if self.active_direction is None:
                return False
            if mission_running or not camera_ready or not robot_ready:
                should_stop = True
                reason = (
                    "mission started"
                    if mission_running
                    else "camera became stale"
                    if not camera_ready
                    else "EV3 status became stale"
                )
                self.message = "Manual drive stopped: {0}.".format(reason)
            elif not self._odometry_ready_locked(now):
                should_stop = True
                self.message = "Manual drive stopped: odometry became stale."
            elif self.last_command_at is None or (
                now - self.last_command_at >= self.command_timeout_seconds
            ):
                should_stop = True
                self.message = "Manual drive stopped when the control was released."
            if should_stop:
                self._clear_motion_locked()
            return should_stop
