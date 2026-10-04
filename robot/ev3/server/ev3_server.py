#!/usr/bin/env python3
"""Small newline-delimited JSON control service for an ev3dev brick.

The module intentionally uses only the Python 3.5 standard library so it can run
on the project's existing ev3dev-stretch installation.
"""

from __future__ import print_function

import argparse
import json
import math
import os
import socket
import sys
import time
import uuid


PROTOCOL_VERSION = 6
DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 9999
DEFAULT_ALLOWED_CLIENT = "192.168.1.48"
DEFAULT_WATCHDOG_SECONDS = 0.5
DEFAULT_CLIENT_IDLE_TIMEOUT_SECONDS = 5.0
DEFAULT_DRIVE_SPEED_LIMIT = 250
DEFAULT_TOOL_SPEED_LIMIT = 1500
DEFAULT_TOOL_POSITION_LIMIT = 180
DEFAULT_TOOL_STEP_LIMIT = 15
DEFAULT_TOOL_MOVE_TIMEOUT_SECONDS = 8.0
DEFAULT_TOOL_STALL_RETRY_SECONDS = 1.0
DEFAULT_TOOL_STALL_RETRY_LIMIT = 1
DEFAULT_TOOL_PROGRESS_COUNTS = 2
TOOL_SETTLE_TARGET_COUNTS = 4
TOOL_SETTLE_POSITION_COUNTS = 1
TOOL_SETTLE_SPEED_COUNTS_PER_SECOND = 5
TOOL_SETTLE_SECONDS = 0.4
DEFAULT_TOOL_HOME_SPEED_LIMIT = 300
DEFAULT_TOOL_HOME_TIMEOUT_SECONDS = 8.0
DEFAULT_TOOL_HOME_NO_PROGRESS_SECONDS = 0.4
DEFAULT_POLL_SECONDS = 0.05
DEFAULT_MAX_LINE_BYTES = 4096

MOTOR_PORTS = {
    "left": "outB",
    "right": "outC",
    "tool": "outA",
}


class ProtocolError(Exception):
    """Raised when a client request is invalid."""


class HardwareError(Exception):
    """Raised when a required EV3 hardware operation fails."""


class SysfsMotor(object):
    """Minimal tacho-motor adapter for ev3dev's sysfs interface."""

    def __init__(self, port, sysfs_root="/sys/class/tacho-motor"):
        self.port = port
        self.sysfs_root = sysfs_root
        self.path = self._find_motor(sysfs_root, port)
        self.generation = uuid.uuid4().hex
        self.path_inode = os.stat(self.path).st_ino

    def _ensure_path(self):
        # ev3dev may remove and recreate a motor with a new motorN name after a
        # driver reset. Resolve the output port again instead of requiring the
        # whole service to be restarted with the stale sysfs path.
        if not os.path.isdir(self.path) or os.stat(self.path).st_ino != self.path_inode:
            self.path = self._find_motor(self.sysfs_root, self.port)
            self.path_inode = os.stat(self.path).st_ino
            self.generation = uuid.uuid4().hex

    @staticmethod
    def _find_motor(sysfs_root, port):
        try:
            entries = os.listdir(sysfs_root)
        except OSError as exc:
            raise HardwareError("cannot list motors: {0}".format(exc))

        for entry in entries:
            path = os.path.join(sysfs_root, entry)
            address_path = os.path.join(path, "address")
            try:
                with open(address_path, "r") as address_file:
                    address = address_file.read().strip()
            except (IOError, OSError):
                continue
            if address.endswith(":" + port) or address == port:
                return path

        raise HardwareError("required motor not found on {0}".format(port))

    def _write(self, attribute, value):
        self._ensure_path()
        path = os.path.join(self.path, attribute)
        try:
            with open(path, "w") as attribute_file:
                attribute_file.write(str(value))
        except (IOError, OSError) as exc:
            raise HardwareError(
                "failed to write {0} for {1}: {2}".format(attribute, self.port, exc)
            )

    def _read(self, attribute):
        self._ensure_path()
        path = os.path.join(self.path, attribute)
        try:
            with open(path, "r") as attribute_file:
                return attribute_file.read().strip()
        except (IOError, OSError) as exc:
            raise HardwareError(
                "failed to read {0} for {1}: {2}".format(attribute, self.port, exc)
            )

    def reference_identity(self):
        self._ensure_path()
        with open("/proc/sys/kernel/random/boot_id") as handle:
            boot_id = handle.read().strip()
        return {"boot_id": boot_id, "device": os.path.realpath(self.path),
                "inode": os.stat(self.path).st_ino, "port": self.port}

    def set_speed(self, speed):
        if speed == 0:
            self.stop()
            return
        self._write("speed_sp", speed)
        self._write("command", "run-forever")

    def move_to(self, position, speed):
        self._write("stop_action", "hold")
        self._write("position_sp", int(position))
        self._write("speed_sp", abs(int(speed)))
        self._write("command", "run-to-abs-pos")

    def set_position(self, position):
        self._write("position", int(position))

    def stop(self):
        self._write("stop_action", "brake")
        self._write("command", "stop")

    def hold(self):
        self._write("stop_action", "hold")
        self._write("command", "stop")

    def snapshot(self):
        state = self._read("state")
        return {
            "port": self.port,
            "position": int(self._read("position")),
            "speed": int(self._read("speed")),
            "state": state.split() if state else [],
            "generation": self.generation,
        }


class MotorController(object):
    """Owns motor state, speed limits, and the stale-command watchdog."""

    def __init__(
        self,
        motors,
        watchdog_seconds=DEFAULT_WATCHDOG_SECONDS,
        drive_speed_limit=DEFAULT_DRIVE_SPEED_LIMIT,
        tool_speed_limit=DEFAULT_TOOL_SPEED_LIMIT,
        tool_position_limit=DEFAULT_TOOL_POSITION_LIMIT,
        tool_step_limit=DEFAULT_TOOL_STEP_LIMIT,
        tool_move_timeout_seconds=DEFAULT_TOOL_MOVE_TIMEOUT_SECONDS,
        tool_stall_retry_seconds=DEFAULT_TOOL_STALL_RETRY_SECONDS,
        tool_stall_retry_limit=DEFAULT_TOOL_STALL_RETRY_LIMIT,
        tool_progress_counts=DEFAULT_TOOL_PROGRESS_COUNTS,
        tool_home_speed_limit=DEFAULT_TOOL_HOME_SPEED_LIMIT,
        tool_home_timeout_seconds=DEFAULT_TOOL_HOME_TIMEOUT_SECONDS,
        tool_home_no_progress_seconds=DEFAULT_TOOL_HOME_NO_PROGRESS_SECONDS,
        clock=None,
        reference_path=None,
    ):
        missing = sorted(set(MOTOR_PORTS) - set(motors))
        if missing:
            raise HardwareError("missing motor roles: {0}".format(", ".join(missing)))
        if watchdog_seconds <= 0:
            raise ValueError("watchdog_seconds must be positive")

        self.motors = motors
        self.watchdog_seconds = float(watchdog_seconds)
        self.drive_speed_limit = int(drive_speed_limit)
        self.tool_speed_limit = int(tool_speed_limit)
        self.tool_position_limit = int(tool_position_limit)
        self.tool_step_limit = int(tool_step_limit)
        self.tool_move_timeout_seconds = float(tool_move_timeout_seconds)
        self.tool_stall_retry_seconds = float(tool_stall_retry_seconds)
        self.tool_stall_retry_limit = int(tool_stall_retry_limit)
        self.tool_progress_counts = int(tool_progress_counts)
        self.tool_home_speed_limit = int(tool_home_speed_limit)
        self.tool_home_timeout_seconds = float(tool_home_timeout_seconds)
        self.tool_home_no_progress_seconds = float(tool_home_no_progress_seconds)
        if self.tool_position_limit <= 0:
            raise ValueError("tool_position_limit must be positive")
        if self.tool_step_limit <= 0 or self.tool_step_limit > self.tool_position_limit:
            raise ValueError("tool_step_limit must be positive and within position limit")
        if self.tool_move_timeout_seconds <= 0:
            raise ValueError("tool_move_timeout_seconds must be positive")
        if self.tool_stall_retry_seconds <= 0:
            raise ValueError("tool_stall_retry_seconds must be positive")
        if self.tool_stall_retry_limit < 0:
            raise ValueError("tool_stall_retry_limit cannot be negative")
        if self.tool_progress_counts <= 0:
            raise ValueError("tool_progress_counts must be positive")
        if self.tool_home_speed_limit <= 0:
            raise ValueError("tool_home_speed_limit must be positive")
        if self.tool_home_timeout_seconds <= 0:
            raise ValueError("tool_home_timeout_seconds must be positive")
        if self.tool_home_no_progress_seconds <= 0:
            raise ValueError("tool_home_no_progress_seconds must be positive")
        self.clock = clock or time.monotonic
        self.commanded = {"left": 0, "right": 0, "tool": 0}
        self.last_motion_at = None
        self.tool_move_started_at = None
        self.tool_target_position = None
        self.tool_homing = False
        self.tool_homed = False
        self.tool_reference_id = uuid.uuid4().hex
        self.tool_device_generation = getattr(self.motors["tool"], "generation", None)
        self.tool_last_position = None
        self.tool_last_progress_at = None
        self._reset_tool_settlement()
        self.tool_stall_retry_count = 0
        self.track_motion_active = False
        self.tool_motion_active = False
        self.motion_active = False
        self.last_stop_reason = None
        self.reference_path = reference_path
        self.stop_all("startup")
        self._restore_reference()

    def _restore_reference(self):
        if not self.reference_path:
            return
        try:
            with open(self.reference_path) as handle:
                saved = json.load(handle)
            identity = self.motors["tool"].reference_identity()
            reference = saved.get("reference_id")
            if (saved.get("identity") == identity and saved.get("homed") is True
                    and isinstance(reference, str) and len(reference) == 32
                    and all(c in "0123456789abcdef" for c in reference)):
                self.tool_reference_id = reference
                self.tool_homed = True
                self.tool_target_position = int(self.motors["tool"].snapshot()["position"])
                self.last_stop_reason = "retained-reference-restored"
        except (OSError, ValueError, TypeError, AttributeError):
            # Missing, corrupt, or incompatible state never enables movement.
            return

    def _save_reference(self):
        if not self.reference_path:
            return
        saved = {"identity": self.motors["tool"].reference_identity(),
                 "reference_id": self.tool_reference_id, "homed": self.tool_homed}
        temporary = self.reference_path + ".tmp"
        try:
            with open(temporary, "w") as handle:
                os.chmod(temporary, 0o600)
                json.dump(saved, handle)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary, self.reference_path)
        except OSError as exc:
            raise HardwareError("cannot persist camera reference: {0}".format(exc))

    @staticmethod
    def _normalize_speed(name, value, limit):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ProtocolError("{0} speed must be a number".format(name))
        if not math.isfinite(value):
            raise ProtocolError("{0} speed must be finite".format(name))
        integer_value = int(round(value))
        return max(-limit, min(limit, integer_value))

    @staticmethod
    def _normalize_position(name, value, limit):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ProtocolError("{0} must be a number".format(name))
        if not math.isfinite(value):
            raise ProtocolError("{0} must be finite".format(name))
        integer_value = int(round(value))
        if integer_value < -limit or integer_value > limit:
            raise ProtocolError(
                "{0} must be between {1} and {2}".format(name, -limit, limit)
            )
        return integer_value

    def _reset_tool_settlement(self):
        self.tool_settle_started_at = None
        self.tool_settle_position = None

    def _tool_position_settled(self, snapshot, now):
        position = int(snapshot["position"])
        states = snapshot.get("state", [])
        near_stopped_target = (
            self.tool_target_position is not None
            and abs(position - self.tool_target_position) <= TOOL_SETTLE_TARGET_COUNTS
            and "running" not in states
            and "stalled" not in states
            and abs(float(snapshot.get("speed", float("inf"))))
            <= TOOL_SETTLE_SPEED_COUNTS_PER_SECOND
        )
        if not near_stopped_target:
            self._reset_tool_settlement()
            return False
        if (
            self.tool_settle_started_at is None
            or abs(position - self.tool_settle_position) > TOOL_SETTLE_POSITION_COUNTS
        ):
            self.tool_settle_started_at = now
            self.tool_settle_position = position
            return False
        return now - self.tool_settle_started_at >= TOOL_SETTLE_SECONDS

    def _refresh_motion_flags(self, tool_snapshot=None):
        if tool_snapshot is None:
            tool_snapshot = self.motors["tool"].snapshot()
        generation = tool_snapshot.get("generation")
        if generation != self.tool_device_generation:
            self.tool_device_generation = generation
            self.tool_homed = False
            self.tool_reference_id = uuid.uuid4().hex
            self._save_reference()
            self.stop_all("tool-reference-lost", suppress_errors=True)
        if self.tool_motion_active:
            if tool_snapshot is None:
                tool_snapshot = self.motors["tool"].snapshot()
            if self.tool_homing:
                now = self.clock()
                position = int(tool_snapshot["position"])
                if (
                    self.tool_last_position is None
                    or abs(position - self.tool_last_position) >= 1
                ):
                    self.tool_last_position = position
                    self.tool_last_progress_at = now
                no_progress = (
                    self.tool_last_progress_at is not None
                    and now - self.tool_last_progress_at
                    >= self.tool_home_no_progress_seconds
                )
                if "stalled" in tool_snapshot.get("state", []) or no_progress:
                    # Never release this loaded mechanism between finding the
                    # stop and establishing zero.  A normal brake allowed the
                    # real head to back-drive several counts in that tiny gap.
                    self.motors["tool"].hold()
                    self.motors["tool"].set_position(0)
                    # Re-issue hold in the new encoder coordinate system.
                    self.motors["tool"].hold()
                    self.tool_motion_active = False
                    self.tool_homing = False
                    self.tool_homed = True
                    self.commanded["tool"] = 0
                    self.tool_move_started_at = None
                    self.tool_target_position = 0
                    self.last_stop_reason = "tool-home-complete"
                    self._save_reference()
            else:
                now = self.clock()
                position = int(tool_snapshot["position"])
                # A first "holding" snapshot can precede actual travel, and
                # the loaded linkage can continue settling afterward. Keep
                # the move active until its position is stable near target.
                if self._tool_position_settled(tool_snapshot, now):
                    self.tool_motion_active = False
                    self.commanded["tool"] = 0
                    self.tool_move_started_at = None
                    self.tool_last_position = None
                    self.tool_last_progress_at = None
                    self._reset_tool_settlement()
                else:
                    if (
                        self.tool_last_position is None
                        or abs(position - self.tool_last_position)
                        >= self.tool_progress_counts
                    ):
                        self.tool_last_position = position
                        self.tool_last_progress_at = now
                    no_progress = (
                        self.tool_last_progress_at is not None
                        and now - self.tool_last_progress_at
                        >= self.tool_stall_retry_seconds
                    )
                    if no_progress and self.tool_settle_started_at is None:
                        if (
                            self.tool_stall_retry_count
                            < self.tool_stall_retry_limit
                            and self.tool_target_position is not None
                        ):
                            target = int(self.tool_target_position)
                            # Negative encoder travel lifts this linkage. Give
                            # that loaded direction the proven near-rated
                            # recovery pulse. A downward retry keeps its
                            # original speed to avoid a gravity-assisted jolt.
                            retry_speed = abs(int(self.commanded["tool"]))
                            if target < position:
                                retry_speed = self.tool_speed_limit
                            retry_speed = max(1, retry_speed)
                            try:
                                self.motors["tool"].move_to(target, retry_speed)
                            except Exception:
                                self.stop_all(
                                    "tool-stall-retry-failure",
                                    suppress_errors=True,
                                )
                                raise
                            self.commanded["tool"] = (
                                retry_speed if target > position else -retry_speed
                            )
                            self.tool_stall_retry_count += 1
                            self._reset_tool_settlement()
                            self.tool_last_position = position
                            self.tool_last_progress_at = now
                            self.last_stop_reason = "tool-stall-retry"
                        else:
                            self.stop_all("tool-stalled-after-retry")
        self.motion_active = self.track_motion_active or self.tool_motion_active

    def drive(self, left, right):
        applied = {
            "left": self._normalize_speed("left", left, self.drive_speed_limit),
            "right": self._normalize_speed("right", right, self.drive_speed_limit),
        }

        if (applied["left"] or applied["right"]) and self.tool_motion_active:
            raise ProtocolError("camera head is moving; chassis command rejected")

        try:
            for role in ("left", "right"):
                self.motors[role].set_speed(applied[role])
        except Exception:
            self.stop_all("motor-write-failure", suppress_errors=True)
            raise

        self.commanded["left"] = applied["left"]
        self.commanded["right"] = applied["right"]
        self.track_motion_active = bool(applied["left"] or applied["right"])

        if self.track_motion_active:
            self.last_motion_at = self.clock()
            self.last_stop_reason = None
        elif not self.tool_motion_active:
            self.last_motion_at = None
            self.last_stop_reason = "zero-command"
        self._refresh_motion_flags()
        return dict(applied)

    def move_tool(self, position, speed):
        self._refresh_motion_flags()
        if self.track_motion_active:
            raise ProtocolError("chassis is moving; camera-head command rejected")
        if not self.tool_homed:
            raise ProtocolError("camera head must be homed or zeroed before moving")
        target = self._normalize_position(
            "tool position", position, self.tool_position_limit
        )
        applied_speed = abs(
            self._normalize_speed("tool", speed, self.tool_speed_limit)
        )
        if applied_speed == 0:
            raise ProtocolError("tool speed must be greater than zero")

        current = int(self.motors["tool"].snapshot()["position"])
        same_target_retry = bool(
            self.tool_motion_active
            and not self.tool_homing
            and self.tool_target_position is not None
            and target == int(self.tool_target_position)
        )
        if abs(target - current) > self.tool_step_limit:
            raise ProtocolError(
                "tool position change must be no more than {0} counts".format(
                    self.tool_step_limit
                )
            )
        if self.tool_motion_active and not same_target_retry:
            raise ProtocolError(
                "camera head is moving; only its current target can be retried"
            )
        try:
            self.motors["left"].stop()
            self.motors["right"].stop()
            if current == target:
                self.motors["tool"].hold()
            else:
                self.motors["tool"].move_to(target, applied_speed)
        except Exception:
            self.stop_all("tool-move-failure", suppress_errors=True)
            raise

        self.commanded["left"] = 0
        self.commanded["right"] = 0
        self.commanded["tool"] = (
            0 if target == current else applied_speed if target > current else -applied_speed
        )
        self.track_motion_active = False
        self.tool_motion_active = True
        now = self.clock()
        self.tool_target_position = target
        if same_target_retry:
            # A repeated UI press re-applies the exact target but cannot reset
            # the absolute eight-second motion deadline indefinitely.
            self.tool_stall_retry_count += 1
            self.last_stop_reason = "tool-manual-retry"
        else:
            self.tool_move_started_at = now
            self.tool_stall_retry_count = 0
            self.last_stop_reason = None
        self.tool_last_position = current
        self.tool_last_progress_at = now
        self._reset_tool_settlement()
        self.tool_homing = False
        self.motion_active = True
        return {"position": target, "speed": applied_speed}

    def home_tool(self, speed=300):
        if self.track_motion_active:
            raise ProtocolError("chassis is moving; camera-head command rejected")
        applied_speed = abs(
            self._normalize_speed("tool home", speed, self.tool_home_speed_limit)
        )
        if applied_speed == 0:
            raise ProtocolError("tool home speed must be greater than zero")
        self.tool_homed = False
        self.tool_reference_id = uuid.uuid4().hex
        self._save_reference()
        try:
            self.motors["left"].stop()
            self.motors["right"].stop()
            snapshot = self.motors["tool"].snapshot()
            self.motors["tool"].set_speed(applied_speed)
        except Exception:
            self.stop_all("tool-home-failure", suppress_errors=True)
            raise
        now = self.clock()
        self.commanded = {"left": 0, "right": 0, "tool": applied_speed}
        self.track_motion_active = False
        self.tool_motion_active = True
        self.tool_homing = True
        self.tool_homed = False
        self.tool_target_position = None
        self.tool_move_started_at = now
        self.tool_last_position = int(snapshot["position"])
        self.tool_last_progress_at = now
        self._reset_tool_settlement()
        self.tool_stall_retry_count = 0
        self.last_stop_reason = None
        self.motion_active = True
        return {"direction": 1, "speed": applied_speed}

    def zero_tool(self):
        self._refresh_motion_flags()
        if self.motion_active:
            raise ProtocolError("all motors must be stopped before zeroing camera head")
        self.tool_homed = False
        self.tool_reference_id = uuid.uuid4().hex
        self._save_reference()
        # The loaded motor may already be actively holding an old encoder
        # target. Freeze it before changing coordinates, then establish a new
        # hold target at zero. Without the second hold, the real mechanism
        # immediately drove back toward the pre-zero numeric target.
        self.motors["tool"].hold()
        self.motors["tool"].set_position(0)
        self.motors["tool"].hold()
        self.tool_target_position = 0
        self.tool_homed = True
        self._save_reference()
        return 0

    def acknowledge_tool_position(self):
        """Trust the retained encoder position after a service restart."""

        self._refresh_motion_flags()
        if self.motion_active:
            raise ProtocolError(
                "all motors must be stopped before acknowledging camera-head position"
            )
        position = int(self.motors["tool"].snapshot()["position"])
        self._normalize_position("tool position", position, self.tool_position_limit)
        self.tool_target_position = position
        self.tool_reference_id = uuid.uuid4().hex
        self.tool_homed = True
        self.last_stop_reason = "tool-position-acknowledged"
        self._save_reference()
        return position

    def stop_all(self, reason, suppress_errors=False, hold_tool=True):
        errors = []
        for role in ("left", "right", "tool"):
            try:
                if role == "tool" and hold_tool:
                    self.motors[role].hold()
                else:
                    self.motors[role].stop()
            except Exception as exc:
                errors.append("{0}: {1}".format(role, exc))

        self.commanded = {"left": 0, "right": 0, "tool": 0}
        self.track_motion_active = False
        self.tool_motion_active = False
        self.tool_move_started_at = None
        self.tool_homing = False
        self.tool_last_position = None
        self.tool_last_progress_at = None
        self._reset_tool_settlement()
        self.motion_active = False
        self.last_motion_at = None
        self.last_stop_reason = reason

        if errors and not suppress_errors:
            raise HardwareError("failed to stop motors: {0}".format("; ".join(errors)))

    def enforce_watchdog(self):
        stopped = False
        if (
            self.track_motion_active
            and self.last_motion_at is not None
            and self.clock() - self.last_motion_at >= self.watchdog_seconds
        ):
            self.stop_all("watchdog")
            return True
        if (
            self.tool_motion_active
            and self.tool_move_started_at is not None
            and self.clock() - self.tool_move_started_at
            >= (
                self.tool_home_timeout_seconds
                if self.tool_homing
                else self.tool_move_timeout_seconds
            )
        ):
            homing = self.tool_homing
            self.stop_all("tool-home-timeout" if homing else "tool-move-timeout")
            return True
        self._refresh_motion_flags()
        return stopped

    def status(self):
        snapshots = {}
        for role in ("left", "right", "tool"):
            snapshots[role] = self.motors[role].snapshot()
        self._refresh_motion_flags(snapshots["tool"])
        for role in ("left", "right", "tool"):
            snapshots[role]["commanded_speed"] = self.commanded[role]
        return {
            "motors": snapshots,
            "motion_active": self.motion_active,
            "last_stop_reason": self.last_stop_reason,
            "watchdog_timeout_ms": int(self.watchdog_seconds * 1000),
            "tool_motion_active": self.tool_motion_active,
            "tool_homing": self.tool_homing,
            "tool_homed": self.tool_homed,
            "tool_reference_id": self.tool_reference_id,
            "tool_target_position": self.tool_target_position,
            "tool_position_limit": self.tool_position_limit,
            "tool_step_limit": self.tool_step_limit,
            "tool_move_timeout_ms": int(self.tool_move_timeout_seconds * 1000),
            "tool_stall_retry_ms": int(self.tool_stall_retry_seconds * 1000),
            "tool_stall_retry_count": self.tool_stall_retry_count,
            "tool_stall_retry_limit": self.tool_stall_retry_limit,
        }


def handle_request(controller, request):
    """Handle one decoded request and return a JSON-serializable response."""

    if not isinstance(request, dict):
        raise ProtocolError("request must be a JSON object")

    command = request.get("command")
    if command == "ping":
        return {
            "status": "ok",
            "message": "pong",
            "protocol_version": PROTOCOL_VERSION,
        }
    if command == "stop":
        controller.stop_all("remote-stop")
        return {"status": "ok", "stopped": True}
    if command == "drive":
        if "left" not in request or "right" not in request:
            raise ProtocolError("drive requires left and right speeds")
        if "tool" in request:
            raise ProtocolError(
                "raw tool speed is disabled; use bounded tool_move commands"
            )
        applied = controller.drive(request["left"], request["right"])
        return {"status": "ok", "applied": applied}
    if command == "tool_move":
        if "position" not in request or "speed" not in request:
            raise ProtocolError("tool_move requires position and speed")
        applied = controller.move_tool(request["position"], request["speed"])
        return {"status": "ok", "applied": applied}
    if command == "tool_home":
        applied = controller.home_tool(request.get("speed", 300))
        return {"status": "ok", "applied": applied}
    if command == "tool_zero":
        return {"status": "ok", "position": controller.zero_tool()}
    if command == "tool_acknowledge_position":
        return {
            "status": "ok",
            "position": controller.acknowledge_tool_position(),
        }
    if command == "status":
        response = controller.status()
        response["status"] = "ok"
        return response

    raise ProtocolError("unknown command")


def encode_response(response):
    return (json.dumps(response, separators=(",", ":"), sort_keys=True) + "\n").encode(
        "utf-8"
    )


def decode_request(line):
    try:
        text = line.decode("utf-8")
    except UnicodeDecodeError:
        raise ProtocolError("request must be UTF-8")
    try:
        return json.loads(text)
    except ValueError:
        raise ProtocolError("request must contain valid JSON")


def process_request_line(controller, line):
    """Decode and handle one line, stopping immediately on every error."""

    try:
        request = decode_request(line)
        return handle_request(controller, request)
    except ProtocolError as exc:
        controller.stop_all("protocol-error", suppress_errors=True)
        return {"status": "error", "error": str(exc)}
    except Exception as exc:
        controller.stop_all("request-failure", suppress_errors=True)
        return {"status": "error", "error": str(exc)}


class Ev3JsonServer(object):
    """Single-client TCP server with a watchdog-aware receive loop."""

    def __init__(
        self,
        controller,
        host=DEFAULT_HOST,
        port=DEFAULT_PORT,
        allowed_client=DEFAULT_ALLOWED_CLIENT,
        poll_seconds=DEFAULT_POLL_SECONDS,
        max_line_bytes=DEFAULT_MAX_LINE_BYTES,
        client_idle_timeout_seconds=DEFAULT_CLIENT_IDLE_TIMEOUT_SECONDS,
        clock=None,
    ):
        self.controller = controller
        self.host = host
        self.port = int(port)
        self.allowed_client = allowed_client
        self.poll_seconds = float(poll_seconds)
        self.max_line_bytes = int(max_line_bytes)
        self.client_idle_timeout_seconds = float(client_idle_timeout_seconds)
        if not math.isfinite(self.client_idle_timeout_seconds) or self.client_idle_timeout_seconds <= 0:
            raise ValueError("client idle timeout must be finite and positive")
        self.clock = clock or time.monotonic
        self._stopping = False
        self._listener = None

    def stop(self):
        self._stopping = True
        if self._listener is not None:
            try:
                self._listener.close()
            except OSError:
                pass

    def serve_forever(self):
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._listener = listener
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((self.host, self.port))
        listener.listen(1)
        listener.settimeout(0.25)
        print("EV3 JSON server listening on {0}:{1}".format(self.host, self.port))

        try:
            while not self._stopping:
                try:
                    connection, address = listener.accept()
                except socket.timeout:
                    continue
                except OSError:
                    if self._stopping:
                        break
                    raise
                if self.allowed_client and address[0] != self.allowed_client:
                    print("Rejected client from {0}".format(address[0]))
                    connection.close()
                    self.controller.stop_all(
                        "rejected-client", suppress_errors=True
                    )
                    continue
                print("Client connected from {0}:{1}".format(address[0], address[1]))
                self._handle_client(connection)
        finally:
            self.controller.stop_all("server-shutdown", suppress_errors=True)
            try:
                listener.close()
            except OSError:
                pass
            self._listener = None

    def _send_error(self, connection, message):
        connection.sendall(encode_response({"status": "error", "error": message}))

    def _handle_client(self, connection):
        connection.settimeout(self.poll_seconds)
        buffer = b""
        last_request_at = self.clock()

        try:
            while not self._stopping:
                if self.clock() - last_request_at >= self.client_idle_timeout_seconds:
                    print("Client idle timeout; accepting a fresh bridge connection")
                    break
                self.controller.enforce_watchdog()
                try:
                    data = connection.recv(1024)
                except socket.timeout:
                    continue
                if not data:
                    break
                buffer += data

                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    if len(line) > self.max_line_bytes:
                        self._send_error(connection, "request line is too long")
                        return
                    if not line.strip():
                        continue
                    last_request_at = self.clock()
                    response = process_request_line(self.controller, line)
                    connection.sendall(encode_response(response))

                if len(buffer) > self.max_line_bytes:
                    self._send_error(connection, "request line is too long")
                    return
        except OSError as exc:
            # A reset peer is a client disconnect, not a server/encoder reset.
            print("Client socket closed: {0}".format(exc))
        finally:
            try:
                connection.close()
            finally:
                self.controller.stop_all("client-disconnect", suppress_errors=True)
                print("Client disconnected; motors stopped")


def build_controller(args):
    motors = {}
    for role, port in MOTOR_PORTS.items():
        motors[role] = SysfsMotor(port, sysfs_root=args.sysfs_root)
    return MotorController(
        motors,
        watchdog_seconds=args.watchdog_ms / 1000.0,
        drive_speed_limit=args.drive_speed_limit,
        tool_speed_limit=args.tool_speed_limit,
        tool_position_limit=args.tool_position_limit,
        tool_step_limit=args.tool_step_limit,
        tool_move_timeout_seconds=args.tool_move_timeout,
        tool_stall_retry_seconds=args.tool_stall_retry,
        reference_path=args.camera_reference_file,
    )


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera-reference-file", default="/home/robot/echora/camera-reference.json")
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--allowed-client", default=DEFAULT_ALLOWED_CLIENT)
    parser.add_argument("--watchdog-ms", type=int, default=500)
    parser.add_argument("--drive-speed-limit", type=int, default=250)
    parser.add_argument(
        "--tool-speed-limit", type=int, default=DEFAULT_TOOL_SPEED_LIMIT
    )
    parser.add_argument(
        "--tool-position-limit", type=int, default=DEFAULT_TOOL_POSITION_LIMIT
    )
    parser.add_argument(
        "--tool-step-limit", type=int, default=DEFAULT_TOOL_STEP_LIMIT
    )
    parser.add_argument(
        "--tool-move-timeout", type=float, default=DEFAULT_TOOL_MOVE_TIMEOUT_SECONDS
    )
    parser.add_argument(
        "--tool-stall-retry",
        type=float,
        default=DEFAULT_TOOL_STALL_RETRY_SECONDS,
    )
    parser.add_argument(
        "--sysfs-root", default="/sys/class/tacho-motor", help=argparse.SUPPRESS
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    controller = build_controller(args)
    server = Ev3JsonServer(
        controller,
        host=args.host,
        port=args.port,
        allowed_client=args.allowed_client,
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("Stopping EV3 server")
    finally:
        server.stop()
        controller.stop_all("process-exit", suppress_errors=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
