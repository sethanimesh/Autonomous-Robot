"""Bounded named-position control for the vertically moving camera head."""

import json
import math
import os
from pathlib import Path
import time


class CameraHeadError(ValueError):
    """Raised when a camera-head command is unsafe or incomplete."""


class CameraHeadController(object):
    """Translate named or small relative commands into EV3 position moves."""

    def __init__(
        self,
        client,
        calibrated=False,
        forward_position=0,
        down_position=0,
        up_position=None,
        minimum_position=-180,
        maximum_position=0,
        speed=40,
        boost_speed=1500,
        home_speed=300,
        max_jog_degrees=15,
        settle_tolerance=8,
        step_timeout_seconds=10.0,
        post_move_settle_seconds=0.4,
        lower_target_margin=None,
        upper_target_margin=0,
        approved_reference_id=None,
        require_approved_reference=False,
        clock=None,
        sleeper=None,
        progress_callback=None,
        cancelled=None,
    ):
        if minimum_position >= maximum_position:
            raise ValueError("camera-head minimum must be below maximum")
        if speed <= 0:
            raise ValueError("camera-head speed must be positive")
        if boost_speed < speed:
            raise ValueError("camera-head boost speed must not be below normal speed")
        if home_speed <= 0:
            raise ValueError("camera-head home speed must be positive")
        if max_jog_degrees <= 0:
            raise ValueError("camera-head max jog must be positive")
        if settle_tolerance < 0 or settle_tolerance >= max_jog_degrees:
            raise ValueError("camera-head settle tolerance must be below jog size")
        if step_timeout_seconds <= 0:
            raise ValueError("camera-head step timeout must be positive")
        if post_move_settle_seconds < 0:
            raise ValueError("camera-head settle time cannot be negative")
        self.saved_limits = None
        self.limit_save_result = None
        self.limits_path = None
        self.manual_reference_id = None
        self.approved_reference_id = approved_reference_id
        self.require_approved_reference = bool(require_approved_reference)
        self.client = client
        self.calibrated = bool(calibrated)
        self.calibrated_reference_id = None
        self.calibration_source = None
        self.disconnected_limits = None
        self.calibration_error = None
        self.forward_position = int(forward_position)
        self.down_position = int(down_position)
        self.up_position = (
            int(minimum_position) if up_position is None else int(up_position)
        )
        self.minimum_position = int(minimum_position)
        self.maximum_position = int(maximum_position)
        self.speed = int(speed)
        self.boost_speed = int(boost_speed)
        self.home_speed = int(home_speed)
        self.max_jog_degrees = int(max_jog_degrees)
        self.settle_tolerance = int(settle_tolerance)
        self.upper_target_margin = int(upper_target_margin)
        if self.upper_target_margin < 0:
            raise ValueError("camera upper target margin cannot be negative")
        self.lower_target_margin = int(settle_tolerance if lower_target_margin is None else lower_target_margin)
        if self.lower_target_margin < 0:
            raise ValueError("camera lower target margin cannot be negative")
        self.step_timeout_seconds = float(step_timeout_seconds)
        self.post_move_settle_seconds = float(post_move_settle_seconds)
        self.clock = clock or time.monotonic
        self.sleeper = sleeper or time.sleep
        self.progress_callback = progress_callback
        self.cancelled = cancelled
        for name, position in (
            ("forward", self.forward_position),
            ("down", self.down_position),
            ("up", self.up_position),
        ):
            self._validate_target(name, position)

    @property
    def minimum_target_position(self):
        return self.minimum_position + (self.upper_target_margin if self.require_approved_reference else 0)

    @property
    def maximum_target_position(self):
        # Keep the loaded motor settling allowance INSIDE the physical boundary.
        return self.maximum_position - (self.lower_target_margin if self.require_approved_reference else 0)

    def _validate_target(self, name, position):
        if position < self.minimum_target_position or position > self.maximum_target_position:
            raise ValueError(
                "{0} position must be between {1} and {2}".format(
                    name, self.minimum_target_position, self.maximum_target_position
                )
            )
        return int(position)

    @staticmethod
    def _number(name, value):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise CameraHeadError("{0} must be a number".format(name))
        if not math.isfinite(value):
            raise CameraHeadError("{0} must be finite".format(name))
        return int(round(value))

    def _current_position(self):
        if self.manual_reference_id is not None:
            raise CameraHeadError("Confirm the camera lower view before automatic camera movement")
        status = self.client.status()
        reason = self.observe_status(status)
        if reason:
            raise CameraHeadError(reason)
        try:
            return int(status["motors"]["tool"]["position"])
        except (KeyError, TypeError, ValueError):
            raise CameraHeadError("EV3 status has no camera-head position")

    def move_named(self, name):
        self._check_move_cancelled()
        self._current_position()
        if not self.calibrated:
            raise CameraHeadError(
                "camera head is not calibrated; use only small jog commands"
            )
        positions = {
            "forward": self.forward_position,
            "down": self.down_position,
            "up": self.up_position,
            "person": self.forward_position,
            "floor": self.down_position,
            "face": self.up_position,
        }
        if name not in positions:
            raise CameraHeadError("unknown camera-head position")
        return self.move_in_steps(positions[name], require_final_target=True)

    def move_in_steps(self, destination, require_final_target=False):
        """Move a loaded head using only the proven small increments."""

        destination = self._validate_target("destination", int(destination))
        self._check_move_cancelled()
        current = self._current_position()
        moves = []
        move_count = 0
        pending_final = require_final_target and current != destination
        while abs(destination - current) > self.settle_tolerance or pending_final:
            self._check_move_cancelled()
            move_count += 1
            if move_count > 24:
                self.client.stop()
                raise CameraHeadError("camera head could not converge on destination")
            delta = destination - current
            step = max(-self.max_jog_degrees, min(self.max_jog_degrees, delta))
            target = current + step
            # Negative counts lift the current heavy linkage. That loaded
            # direction gets exactly one near-rated retry when a normal
            # 15-count pulse makes no measurable progress. Never enlarge the
            # pulse itself.
            speeds = [self.speed]
            if step < 0 and self.boost_speed > self.speed:
                speeds.append(self.boost_speed)
            step_complete = False
            # Retries and settling share the same absolute chunk deadline.
            deadline = self.clock() + self.step_timeout_seconds
            for attempt_index, speed in enumerate(speeds):
                self._check_move_cancelled()
                if self.clock() >= deadline:
                    self.client.stop()
                    raise CameraHeadError("camera head timed out before position {0}".format(target))
                self.client.move_tool(target, speed)
                stable_since = None
                stable_position = None
                while True:
                    self._check_move_cancelled()
                    status = self.client.status()
                    reason = self.observe_status(status)
                    if self.progress_callback is not None:
                        self.progress_callback(status)
                    self._check_move_cancelled()
                    if reason:
                        self.client.stop()
                        raise CameraHeadError(reason)
                    try:
                        actual = int(status["motors"]["tool"]["position"])
                    except (KeyError, TypeError, ValueError):
                        self.client.stop()
                        raise CameraHeadError(
                            "EV3 status has no camera-head position"
                        )
                    now = self.clock()
                    if now >= deadline:
                        self.client.stop()
                        raise CameraHeadError("camera head timed out before position {0}".format(target))
                    terminal_reason = status.get("last_stop_reason")
                    if terminal_reason in (
                        "tool-stalled-after-retry", "tool-stall-retry-failure",
                        "tool-move-timeout", "tool-move-failure", "tool-home-timeout",
                    ):
                        self.client.stop()
                        raise CameraHeadError("camera head stopped: {0}".format(terminal_reason))
                    if status.get("tool_motion_active", False):
                        stable_since = None
                        stable_position = None
                    else:
                        # A stopped flag alone is not settlement: the loaded
                        # linkage may drift, or its motor may resume between polls.
                        if stable_position != actual or stable_since is None:
                            stable_position = actual
                            stable_since = now
                        if now - stable_since + 1e-9 < self.post_move_settle_seconds:
                            self.sleeper(min(0.1, deadline - now,
                                             self.post_move_settle_seconds - (now - stable_since)))
                            continue
                        if abs(actual - target) > self.settle_tolerance:
                            try:
                                retry_count = int(status.get("tool_stall_retry_count", 0))
                                retry_limit = int(status.get("tool_stall_retry_limit", 0))
                            except (TypeError, ValueError):
                                retry_count = retry_limit = 0
                            self.client.stop()
                            if ("tool_stall_retry_limit" in status
                                    and retry_count >= retry_limit):
                                # Partial encoder progress does not justify a new
                                # command that resets an exhausted brick retry.
                                raise CameraHeadError("camera head stalled before destination {0}".format(destination))
                            break
                        moves.append(
                            {
                                "requested": target,
                                "actual": actual,
                                "speed": speed,
                                "boosted": attempt_index > 0,
                            }
                        )
                        current = actual
                        pending_final = require_final_target and target != destination
                        step_complete = True
                        break
                    self.sleeper(min(0.1, deadline - now))
                if step_complete:
                    break
            if not step_complete:
                raise CameraHeadError(
                    "camera head stalled before destination {0}".format(destination)
                )
        return {"position": current, "destination": destination, "steps": moves}

    def _check_move_cancelled(self):
        if self.cancelled is not None and self.cancelled():
            self.client.stop()
            raise CameraHeadError("camera head movement cancelled")

    def jog(self, degrees, speed=None):
        degrees = self._number("jog degrees", degrees)
        if degrees == 0 or abs(degrees) > self.max_jog_degrees:
            raise CameraHeadError(
                "jog must be non-zero and no more than {0} degrees".format(
                    self.max_jog_degrees
                )
            )
        position = self._current_position()
        if degrees < 0 and position <= self.minimum_target_position:
            raise CameraHeadError("Upward jog would enter the upper-boundary settling margin")
        if degrees > 0 and position >= self.maximum_target_position:
            raise CameraHeadError("Downward jog would enter the approved lower-boundary settling margin")
        target = min(
            self.maximum_target_position, max(self.minimum_target_position, position + degrees)
        )
        if target == position:
            raise CameraHeadError("camera head is already at the configured limit")
        applied_speed = self.speed if speed is None else self._number("jog speed", speed)
        if applied_speed <= 0 or applied_speed > self.boost_speed:
            raise CameraHeadError(
                "jog speed must be between 1 and {0}".format(self.boost_speed)
            )
        return self.client.move_tool(target, applied_speed)

    def jog_to(self, target, speed=None):
        """Move to one bounded absolute target supplied by a manual UI."""

        target = self._validate_target(
            "jog target", self._number("jog target", target)
        )
        position = self._current_position()
        distance = target - position
        if distance == 0 or abs(distance) > self.max_jog_degrees:
            raise CameraHeadError(
                "jog target must be within one non-zero {0}-degree step".format(
                    self.max_jog_degrees
                )
            )
        applied_speed = self.speed if speed is None else self._number("jog speed", speed)
        if applied_speed <= 0 or applied_speed > self.boost_speed:
            raise CameraHeadError(
                "jog speed must be between 1 and {0}".format(self.boost_speed)
            )
        return self.client.move_tool(target, applied_speed)

    def retry_jog(self, target):
        """Reapply one active jog's absolute target without stacking travel."""

        target = self._validate_target(
            "retry target", self._number("retry target", target)
        )
        status = self.client.status()
        reason = self.observe_status(status)
        if reason:
            raise CameraHeadError(reason)
        try:
            current = int(status["motors"]["tool"]["position"])
            active_target = int(status["tool_target_position"])
        except (KeyError, TypeError, ValueError):
            raise CameraHeadError("EV3 status has no camera-head retry target")
        if target != active_target:
            raise CameraHeadError("camera-head retry target is no longer active")
        remaining = target - current
        if abs(remaining) > self.max_jog_degrees:
            raise CameraHeadError(
                "camera-head retry would exceed the bounded jog distance"
            )
        # Negative encoder travel lifts this heavy linkage. Manual retries use
        # the same near-rated recovery as the automatic no-progress watchdog;
        # downward retries stay at normal speed to avoid a gravity-driven jolt.
        speed = self.boost_speed if remaining < 0 else self.speed
        return self.client.move_tool(target, speed)

    def manual_jog(self, expected_reference, expected_position, target):
        """Honor a human's existing UI buttons independently of automatic limits."""
        status = self.client.status()
        position = int(status["motors"]["tool"]["position"])
        target = self._number("manual target", target)
        if (status.get("tool_reference_id") != expected_reference
                or position != self._number("expected position", expected_position)):
            raise CameraHeadError("Camera position changed; refresh and press again")
        if abs(target - position) > self.max_jog_degrees:
            raise CameraHeadError("Manual target must fit the selected button step")
        # Reading the current encoder does not zero it or claim a physical limit.
        if not status.get("tool_homed"):
            self.client.acknowledge_tool_position()
            status = self.client.status()
            if not status.get("tool_homed") or int(status["motors"]["tool"]["position"]) != position:
                raise CameraHeadError("Encoder changed while enabling manual control")
        within_saved_limits = (self.saved_limits is not None
            and self.saved_limits.get('upper') is not None
            and status['tool_reference_id'] == self.approved_reference_id
            and self.minimum_position <= target <= self.maximum_position)
        if not within_saved_limits:
            self.invalidate_calibration()
        self.manual_reference_id = None if within_saved_limits else status["tool_reference_id"]
        if within_saved_limits and not self.calibrated:
            self._apply_manual_limits(self.saved_limits)
        return self.client.move_tool(target, self.speed)

    def load_manual_limits(self, path):
        """Restore saved endpoints; normal reference checks still apply after reboot."""
        self.limits_path = Path(path)
        if not self.limits_path.exists():
            return
        limits = json.loads(self.limits_path.read_text())
        self._validate_manual_limits(limits)
        self._apply_manual_limits(limits)

    def _validate_manual_limits(self, limits):
        if not isinstance(limits, dict) or limits.get('version') != 1:
            raise CameraHeadError('Saved camera limits are invalid; set them again')
        if not isinstance(limits.get('reference_id'), str) or not limits['reference_id']:
            raise CameraHeadError('Camera reference is missing')
        lower = self._number('lower limit', limits.get('lower'))
        upper = limits.get('upper')
        if upper is not None:
            upper = self._number('upper limit', upper)
            minimum_span = self.lower_target_margin + self.upper_target_margin + 10
            if lower - upper < minimum_span:
                raise CameraHeadError('Raise the camera farther before saving the upper limit')

    def _apply_manual_limits(self, limits):
        self.saved_limits = dict(limits)
        self.require_approved_reference = True
        self.invalidate_calibration()
        if limits['upper'] is None:
            self.approved_reference_id = None
            self.manual_reference_id = limits['reference_id']
            return
        self.maximum_position = int(limits['lower'])
        self.minimum_position = int(limits['upper'])
        self.down_position = self.maximum_target_position
        self.up_position = self.minimum_target_position
        # Cloud setup verified the lower view contains both room and floor.
        # The numeric midpoint was never observed and may face the ceiling.
        self.forward_position = (self.down_position
            if limits.get('source') in ('cloud_useful_views', 'groq_useful_views')
            else round((self.down_position + self.up_position) / 2))
        self.approved_reference_id = limits['reference_id']
        self.manual_reference_id = None
        self.calibrated = True
        self.calibrated_reference_id = limits['reference_id']
        self.calibration_source = limits.get('source', 'operator_limits')
        self.calibration_error = None

    def use_saved_limits(self):
        """Reuse physical endpoints only after checking this boot and stopped pose."""
        status = self.client.status()
        reason = self.observe_status(status)
        if reason:
            raise CameraHeadError(reason)
        if (not self.saved_limits or self.saved_limits.get('upper') is None
                or self.saved_limits.get('reference_id') != status.get('tool_reference_id')
                or self.manual_reference_id is not None):
            raise CameraHeadError('Save both camera limits for this boot first')
        if status.get('tool_motion_active') or status.get('tool_homing'):
            raise CameraHeadError('Wait for the camera to stop')
        self._apply_manual_limits(self.saved_limits)
        return {'calibrated': True, 'calibration_source': self.calibration_source}

    def _persist_manual_limits(self, limits):
        if self.limits_path is not None:
            self.limits_path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.limits_path.with_suffix('.tmp')
            temporary.write_text(json.dumps(limits, indent=2) + '\n')
            os.replace(temporary, self.limits_path)

    def _confirmed_lower_status(self, reference, position):
        status = self.client.status()
        actual_reference = status.get('tool_reference_id')
        if (not isinstance(actual_reference, str) or not actual_reference
                or (reference is not None and actual_reference != reference)
                or self._number('camera position', status['motors']['tool']['position']) != position):
            raise CameraHeadError('Camera position or reference changed; confirm the lower view again')
        if any(status.get(key) for key in (
                'tool_motion_active', 'tool_homing', 'track_motion_active', 'motion_active')):
            raise CameraHeadError('Wait for the camera and tracks to stop before restoring the range')
        for motor in status['motors'].values():
            if (motor.get('speed', 0) != 0
                    or any(state in motor.get('state', []) for state in ('running', 'ramping'))):
                raise CameraHeadError('Wait for the camera and tracks to stop before restoring the range')
        return status

    def restore_saved_range_from_lower(self, request):
        """Rebind a saved span only at an operator-confirmed physical lower view."""
        request_id = request.get('request_id')
        reference_changed = False
        try:
            if request.get('operator_confirmed') is not True:
                raise CameraHeadError('Confirm that the camera is at the saved physical lower view')
            if not isinstance(request_id, str) or not request_id:
                raise CameraHeadError('Camera range request ID is missing')
            reference = request.get('expected_reference_id')
            if not isinstance(reference, str) or not reference:
                raise CameraHeadError('Current camera reference is missing')
            self._validate_manual_limits(self.saved_limits)
            if self.saved_limits.get('upper') is None:
                raise CameraHeadError('Save both camera limits once before restoring their range')
            span = (self._number('saved lower limit', self.saved_limits['lower'])
                    - self._number('saved upper limit', self.saved_limits['upper']))
            position = self._number('expected position', request.get('expected_position'))
            self._confirmed_lower_status(reference, position)
            self.sleeper(0.1)
            status = self._confirmed_lower_status(reference, position)
            position_limit = status.get('tool_position_limit')
            if position_limit is not None:
                position_limit = self._number('encoder position limit', position_limit)
                if position_limit <= 0 or max(abs(position), abs(position-span)) > position_limit:
                    raise CameraHeadError('Saved camera span exceeds the supported encoder range at this position')
            reference_changed = reference != self.approved_reference_id
            if not status.get('tool_homed'):
                # Acknowledge the current count without zeroing or moving. If
                # persistence later fails, the old saved span must remain
                # available, but it must not authorize this new reference.
                reference_changed = True
                self.invalidate_calibration()
                acknowledgement = self.client.acknowledge_tool_position()
                if self._number('acknowledged position', acknowledgement.get('position')) != position:
                    raise CameraHeadError('Camera moved while restoring its reference; confirm the lower view again')
                status = self._confirmed_lower_status(None, position)
                reference = status['tool_reference_id']
                self.sleeper(0.1)
                status = self._confirmed_lower_status(reference, position)
            if not status.get('tool_homed'):
                raise CameraHeadError('Camera reference could not be established; try again')
            limits = dict(version=1, reference_id=reference, lower=position, upper=position-span)
            self._validate_manual_limits(limits)
            self._persist_manual_limits(limits)
            self._apply_manual_limits(limits)
            self.limit_save_result = dict(request_id=request_id, ok=True,
                                          limit='restore_saved_range_from_lower')
            return self.limit_save_result
        except Exception as exc:
            if reference_changed:
                self.invalidate_calibration()
                self.calibration_error = str(exc)
            self.limit_save_result = dict(request_id=request_id, ok=False, error=str(exc))
            raise CameraHeadError(str(exc))

    def save_manual_limit(self, request):
        """Save the user's current endpoint without moving or zeroing the motor."""
        request_id = request.get('request_id')
        try:
            kind = request.get('limit')
            if kind not in ('lower', 'upper'):
                raise CameraHeadError('Choose lower or upper limit')
            status = self.client.status()
            position = int(status['motors']['tool']['position'])
            if (status.get('tool_reference_id') != request.get('expected_reference_id')
                    or position != self._number('expected position', request.get('expected_position'))):
                raise CameraHeadError('Camera position changed; wait for it to stop and save again')
            if any(status.get(key) for key in ('tool_motion_active', 'tool_homing', 'track_motion_active', 'motion_active')):
                raise CameraHeadError('Wait for the camera and tracks to stop before saving')
            if not status.get('tool_homed'):
                self.client.acknowledge_tool_position()
                status = self.client.status()
                if not status.get('tool_homed') or int(status['motors']['tool']['position']) != position:
                    raise CameraHeadError('Camera reference changed while saving; try again')
            reference = status['tool_reference_id']
            if kind == 'lower':
                limits = dict(version=1, reference_id=reference, lower=position, upper=None)
            else:
                if not self.saved_limits or self.saved_limits['reference_id'] != reference:
                    raise CameraHeadError('Save the lower limit first')
                limits = dict(self.saved_limits, upper=position)
            self._validate_manual_limits(limits)
            self._persist_manual_limits(limits)
            self._apply_manual_limits(limits)
            self.limit_save_result = dict(request_id=request_id, ok=True, limit=kind)
            return self.limit_save_result
        except Exception as exc:
            self.limit_save_result = dict(request_id=request_id, ok=False, error=str(exc))
            raise CameraHeadError(str(exc))

    def save_visual_limits(self, request):
        """Persist the coordinator's observed operating views, without homing."""
        request_id = request.get('request_id')
        try:
            reference = request.get('expected_reference_id')
            position = self._number('expected position', request.get('expected_position'))
            status = self._confirmed_lower_status(reference, position)
            if not reference or not status.get('tool_homed') or not request_id:
                raise CameraHeadError('Visual setup needs the current encoder reference')
            lower = self._number('lower view', request.get('lower'))
            upper = self._number('upper view', request.get('upper'))
            if not upper <= position <= lower:
                raise CameraHeadError('Camera is outside the observed view range')
            limits = dict(version=1, reference_id=reference, lower=lower, upper=upper,
                          source='cloud_useful_views')
            self._validate_manual_limits(limits)
            self._persist_manual_limits(limits)
            self._apply_manual_limits(limits)
            self.limit_save_result = dict(request_id=request_id, ok=True, limit='visual_views')
            return self.limit_save_result
        except Exception as exc:
            self.limit_save_result = dict(request_id=request_id, ok=False, error=str(exc))
            raise CameraHeadError(str(exc))

    def begin_visual_setup(self, request):
        """Acknowledge a new boot at its actual count; no physical datum claim."""
        request_id = request.get('request_id')
        try:
            reference = request.get('expected_reference_id')
            position = self._number('expected position', request.get('expected_position'))
            status = self._confirmed_lower_status(reference, position)
            if not reference or not request_id:
                raise CameraHeadError('Visual setup reference request is missing')
            if not status.get('tool_homed'):
                self.client.acknowledge_tool_position()
                status = self._confirmed_lower_status(None, position)
            if not status.get('tool_homed') or not status.get('tool_reference_id'):
                raise CameraHeadError('Camera encoder could not be acknowledged')
            self.invalidate_calibration()
            self.manual_reference_id = status['tool_reference_id']
            self.setup_reference_result = dict(request_id=request_id, ok=True,
                previous_reference_id=reference, reference_id=status['tool_reference_id'], position=position)
            return self.setup_reference_result
        except Exception as exc:
            self.setup_reference_result = dict(request_id=request_id, ok=False, error=str(exc))
            raise CameraHeadError(str(exc))

    def reference_current_lower(self, expected_reference, expected_position, operator_confirmed=False):
        """Record an explicitly confirmed physical lower pose without zero/motion.

        This setup action is never part of automatic startup. Relative travel
        spans are retained; only their encoder coordinates change.
        """
        if operator_confirmed is not True:
            raise CameraHeadError("Current physical lower view requires operator confirmation")
        status = self.client.status()
        position = int(status["motors"]["tool"]["position"])
        if (status.get("tool_reference_id") != expected_reference
                or position != self._number("expected position", expected_position)
                or status.get("tool_motion_active") or status.get("tool_homing")
                or status.get("track_motion_active") or status.get("motion_active")):
            raise CameraHeadError("Robot moved or reference changed after lower-view confirmation")
        span = self.maximum_position - self.minimum_position
        forward_offset = self.forward_position - self.maximum_position
        up_offset = self.up_position - self.maximum_position
        self.invalidate_calibration()
        self.client.acknowledge_tool_position()
        fresh = self.client.status()
        if (not fresh.get("tool_homed") or fresh.get("tool_motion_active")
                or int(fresh["motors"]["tool"]["position"]) != position
                or not fresh.get("tool_reference_id")):
            raise CameraHeadError("Retained encoder reference could not be established")
        self.require_approved_reference = True
        self.minimum_position = position - span
        self.maximum_position = position
        self.down_position = self.maximum_target_position
        self.forward_position = min(self.maximum_target_position, position + forward_offset)
        self.up_position = min(self.maximum_target_position, position + up_offset)
        self.approved_reference_id = fresh["tool_reference_id"]
        self.manual_reference_id = None
        self.require_approved_reference = True
        return {"approved_reference_id": self.approved_reference_id,
                "minimum_position": self.minimum_position, "maximum_position": self.maximum_position,
                "down_position": self.down_position, "forward_position": self.forward_position,
                "up_position": self.up_position, "calibrated": False}

    def _check_reference_change_allowed(self):
        if self.require_approved_reference:
            raise CameraHeadError("Approved physical limits forbid zeroing, homing, or changing the encoder reference")

    def zero(self):
        self._check_reference_change_allowed()
        if self.calibrated:
            raise CameraHeadError("disable calibration before changing camera-head zero")
        self.invalidate_calibration()
        return self.client.zero_tool()

    def home(self):
        self._check_reference_change_allowed()
        self.invalidate_calibration()
        return self.client.home_tool(self.home_speed)

    def set_runtime_positions(self, forward, down, up=None, reference_id=None):
        if self.manual_reference_id is not None:
            raise CameraHeadError("Mark the lowest view before calibration")
        status = self.client.status()
        reason = self.observe_status(status)
        if reason:
            raise CameraHeadError(reason)
        if reference_id is not None and status.get("tool_reference_id") != reference_id:
            raise CameraHeadError("calibration belongs to a different encoder reference")
        if status.get("tool_motion_active") or status.get("tool_homing"):
            raise CameraHeadError("camera head must be stopped before calibration")
        try:
            forward = self._validate_target(
                "forward", self._number("forward", forward)
            )
            down = self._validate_target("down", self._number("down", down))
            up = self._validate_target(
                "up",
                self._number("up", self.up_position if up is None else up),
            )
        except ValueError as exc:
            raise CameraHeadError(str(exc))
        if down == up:
            raise CameraHeadError("floor and face camera positions must be distinct")
        self.forward_position = forward
        self.down_position = down
        self.up_position = up
        # Static boot values are only travel bounds. Receiving a complete
        # runtime set means this boot's vision or operator-assisted selection
        # succeeded. A low-mounted camera may intentionally use one view for
        # both clear-floor checking and initial lower-body person acquisition.
        self.calibrated = True
        self.calibration_source = "vision"
        self.calibrated_reference_id = status["tool_reference_id"]
        self.calibration_error = None
        return {
            "forward_position": forward,
            "down_position": down,
            "up_position": up,
        }

    def invalidate_calibration(self):
        """Lock named poses until a complete boot sweep succeeds again."""

        self.calibrated = False
        self.calibration_source = None
        self.calibrated_reference_id = None
        self.disconnected_limits = None
        return {"calibrated": False}

    def suspend_for_status_disconnect(self):
        """Keep operator endpoints pending validation after an idle poll timeout."""
        limits = self.disconnected_limits
        if self.calibrated and self.calibration_source in ('operator_limits', 'groq_useful_views', 'cloud_useful_views'):
            limits = dict(self.saved_limits) if self.saved_limits else None
        self.invalidate_calibration()
        self.disconnected_limits = limits

    def acknowledge_position(self):
        self._check_reference_change_allowed()
        self.invalidate_calibration()
        return self.client.acknowledge_tool_position()

    def observe_status(self, status):
        """Invalidate poses when their physical encoder reference is lost."""
        reason = None
        try:
            position = self._number("camera position", status["motors"]["tool"]["position"])
        except (KeyError, TypeError, ValueError):
            reason = "camera-head encoder is unavailable"
            position = None
        reference = status.get("tool_reference_id")
        if self.manual_reference_id is not None:
            if (reference == self.manual_reference_id and status.get("tool_homed")
                    and not status.get("tool_homing") and position is not None):
                self.invalidate_calibration()
                self.calibration_error = "Manual camera control; automatic view calibration is required"
                return None
            self.manual_reference_id = None
        if not isinstance(reference, str) or not reference:
            reason = "EV3 camera reference is unavailable; update the EV3 service"
        elif not status.get("tool_homed") or status.get("tool_homing"):
            reason = "camera head needs a mechanical reference"
        elif self.require_approved_reference and (not self.approved_reference_id or reference != self.approved_reference_id):
            reason = "Approved physical camera limits need a matching encoder reference; head motion is locked"
        elif position is not None and self.require_approved_reference and position > self.maximum_position:
            reason = "Camera head crossed the approved lower physical boundary"
        elif position is not None and not (
            self.minimum_position - self.settle_tolerance
            <= position <= self.maximum_position + self.settle_tolerance
        ):
            reason = "camera head is outside its calibrated travel range"
        elif self.calibrated_reference_id is not None and reference != self.calibrated_reference_id:
            reason = "camera encoder reference changed; recalibration is required"
        if reason:
            self.invalidate_calibration()
            self.calibration_error = reason
        elif self.calibrated:
            self.calibrated_reference_id = reference
        elif (self.disconnected_limits is not None
                and self.disconnected_limits == self.saved_limits
                and reference == self.disconnected_limits.get('reference_id')
                and not status.get('tool_motion_active', True)
                and not status.get('tool_homing', False)
                and self.minimum_position <= position <= self.maximum_position
                and abs(status.get('motors', {}).get('tool', {}).get('speed', 999)) <= 5):
            self._apply_manual_limits(self.disconnected_limits)
        else:
            self.calibration_error = "camera view calibration is required"
        return reason

    def execute(self, payload):
        if isinstance(payload, str):
            try:
                request = json.loads(payload)
            except ValueError:
                request = {"action": payload.strip()}
        elif isinstance(payload, dict):
            request = payload
        else:
            raise CameraHeadError("camera-head command must be text or JSON")
        if not isinstance(request, dict):
            raise CameraHeadError("camera-head JSON command must be an object")

        action = request.get("action")
        if action == 'move_to':
            request_id = request.get('request_id')
            try:
                result = self.move_in_steps(self._number('target', request.get('target')),
                                            require_final_target=True)
            except Exception as exc:
                self.move_result = dict(request_id=request_id, ok=False, error=str(exc))
                raise
            self.move_result = dict(request_id=request_id, ok=True)
            return result
        if action == "stop":
            return self.client.stop()
        if action == "use_saved_limits":
            return self.use_saved_limits()
        if action == "save_manual_limit":
            return self.save_manual_limit(request)
        if action == 'save_visual_limits':
            return self.save_visual_limits(request)
        if action == 'begin_visual_setup':
            return self.begin_visual_setup(request)
        if action == "restore_saved_range_from_lower":
            return self.restore_saved_range_from_lower(request)
        if action == "manual_jog":
            return self.manual_jog(request.get("expected_reference_id"), request.get("expected_position"), request.get("target"))
        if action == 'setup_jog':
            # The visual coordinator owns its finite discovery window. Old
            # absolute limits may belong to a different boot or an incorrect
            # view; they must not block finding the actual floor view.
            self._confirmed_lower_status(request.get('expected_reference_id'),
                                         self._number('expected position', request.get('expected_position')))
            result = self.manual_jog(request.get('expected_reference_id'),
                                     request.get('expected_position'), request.get('target'))
            self.manual_reference_id = request.get('expected_reference_id')
            self.invalidate_calibration()
            return result
        if action == "reference_current_lower":
            return self.reference_current_lower(request.get("expected_reference_id"),
                                                request.get("expected_position"),
                                                request.get("operator_confirmed", False))
        if action == "zero":
            return self.zero()
        if action == "home":
            return self.home()
        if action == "acknowledge_position":
            return self.acknowledge_position()
        if action == "set_runtime_positions":
            if "forward" not in request or "down" not in request:
                raise CameraHeadError(
                    "set_runtime_positions requires forward and down"
                )
            return self.set_runtime_positions(
                request["forward"], request["down"], request.get("up"), request.get("reference_id")
            )
        if action == "invalidate_calibration":
            return self.invalidate_calibration()
        if action == "jog":
            if "degrees" not in request:
                raise CameraHeadError("jog requires degrees")
            return self.jog(request["degrees"], request.get("speed"))
        if action == "jog_to":
            if "target" not in request:
                raise CameraHeadError("jog_to requires target")
            return self.jog_to(request["target"], request.get("speed"))
        if action == "retry_jog":
            if "target" not in request:
                raise CameraHeadError("retry_jog requires target")
            return self.retry_jog(request["target"])
        if action == "look_forward":
            return self.move_named("forward")
        if action == "look_down":
            return self.move_named("down")
        if action == "look_up":
            return self.move_named("up")
        if action == "look_person":
            return self.move_named("person")
        if action == "look_floor":
            return self.move_named("floor")
        if action == "look_face":
            return self.move_named("face")
        raise CameraHeadError("unknown camera-head action")

    def describe(self, ev3_status):
        self.observe_status(ev3_status)
        result = {
            "move_result": getattr(self, "move_result", None),
            "setup_reference_result": getattr(self, 'setup_reference_result', None),
            "saved_limits": self.saved_limits,
            "limit_save_result": self.limit_save_result,
            "manual_override": self.manual_reference_id is not None,
            "calibrated": self.calibrated,
            "calibration_source": self.calibration_source,
            "forward_position": self.forward_position,
            "down_position": self.down_position,
            "up_position": self.up_position,
            "minimum_position": self.minimum_position,
            "maximum_position": self.maximum_position,
            "maximum_target_position": self.maximum_target_position,
            "minimum_target_position": self.minimum_target_position,
            "upper_target_margin": self.upper_target_margin,
            "lower_target_margin": self.lower_target_margin,
            "approved_reference_id": self.approved_reference_id,
            "require_approved_reference": self.require_approved_reference,
            "speed": self.speed,
            "boost_speed": self.boost_speed,
            "home_speed": self.home_speed,
            "settle_tolerance": self.settle_tolerance,
            "reference_id": ev3_status.get("tool_reference_id"),
            "calibration_error": self.calibration_error,
        }
        try:
            position = int(ev3_status["motors"]["tool"]["position"])
            result["position"] = position
            if self.calibrated:
                at_forward = abs(position - self.forward_position) <= 2
                at_down = abs(position - self.down_position) <= 2
                at_up = abs(position - self.up_position) <= 2
                if at_forward and at_down:
                    result["named_position"] = "floor_person"
                elif at_forward:
                    result["named_position"] = "forward"
                elif at_down:
                    result["named_position"] = "down"
                elif at_up:
                    result["named_position"] = "up"
                else:
                    result["named_position"] = "between"
        except (KeyError, TypeError, ValueError):
            result["position"] = None
        result["moving"] = bool(ev3_status.get("tool_motion_active", False))
        result["homing"] = bool(ev3_status.get("tool_homing", False))
        result["homed"] = bool(ev3_status.get("tool_homed", False))
        try:
            target = ev3_status.get("tool_target_position")
            result["target_position"] = None if target is None else int(target)
        except (TypeError, ValueError):
            result["target_position"] = None
        result["stall_retry_count"] = int(
            ev3_status.get("tool_stall_retry_count", 0)
        )
        result["stall_retry_limit"] = int(
            ev3_status.get("tool_stall_retry_limit", 1)
        )
        result["stall_retry_ms"] = int(
            ev3_status.get("tool_stall_retry_ms", 1000)
        )
        return result
