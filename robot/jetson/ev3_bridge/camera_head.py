"""Bounded named-position control for the vertically moving camera head."""

import json
import math
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
        minimum_position=-180,
        maximum_position=0,
        speed=40,
        home_speed=60,
        max_jog_degrees=15,
        settle_tolerance=8,
        step_timeout_seconds=10.0,
        clock=None,
        sleeper=None,
    ):
        if minimum_position >= maximum_position:
            raise ValueError("camera-head minimum must be below maximum")
        if speed <= 0:
            raise ValueError("camera-head speed must be positive")
        if home_speed <= 0:
            raise ValueError("camera-head home speed must be positive")
        if max_jog_degrees <= 0:
            raise ValueError("camera-head max jog must be positive")
        if settle_tolerance < 0 or settle_tolerance >= max_jog_degrees:
            raise ValueError("camera-head settle tolerance must be below jog size")
        if step_timeout_seconds <= 0:
            raise ValueError("camera-head step timeout must be positive")
        self.client = client
        self.calibrated = bool(calibrated)
        self.forward_position = int(forward_position)
        self.down_position = int(down_position)
        self.minimum_position = int(minimum_position)
        self.maximum_position = int(maximum_position)
        self.speed = int(speed)
        self.home_speed = int(home_speed)
        self.max_jog_degrees = int(max_jog_degrees)
        self.settle_tolerance = int(settle_tolerance)
        self.step_timeout_seconds = float(step_timeout_seconds)
        self.clock = clock or time.monotonic
        self.sleeper = sleeper or time.sleep
        for name, position in (
            ("forward", self.forward_position),
            ("down", self.down_position),
        ):
            self._validate_target(name, position)

    def _validate_target(self, name, position):
        if position < self.minimum_position or position > self.maximum_position:
            raise ValueError(
                "{0} position must be between {1} and {2}".format(
                    name, self.minimum_position, self.maximum_position
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
        status = self.client.status()
        try:
            return int(status["motors"]["tool"]["position"])
        except (KeyError, TypeError, ValueError):
            raise CameraHeadError("EV3 status has no camera-head position")

    def move_named(self, name):
        if not self.calibrated:
            raise CameraHeadError(
                "camera head is not calibrated; use only small jog commands"
            )
        positions = {
            "forward": self.forward_position,
            "down": self.down_position,
        }
        if name not in positions:
            raise CameraHeadError("unknown camera-head position")
        return self.move_in_steps(positions[name])

    def move_in_steps(self, destination):
        """Move a loaded head using only the proven small increments."""

        destination = self._validate_target("destination", int(destination))
        current = self._current_position()
        moves = []
        while abs(destination - current) > self.settle_tolerance:
            delta = destination - current
            step = max(-self.max_jog_degrees, min(self.max_jog_degrees, delta))
            target = current + step
            self.client.move_tool(target, self.speed)
            deadline = self.clock() + self.step_timeout_seconds
            while True:
                status = self.client.status()
                try:
                    actual = int(status["motors"]["tool"]["position"])
                except (KeyError, TypeError, ValueError):
                    raise CameraHeadError("EV3 status has no camera-head position")
                if not status.get("tool_motion_active", False):
                    if abs(actual - target) > self.settle_tolerance:
                        raise CameraHeadError(
                            "camera head stalled before position {0}".format(target)
                        )
                    moves.append({"requested": target, "actual": actual})
                    current = actual
                    break
                if self.clock() >= deadline:
                    self.client.stop()
                    raise CameraHeadError(
                        "camera head timed out before position {0}".format(target)
                    )
                self.sleeper(0.1)
        return {"position": current, "destination": destination, "steps": moves}

    def jog(self, degrees):
        degrees = self._number("jog degrees", degrees)
        if degrees == 0 or abs(degrees) > self.max_jog_degrees:
            raise CameraHeadError(
                "jog must be non-zero and no more than {0} degrees".format(
                    self.max_jog_degrees
                )
            )
        position = self._current_position()
        target = min(
            self.maximum_position, max(self.minimum_position, position + degrees)
        )
        if target == position:
            raise CameraHeadError("camera head is already at the configured limit")
        return self.client.move_tool(target, self.speed)

    def zero(self):
        if self.calibrated:
            raise CameraHeadError("disable calibration before changing camera-head zero")
        return self.client.zero_tool()

    def home(self):
        return self.client.home_tool(self.home_speed)

    def set_runtime_positions(self, forward, down):
        try:
            forward = self._validate_target(
                "forward", self._number("forward", forward)
            )
            down = self._validate_target("down", self._number("down", down))
        except ValueError as exc:
            raise CameraHeadError(str(exc))
        if forward >= down:
            raise CameraHeadError("forward position must be above down position")
        self.forward_position = forward
        self.down_position = down
        return {"forward_position": forward, "down_position": down}

    def acknowledge_position(self):
        return self.client.acknowledge_tool_position()

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
        if action == "stop":
            return self.client.stop()
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
            return self.set_runtime_positions(request["forward"], request["down"])
        if action == "jog":
            if "degrees" not in request:
                raise CameraHeadError("jog requires degrees")
            return self.jog(request["degrees"])
        if action == "look_forward":
            return self.move_named("forward")
        if action == "look_down":
            return self.move_named("down")
        raise CameraHeadError("unknown camera-head action")

    def describe(self, ev3_status):
        result = {
            "calibrated": self.calibrated,
            "forward_position": self.forward_position,
            "down_position": self.down_position,
            "minimum_position": self.minimum_position,
            "maximum_position": self.maximum_position,
            "speed": self.speed,
            "home_speed": self.home_speed,
            "settle_tolerance": self.settle_tolerance,
        }
        try:
            position = int(ev3_status["motors"]["tool"]["position"])
            result["position"] = position
            if self.calibrated:
                if abs(position - self.forward_position) <= 2:
                    result["named_position"] = "forward"
                elif abs(position - self.down_position) <= 2:
                    result["named_position"] = "down"
                else:
                    result["named_position"] = "between"
        except (KeyError, TypeError, ValueError):
            result["position"] = None
        result["moving"] = bool(ev3_status.get("tool_motion_active", False))
        result["homing"] = bool(ev3_status.get("tool_homing", False))
        result["homed"] = bool(ev3_status.get("tool_homed", False))
        return result
