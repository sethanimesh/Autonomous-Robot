"""Pure state machine for finding and approaching one target in one room."""

from dataclasses import dataclass
from enum import Enum
import math

from robot.jetson.navigation.local_planner import RouteDecision

from .mission_types import TargetObservation


class MissionState(str, Enum):
    IDLE = "idle"
    SCANNING = "scanning"
    ROUTE_CHECK = "route_check"
    APPROACH_STEP = "approach_step"
    VERIFY_TARGET = "verify_target"
    FOUND = "found"
    STOPPED = "stopped"


class MissionAction(str, Enum):
    NONE = "none"
    STOP = "stop"
    LOOK_FORWARD_AND_SCAN = "look_forward_and_scan"
    LOOK_DOWN_AND_CHECK_ROUTE = "look_down_and_check_route"
    EXECUTE_BOUNDED_ROUTE = "execute_bounded_route"
    ANNOUNCE_FOUND = "announce_found"


@dataclass(frozen=True)
class MissionCommand:
    action: MissionAction
    heading_degrees: float = 0.0
    distance_m: float = 0.0
    reason: str = ""


class SingleRoomMission:
    """Emit explicit actions while all motor execution remains elsewhere."""

    def __init__(
        self,
        maximum_observation_age_seconds=0.75,
        found_box_height_fraction=0.65,
    ):
        if maximum_observation_age_seconds <= 0:
            raise ValueError("observation age limit must be positive")
        if not 0.0 < found_box_height_fraction <= 1.0:
            raise ValueError("found box height fraction must be in (0, 1]")
        self.maximum_observation_age_seconds = float(
            maximum_observation_age_seconds
        )
        self.found_box_height_fraction = float(found_box_height_fraction)
        self.state = MissionState.IDLE
        self.approach_steps = 0
        self.last_reason = ""

    def _command(self, action, reason, heading=0.0, distance=0.0):
        self.last_reason = reason
        return MissionCommand(action, heading, distance, reason)

    def start(self, target_enrolled, camera_ready):
        if self.state not in (MissionState.IDLE, MissionState.STOPPED):
            raise RuntimeError("mission is already active")
        if not target_enrolled:
            self.state = MissionState.STOPPED
            return self._command(MissionAction.STOP, "target is not enrolled")
        if not camera_ready:
            self.state = MissionState.STOPPED
            return self._command(MissionAction.STOP, "camera is not ready")
        self.state = MissionState.SCANNING
        self.approach_steps = 0
        return self._command(
            MissionAction.LOOK_FORWARD_AND_SCAN, "begin bounded room scan"
        )

    def observe_target(self, observation):
        if self.state not in (MissionState.SCANNING, MissionState.VERIFY_TARGET):
            raise RuntimeError("target observation is unexpected in this state")
        if not isinstance(observation, TargetObservation):
            raise TypeError("observation must be TargetObservation")
        values = (observation.age_seconds, observation.box_height_fraction)
        if not all(math.isfinite(value) for value in values):
            return self.abort("target observation is invalid")
        if (
            observation.age_seconds < 0
            or observation.age_seconds > self.maximum_observation_age_seconds
        ):
            return self.abort("target observation is stale")
        if not observation.confirmed:
            self.state = MissionState.SCANNING
            return self._command(
                MissionAction.LOOK_FORWARD_AND_SCAN,
                "target not confirmed; continue bounded scan",
            )
        if observation.box_height_fraction >= self.found_box_height_fraction:
            self.state = MissionState.FOUND
            return self._command(
                MissionAction.ANNOUNCE_FOUND, "target confirmed at standoff"
            )
        self.state = MissionState.ROUTE_CHECK
        return self._command(
            MissionAction.LOOK_DOWN_AND_CHECK_ROUTE,
            "target confirmed; check floor before approaching",
        )

    def approve_route(self, decision, fresh=True):
        if self.state != MissionState.ROUTE_CHECK:
            raise RuntimeError("route result is unexpected in this state")
        if not fresh:
            return self.abort("route result is stale")
        if not isinstance(decision, RouteDecision):
            raise TypeError("decision must be RouteDecision")
        if decision.blocked:
            self.state = MissionState.SCANNING
            return self._command(
                MissionAction.LOOK_FORWARD_AND_SCAN,
                "route blocked; stop and rescan",
            )
        if abs(decision.heading_degrees) > 45.0:
            return self.abort("route heading exceeds mission limit")
        if not 0.05 <= decision.distance_m <= 0.10:
            return self.abort("route distance exceeds mission limit")
        self.state = MissionState.APPROACH_STEP
        return self._command(
            MissionAction.EXECUTE_BOUNDED_ROUTE,
            "execute one approved approach step",
            decision.heading_degrees,
            decision.distance_m,
        )

    def finish_step(self, succeeded):
        if self.state != MissionState.APPROACH_STEP:
            raise RuntimeError("step completion is unexpected in this state")
        if not succeeded:
            return self.abort("approach step failed")
        self.approach_steps += 1
        self.state = MissionState.VERIFY_TARGET
        return self._command(
            MissionAction.LOOK_FORWARD_AND_SCAN,
            "step complete; reacquire target before continuing",
        )

    def abort(self, reason):
        self.state = MissionState.STOPPED
        return self._command(MissionAction.STOP, reason)
