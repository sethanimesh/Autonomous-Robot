"""Small, deterministic route selector for stop-look-move navigation.

Perception supplies clearance for footprint-wide candidate corridors.  This
module deliberately contains no camera, model, ROS, or motor code, so uncertain
perception can be tested independently and always fails closed.
"""

from dataclasses import dataclass
import math


def normalize_angle_degrees(angle):
    """Return an angle in [-180, 180]."""
    value = (float(angle) + 180.0) % 360.0 - 180.0
    return 180.0 if value == -180.0 and angle > 0 else value


def cable_safe_scan_headings(step_degrees=30, sweep_limit_degrees=180):
    """Generate a complete scan that unwinds to its starting heading.

    The attached external cable is never twisted past ``sweep_limit_degrees``:
    scan right, unwind, scan left, then unwind again.
    """
    step = int(step_degrees)
    limit = int(sweep_limit_degrees)
    if step <= 0 or step > 90:
        raise ValueError("scan step must be between 1 and 90 degrees")
    if limit <= 0 or limit > 180 or limit % step:
        raise ValueError("sweep limit must be <= 180 and divisible by scan step")

    positive = list(range(0, limit + 1, step))
    positive_unwind = list(range(limit - step, -1, -step))
    negative = list(range(-step, -limit - 1, -step))
    negative_unwind = list(range(-limit + step, 1, step))
    return positive + positive_unwind + negative + negative_unwind


@dataclass(frozen=True)
class RouteCandidate:
    """Clearance evidence for one turn-then-drive candidate."""

    heading_degrees: float
    clear_distance_m: float
    confidence: float
    known_fraction: float


@dataclass(frozen=True)
class RouteDecision:
    """One bounded motion primitive, or a fail-closed blocked result."""

    blocked: bool
    heading_degrees: float = 0.0
    distance_m: float = 0.0
    score: float = float("-inf")
    reason: str = ""


class LocalRoutePlanner:
    """Choose one short route from already footprint-inflated corridors."""

    def __init__(
        self,
        robot_width_m=0.20,
        robot_length_m=0.25,
        safety_margin_m=0.05,
        minimum_step_m=0.05,
        maximum_step_m=0.10,
        maximum_heading_degrees=45.0,
        minimum_confidence=0.70,
        minimum_known_fraction=0.85,
    ):
        values = (
            robot_width_m,
            robot_length_m,
            safety_margin_m,
            minimum_step_m,
            maximum_step_m,
        )
        if not all(math.isfinite(value) and value > 0 for value in values):
            raise ValueError("robot geometry and distances must be positive")
        if minimum_step_m > maximum_step_m:
            raise ValueError("minimum step cannot exceed maximum step")
        if maximum_heading_degrees <= 0 or maximum_heading_degrees > 90:
            raise ValueError("maximum heading must be in (0, 90]")
        if not 0.0 <= minimum_confidence <= 1.0:
            raise ValueError("minimum confidence must be in [0, 1]")
        if not 0.0 <= minimum_known_fraction <= 1.0:
            raise ValueError("minimum known fraction must be in [0, 1]")

        self.robot_width_m = float(robot_width_m)
        self.robot_length_m = float(robot_length_m)
        self.safety_margin_m = float(safety_margin_m)
        self.minimum_step_m = float(minimum_step_m)
        self.maximum_step_m = float(maximum_step_m)
        self.maximum_heading_degrees = float(maximum_heading_degrees)
        self.minimum_confidence = float(minimum_confidence)
        self.minimum_known_fraction = float(minimum_known_fraction)

    @property
    def corridor_width_m(self):
        """Width perception must prove clear for every candidate."""
        return self.robot_width_m + 2.0 * self.safety_margin_m

    def choose(self, candidates, desired_heading_degrees=0.0):
        desired = normalize_angle_degrees(desired_heading_degrees)
        viable = []
        for candidate in candidates:
            if not isinstance(candidate, RouteCandidate):
                raise TypeError("candidates must be RouteCandidate instances")
            numbers = (
                candidate.heading_degrees,
                candidate.clear_distance_m,
                candidate.confidence,
                candidate.known_fraction,
            )
            if not all(math.isfinite(value) for value in numbers):
                continue
            heading = normalize_angle_degrees(candidate.heading_degrees)
            if abs(heading) > self.maximum_heading_degrees:
                continue
            if candidate.confidence < self.minimum_confidence:
                continue
            if candidate.known_fraction < self.minimum_known_fraction:
                continue

            usable_distance = candidate.clear_distance_m - self.safety_margin_m
            if usable_distance < self.minimum_step_m:
                continue
            distance = min(self.maximum_step_m, usable_distance)
            alignment_error = abs(normalize_angle_degrees(heading - desired))
            score = (
                4.0 * distance
                + 0.10 * min(candidate.clear_distance_m, 0.50)
                + 0.25 * candidate.confidence
                + 0.25 * candidate.known_fraction
                - 0.012 * alignment_error
                - 0.003 * abs(heading)
            )
            viable.append((score, -alignment_error, distance, -abs(heading), heading))

        if not viable:
            return RouteDecision(blocked=True, reason="no proven footprint-wide route")

        score, _, distance, _, heading = max(viable)
        return RouteDecision(
            blocked=False,
            heading_degrees=heading,
            distance_m=distance,
            score=score,
            reason="best proven short route",
        )
