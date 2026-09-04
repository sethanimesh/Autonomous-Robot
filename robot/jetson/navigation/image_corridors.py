"""Conservative image-space corridors for the first floor-mask prototype."""

from dataclasses import dataclass

from .local_planner import RouteCandidate


@dataclass(frozen=True)
class ImageCorridor:
    heading_degrees: float
    top_y: float
    bottom_y: float
    top_center_x: float
    bottom_center_x: float
    top_half_width: float
    bottom_half_width: float


@dataclass(frozen=True)
class CorridorEvidence:
    heading_degrees: float
    floor_fraction: float
    known_fraction: float
    sample_count: int


# Broad overlapping trapezoids match the 640x480 route view. They are normalized
# so the same test works if the camera image size changes later.
DEFAULT_CORRIDORS = (
    # The calibrated down view reaches the floor near y=0.35. Starting at the
    # old y=0.50 omitted the upper half of a slipper 35 cm from the robot and
    # could incorrectly approve the straight corridor.
    ImageCorridor(-30, 0.35, 0.98, 0.293, 0.227, 0.098, 0.195),
    ImageCorridor(0, 0.35, 0.98, 0.512, 0.500, 0.098, 0.195),
    ImageCorridor(30, 0.35, 0.98, 0.707, 0.781, 0.098, 0.203),
)


def estimate_floor_horizon(
    floor_mask,
    minimum_y=0.20,
    maximum_y=0.75,
    minimum_row_floor_fraction=0.55,
    consecutive_rows=5,
    stride=2,
):
    """Find the first sustained broad floor band in a route-view image."""
    height = len(floor_mask)
    if height == 0:
        raise ValueError("floor mask is empty")
    width = len(floor_mask[0])
    if width == 0 or any(len(row) != width for row in floor_mask):
        raise ValueError("floor mask must be a non-empty rectangle")
    y_start = int(round(minimum_y * (height - 1)))
    y_stop = int(round(maximum_y * (height - 1)))
    x_start = int(round(0.05 * (width - 1)))
    x_stop = int(round(0.95 * (width - 1)))
    run_start = None
    run_length = 0
    for y in range(y_start, y_stop + 1):
        values = [floor_mask[y][x] for x in range(x_start, x_stop + 1, stride)]
        fraction = sum(value is True for value in values) / float(len(values))
        if fraction >= minimum_row_floor_fraction:
            if run_start is None:
                run_start = y
            run_length += 1
            if run_length >= consecutive_rows:
                return run_start / float(height - 1)
        else:
            run_start = None
            run_length = 0
    return None


def trim_corridor_top(corridor, top_y):
    """Trim a trapezoid at the detected horizon without changing its sides."""
    top = min(corridor.bottom_y, max(corridor.top_y, float(top_y)))
    span = corridor.bottom_y - corridor.top_y
    progress = 0.0 if span <= 0 else (top - corridor.top_y) / span
    center = corridor.top_center_x + progress * (
        corridor.bottom_center_x - corridor.top_center_x
    )
    half_width = corridor.top_half_width + progress * (
        corridor.bottom_half_width - corridor.top_half_width
    )
    return ImageCorridor(
        corridor.heading_degrees,
        top,
        corridor.bottom_y,
        center,
        corridor.bottom_center_x,
        half_width,
        corridor.bottom_half_width,
    )


def evaluate_corridor(floor_mask, corridor, stride=2):
    """Measure known and floor pixels inside one perspective trapezoid.

    Mask entries are ``True`` for floor, ``False`` for non-floor and ``None``
    for unknown. Unknown pixels reduce coverage and never count as floor.
    """
    height = len(floor_mask)
    if height == 0:
        raise ValueError("floor mask is empty")
    width = len(floor_mask[0])
    if width == 0 or any(len(row) != width for row in floor_mask):
        raise ValueError("floor mask must be a non-empty rectangle")
    if stride <= 0:
        raise ValueError("stride must be positive")

    y_start = max(0, int(round(corridor.top_y * (height - 1))))
    y_stop = min(height - 1, int(round(corridor.bottom_y * (height - 1))))
    total = known = floor = 0
    for y in range(y_start, y_stop + 1, stride):
        span = max(1, y_stop - y_start)
        progress = (y - y_start) / float(span)
        center = corridor.top_center_x + progress * (
            corridor.bottom_center_x - corridor.top_center_x
        )
        half_width = corridor.top_half_width + progress * (
            corridor.bottom_half_width - corridor.top_half_width
        )
        x_start = max(0, int(round((center - half_width) * (width - 1))))
        x_stop = min(width - 1, int(round((center + half_width) * (width - 1))))
        for x in range(x_start, x_stop + 1, stride):
            total += 1
            value = floor_mask[y][x]
            if value is None:
                continue
            known += 1
            if value is True:
                floor += 1

    return CorridorEvidence(
        heading_degrees=corridor.heading_degrees,
        floor_fraction=0.0 if known == 0 else floor / float(known),
        known_fraction=0.0 if total == 0 else known / float(total),
        sample_count=total,
    )


def semantic_route_candidates(
    floor_mask,
    corridors=DEFAULT_CORRIDORS,
    minimum_floor_fraction=0.95,
    proven_clear_distance_m=0.15,
    minimum_route_y=0.50,
    stride=2,
):
    """Convert the near-floor footprint to bounded short-route candidates."""
    horizon = estimate_floor_horizon(floor_mask)
    active_corridors = (
        corridors
        if horizon is None
        else tuple(
            trim_corridor_top(corridor, max(horizon, minimum_route_y))
            for corridor in corridors
        )
    )
    candidates = []
    evidence = []
    for corridor in active_corridors:
        item = evaluate_corridor(floor_mask, corridor, stride=stride)
        evidence.append(item)
        clear = (
            proven_clear_distance_m
            if horizon is not None and item.floor_fraction >= minimum_floor_fraction
            else 0.0
        )
        candidates.append(
            RouteCandidate(
                heading_degrees=item.heading_degrees,
                clear_distance_m=clear,
                confidence=item.floor_fraction,
                known_fraction=item.known_fraction,
            )
        )
    return tuple(candidates), tuple(evidence)
