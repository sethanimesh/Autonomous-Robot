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
    ImageCorridor(-30, 0.50, 0.98, 0.293, 0.227, 0.098, 0.195),
    ImageCorridor(0, 0.50, 0.98, 0.512, 0.500, 0.098, 0.195),
    ImageCorridor(30, 0.50, 0.98, 0.707, 0.781, 0.098, 0.203),
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
    minimum_floor_fraction=0.90,
    proven_clear_distance_m=0.15,
    stride=2,
):
    """Convert floor masks to candidates; failed corridors have zero clearance."""
    candidates = []
    evidence = []
    for corridor in corridors:
        item = evaluate_corridor(floor_mask, corridor, stride=stride)
        evidence.append(item)
        clear = (
            proven_clear_distance_m
            if item.floor_fraction >= minimum_floor_fraction
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

