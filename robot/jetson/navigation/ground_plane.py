"""Anchor relative monocular depth to the robot's known floor geometry."""

from dataclasses import dataclass
import math
import statistics


@dataclass(frozen=True)
class GroundPlaneFit:
    pitch_degrees: float
    depth_scale: float
    median_log_error: float
    sample_count: int

    def accepted(self, maximum_median_log_error=0.08, minimum_samples=100):
        return (
            self.sample_count >= minimum_samples
            and self.median_log_error <= maximum_median_log_error
        )


def expected_floor_z_m(pixel_y, focal_y, center_y, pitch_degrees, lens_height_m):
    """Return optical-axis depth to a flat floor for one image row."""
    values = (pixel_y, focal_y, center_y, pitch_degrees, lens_height_m)
    if not all(math.isfinite(value) for value in values):
        return None
    if focal_y <= 0 or lens_height_m <= 0:
        return None
    pitch = math.radians(pitch_degrees)
    vertical_ray = (pixel_y - center_y) / focal_y
    denominator = vertical_ray * math.cos(pitch) + math.sin(pitch)
    if denominator <= 0.05:
        return None
    # Model depth is optical-axis Z. Ground-forward distance is a separate
    # rotation: Z * (cos(pitch) - vertical_ray * sin(pitch)).
    optical_z = lens_height_m / denominator
    return optical_z if optical_z > 0.02 else None


def fit_ground_plane(
    samples,
    focal_y,
    center_y,
    lens_height_m,
    minimum_pitch_degrees=2.0,
    maximum_pitch_degrees=40.0,
    pitch_step_degrees=0.25,
):
    """Fit camera pitch and one depth scale from presumed floor samples.

    ``samples`` contains ``(pixel_y, raw_model_depth)`` pairs from a central,
    lower image region. The robust median fit tolerates isolated obstacle pixels.
    """
    if pitch_step_degrees <= 0:
        raise ValueError("pitch step must be positive")
    if minimum_pitch_degrees >= maximum_pitch_degrees:
        raise ValueError("pitch search range is empty")
    clean = []
    for pixel_y, raw_depth in samples:
        if (
            math.isfinite(pixel_y)
            and math.isfinite(raw_depth)
            and raw_depth > 0
        ):
            clean.append((float(pixel_y), float(raw_depth)))
    if len(clean) < 3:
        raise ValueError("at least three valid depth samples are required")

    best = None
    pitch = float(minimum_pitch_degrees)
    while pitch <= maximum_pitch_degrees + 1e-9:
        log_ratios = []
        for pixel_y, raw_depth in clean:
            expected = expected_floor_z_m(
                pixel_y, focal_y, center_y, pitch, lens_height_m
            )
            if expected is not None:
                log_ratios.append(math.log(expected) - math.log(raw_depth))
        if len(log_ratios) >= 3:
            offset = statistics.median(log_ratios)
            error = statistics.median(
                abs(value - offset) for value in log_ratios
            )
            candidate = (error, pitch, math.exp(offset), len(log_ratios))
            if best is None or candidate < best:
                best = candidate
        pitch += pitch_step_degrees

    if best is None:
        raise ValueError("no floor rays intersected the ground plane")
    return GroundPlaneFit(
        pitch_degrees=best[1],
        depth_scale=best[2],
        median_log_error=best[0],
        sample_count=best[3],
    )


def floor_depth_ratio(
    pixel_y, raw_depth, fit, focal_y, center_y, lens_height_m
):
    """Return scaled observed depth / expected flat-floor depth."""
    expected = expected_floor_z_m(
        pixel_y, focal_y, center_y, fit.pitch_degrees, lens_height_m
    )
    if expected is None or not math.isfinite(raw_depth) or raw_depth <= 0:
        return None
    return raw_depth * fit.depth_scale / expected

