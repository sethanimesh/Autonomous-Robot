"""Absolute turn accounting for a chassis tethered by an external cable."""

import math


class CableLimitError(RuntimeError):
    """Raised before a turn that could over-twist the external cable."""


def _finite(name, value):
    value = float(value)
    if not math.isfinite(value):
        raise CableLimitError("{0} must be finite".format(name))
    return value


def validate_cable_heading(heading_degrees, limit_degrees=120.0, margin=0.0):
    """Validate an absolute heading measured from the marked cable-neutral pose."""

    heading = _finite("cable heading", heading_degrees)
    limit = _finite("cable limit", limit_degrees)
    margin = _finite("cable margin", margin)
    if limit <= 0.0 or limit > 180.0:
        raise CableLimitError("cable limit must be in (0, 180]")
    if margin < 0.0 or margin >= limit:
        raise CableLimitError("cable margin must be non-negative and below the limit")
    usable_limit = limit - margin
    if abs(heading) > usable_limit + 1e-6:
        raise CableLimitError(
            "cable heading {0:.1f} exceeds the safe +/-{1:.1f} envelope".format(
                heading, usable_limit
            )
        )
    return heading


def project_cable_turn(
    current_heading_degrees,
    relative_turn_degrees,
    limit_degrees=120.0,
    margin=5.0,
):
    """Return the absolute heading after a turn, or reject it before motion."""

    current = validate_measured_cable_heading(current_heading_degrees, limit_degrees, margin)
    turn = _finite("relative cable turn", relative_turn_degrees)
    projected = validate_measured_cable_heading(current + turn, limit_degrees, margin)
    # Allow an inward correction from a small measured excursion into reserve.
    # Crossing to the opposite reserve is not an inward correction.
    if current * projected >= 0 and abs(projected) <= abs(current):
        return projected
    return validate_cable_heading(projected, limit_degrees, margin)


def validate_measured_cable_heading(heading_degrees, limit_degrees=120.0, margin=0.0):
    """Retain actual heading inside the absolute limit, including its reserve."""
    validate_cable_heading(0, limit_degrees, margin)  # Validate configuration too.
    return validate_cable_heading(heading_degrees, limit_degrees)


def record_cable_turn(
    current_heading_degrees,
    requested_turn_degrees,
    actual_turn_degrees,
    limit_degrees=120.0,
    maximum_turn_error_degrees=8.0,
    margin=0.0,
):
    """Validate measured turn direction/error and update absolute cable heading."""

    current = validate_measured_cable_heading(
        current_heading_degrees, limit_degrees, margin
    )
    requested = _finite("requested cable turn", requested_turn_degrees)
    actual = _finite("actual cable turn", actual_turn_degrees)
    maximum_error = _finite("maximum cable turn error", maximum_turn_error_degrees)
    if maximum_error < 0.0:
        raise CableLimitError("maximum cable turn error cannot be negative")
    if requested and actual and requested * actual < 0.0:
        raise CableLimitError("chassis turned opposite to the cable-safe request")
    if abs(actual - requested) > maximum_error:
        raise CableLimitError("measured chassis turn differs too far from its request")
    return validate_measured_cable_heading(current + actual, limit_degrees, margin)


def relative_scan_headings_within_cable_limit(
    relative_headings,
    initial_heading_degrees=0.0,
    limit_degrees=120.0,
    margin=5.0,
):
    """Drop unsafe scan samples while retaining a return to the scan origin."""

    initial = validate_measured_cable_heading(initial_heading_degrees, limit_degrees, margin)
    safe = []
    for relative in relative_headings:
        try:
            validate_cable_heading(initial + float(relative), limit_degrees, margin)
        except CableLimitError:
            continue
        value = float(relative)
        if not safe or value != safe[-1]:
            safe.append(value)
    if not safe or safe[0] != 0.0:
        safe.insert(0, 0.0)
    # Observe the current pose once, but do not plan an outward return to a
    # scan origin that settled into reserve. Finish at cable-neutral instead.
    finish = 0.0 if abs(initial) <= limit_degrees - margin else -initial
    if safe[-1] != finish:
        safe.append(finish)
    return tuple(safe)
