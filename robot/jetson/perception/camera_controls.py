"""Validation helpers for the temporary camera-head browser controls."""

# Motor A's encoder counts increase as the lens tilts down. Positive aims the
# lens down and negative aims it up. Re-confirmed on 2026-09-04 with a settled
# image sequence: 0 showed only ceiling, +52/+66 crossed the ceiling edge, and
# +81 showed the room forward. The low-speed homing attempt at 0 did not move the
# loaded head and was not valid direction evidence. Anything that turns a
# direction into encoder degrees must go through
# CAMERA_UP_SIGN / CAMERA_DOWN_SIGN rather than hard-coding a sign.
CAMERA_UP_SIGN = -1
CAMERA_DOWN_SIGN = 1

CAMERA_FINE_JOG_DEGREES = 5
CAMERA_COARSE_JOG_DEGREES = 15
CAMERA_JOG_DEGREES = CAMERA_FINE_JOG_DEGREES
CAMERA_ALLOWED_JOG_DEGREES = (
    -CAMERA_COARSE_JOG_DEGREES,
    -CAMERA_FINE_JOG_DEGREES,
    CAMERA_FINE_JOG_DEGREES,
    CAMERA_COARSE_JOG_DEGREES,
)
CAMERA_STATUS_MAX_AGE_SECONDS = 2.0


def validate_camera_jog(status, status_age_seconds, degrees):
    """Return the absolute target for one manual tilt step.

    A step that would run past a software limit is trimmed to that limit rather
    than refused, so the last few degrees stay reachable even when the head has
    drifted off a round number. Only a head already parked on the limit is
    rejected, because there is nowhere left for it to go.
    """

    if degrees not in CAMERA_ALLOWED_JOG_DEGREES:
        raise ValueError(
            "Camera tilt accepts only a {0}-degree or {1}-degree step.".format(
                CAMERA_FINE_JOG_DEGREES, CAMERA_COARSE_JOG_DEGREES
            )
        )
    if not isinstance(status, dict) or status_age_seconds is None:
        raise ValueError("Camera-head status is unavailable.")
    if status_age_seconds < 0 or status_age_seconds > CAMERA_STATUS_MAX_AGE_SECONDS:
        raise ValueError("Camera-head status is stale.")
    if not status.get("homed"):
        raise ValueError("Camera head must be homed before manual tilt.")
    if status.get("moving") or status.get("homing"):
        raise ValueError("Camera head is already moving.")
    try:
        position = int(status["position"])
        minimum = int(status["minimum_position"])
        maximum = int(status["maximum_position"])
    except (KeyError, TypeError, ValueError):
        raise ValueError("Camera-head position limits are unavailable.")
    if minimum > maximum:
        raise ValueError("Camera-head limits are inconsistent.")
    target = min(maximum, max(minimum, position + degrees))
    if target == position:
        raise ValueError("The camera head is already at that limit.")
    return target


def jog_degrees(direction, magnitude=CAMERA_FINE_JOG_DEGREES):
    """Encoder degrees for one step in a named direction.

    Callers say "up" or "down" and never spell out a sign, so the convention
    recorded at the top of this module stays the only place it is written down.
    """

    signs = {"up": CAMERA_UP_SIGN, "down": CAMERA_DOWN_SIGN}
    if direction not in signs:
        raise ValueError("Camera tilt direction must be 'up' or 'down'.")
    if magnitude not in (CAMERA_FINE_JOG_DEGREES, CAMERA_COARSE_JOG_DEGREES):
        raise ValueError(
            "Camera tilt magnitude must be {0} or {1} degrees.".format(
                CAMERA_FINE_JOG_DEGREES, CAMERA_COARSE_JOG_DEGREES
            )
        )
    return signs[direction] * magnitude
