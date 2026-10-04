"""Differential-drive conversion helpers."""

import math


def twist_to_motor_speeds(
    linear_mps,
    angular_rps,
    wheel_radius_m,
    track_width_m,
    left_sign=1,
    right_sign=1,
    max_motor_speed=240,
):
    """Convert chassis velocity to EV3 motor degrees per second."""

    if wheel_radius_m <= 0:
        raise ValueError("wheel_radius_m must be positive")
    if track_width_m <= 0:
        raise ValueError("track_width_m must be positive")
    if left_sign not in (-1, 1) or right_sign not in (-1, 1):
        raise ValueError("motor signs must be -1 or 1")
    if max_motor_speed <= 0:
        raise ValueError("max_motor_speed must be positive")

    left_linear = linear_mps - angular_rps * track_width_m / 2.0
    right_linear = linear_mps + angular_rps * track_width_m / 2.0
    radians_to_degrees = 180.0 / math.pi
    left_speed = left_sign * left_linear / wheel_radius_m * radians_to_degrees
    right_speed = right_sign * right_linear / wheel_radius_m * radians_to_degrees

    left_speed = int(round(max(-max_motor_speed, min(max_motor_speed, left_speed))))
    right_speed = int(
        round(max(-max_motor_speed, min(max_motor_speed, right_speed)))
    )
    return left_speed, right_speed
