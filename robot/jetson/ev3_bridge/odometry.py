"""Differential-drive encoder odometry and calibration helpers."""

import math


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class DifferentialOdometry(object):
    def __init__(
        self,
        wheel_radius_m,
        track_width_m,
        counts_per_revolution=360,
        left_sign=1,
        right_sign=1,
    ):
        if wheel_radius_m <= 0 or track_width_m <= 0:
            raise ValueError("wheel radius and track width must be positive")
        if counts_per_revolution <= 0:
            raise ValueError("counts_per_revolution must be positive")
        if left_sign not in (-1, 1) or right_sign not in (-1, 1):
            raise ValueError("encoder signs must be -1 or 1")

        self.wheel_radius_m = float(wheel_radius_m)
        self.track_width_m = float(track_width_m)
        self.counts_per_revolution = float(counts_per_revolution)
        self.left_sign = int(left_sign)
        self.right_sign = int(right_sign)
        self.radians_per_count = 2.0 * math.pi / self.counts_per_revolution
        self.reset()

    def reset(self):
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.last_left_count = None
        self.last_right_count = None
        self.last_timestamp = None

    def update(self, left_count, right_count, timestamp_seconds):
        left_count = int(left_count)
        right_count = int(right_count)
        timestamp_seconds = float(timestamp_seconds)

        if self.last_left_count is None:
            self.last_left_count = left_count
            self.last_right_count = right_count
            self.last_timestamp = timestamp_seconds
            return self.state(left_count, right_count, 0.0, 0.0)

        left_delta_counts = left_count - self.last_left_count
        right_delta_counts = right_count - self.last_right_count
        left_radians = left_delta_counts * self.radians_per_count * self.left_sign
        right_radians = right_delta_counts * self.radians_per_count * self.right_sign
        left_distance = left_radians * self.wheel_radius_m
        right_distance = right_radians * self.wheel_radius_m
        distance = (left_distance + right_distance) / 2.0
        heading_change = (right_distance - left_distance) / self.track_width_m

        midpoint_heading = self.heading + heading_change / 2.0
        self.x += distance * math.cos(midpoint_heading)
        self.y += distance * math.sin(midpoint_heading)
        self.heading = normalize_angle(self.heading + heading_change)

        elapsed = timestamp_seconds - self.last_timestamp
        if elapsed > 0:
            linear_velocity = distance / elapsed
            angular_velocity = heading_change / elapsed
        else:
            linear_velocity = 0.0
            angular_velocity = 0.0

        self.last_left_count = left_count
        self.last_right_count = right_count
        self.last_timestamp = timestamp_seconds
        return self.state(
            left_count, right_count, linear_velocity, angular_velocity
        )

    def state(self, left_count, right_count, linear_velocity, angular_velocity):
        return {
            "x": self.x,
            "y": self.y,
            "heading": self.heading,
            "linear_velocity": linear_velocity,
            "angular_velocity": angular_velocity,
            "left_joint_position": left_count
            * self.radians_per_count
            * self.left_sign,
            "right_joint_position": right_count
            * self.radians_per_count
            * self.right_sign,
        }


def estimate_wheel_radius(
    measured_distance_m, left_delta_counts, right_delta_counts, counts_per_revolution=360
):
    """Estimate effective wheel radius from a measured straight run."""

    average_counts = (abs(left_delta_counts) + abs(right_delta_counts)) / 2.0
    if measured_distance_m <= 0 or average_counts <= 0 or counts_per_revolution <= 0:
        raise ValueError("distance, encoder change, and counts per revolution must be positive")
    wheel_radians = average_counts * 2.0 * math.pi / counts_per_revolution
    return measured_distance_m / wheel_radians


def estimate_track_width(
    measured_yaw_radians,
    left_delta_counts,
    right_delta_counts,
    wheel_radius_m,
    counts_per_revolution=360,
):
    """Estimate effective track width from a measured in-place turn."""

    if measured_yaw_radians == 0 or wheel_radius_m <= 0 or counts_per_revolution <= 0:
        raise ValueError("yaw, wheel radius, and counts per revolution must be non-zero")
    radians_per_count = 2.0 * math.pi / counts_per_revolution
    left_distance = left_delta_counts * radians_per_count * wheel_radius_m
    right_distance = right_delta_counts * radians_per_count * wheel_radius_m
    return abs((right_distance - left_distance) / measured_yaw_radians)
