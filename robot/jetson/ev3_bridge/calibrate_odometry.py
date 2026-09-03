#!/usr/bin/env python3
"""Capture bounded real-motion encoder data or calculate odometry geometry."""

import argparse
import copy
import json
import sys
import time

try:
    from .calibration import CALIBRATION_REPORT_VERSION
    from .calibration import CalibrationError
    from .calibration import calibration_result
    from .calibration import encoder_deltas
    from .calibration import load_report
    from .calibration import motor_positions
    from .calibration import validate_capture
    from .calibration import validate_encoder_motion
    from .calibration import write_report
except (ImportError, ValueError):
    from calibration import CALIBRATION_REPORT_VERSION
    from calibration import CalibrationError
    from calibration import calibration_result
    from calibration import encoder_deltas
    from calibration import load_report
    from calibration import motor_positions
    from calibration import validate_capture
    from calibration import validate_encoder_motion
    from calibration import write_report


DEFAULT_REPORT = "/home/animesh/echora/calibration/latest_motion.json"
COMMAND_INTERVAL_SECONDS = 0.1


def print_json(value):
    print(json.dumps(value, indent=2, sort_keys=True))


def capture(options):
    validate_capture(options.motion, options.speed, options.duration)

    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from std_msgs.msg import String

    class CaptureNode(Node):
        def __init__(self):
            super().__init__("echora_odometry_calibration")
            self.status = None
            self.status_count = 0
            self.odom = None
            # Keep only the newest command so a stop cannot sit behind a burst
            # of stale motion messages in the DDS queue.
            self.publisher = self.create_publisher(Twist, "/cmd_vel", 1)
            self.create_subscription(String, "/robot_status", self.on_status, 10)
            self.create_subscription(Odometry, "/odom", self.on_odom, 10)

        def on_status(self, message):
            try:
                value = json.loads(message.data)
                motor_positions(value)
            except (ValueError, CalibrationError):
                return
            self.status = value
            self.status_count += 1

        def on_odom(self, message):
            self.odom = {
                "x": float(message.pose.pose.position.x),
                "y": float(message.pose.pose.position.y),
                "orientation_z": float(message.pose.pose.orientation.z),
                "orientation_w": float(message.pose.pose.orientation.w),
            }

        def command(self, moving):
            message = Twist()
            if moving and options.motion == "straight":
                message.linear.x = float(options.speed)
            elif moving:
                message.angular.z = float(options.speed)
            self.publisher.publish(message)

    report = {
        "version": CALIBRATION_REPORT_VERSION,
        "outcome": "failure",
        "motion": options.motion,
        "commanded_speed": float(options.speed),
        "commanded_duration_seconds": float(options.duration),
        "encoder_counts_per_rev": int(options.encoder_counts_per_rev),
        "wheel_radius_m": float(options.wheel_radius_m),
        "track_width_m": float(options.track_width_m),
        "started_at_unix": time.time(),
    }
    rclpy.init()
    node = CaptureNode()
    try:
        wait_deadline = time.monotonic() + options.wait_timeout
        while node.status is None and time.monotonic() < wait_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.status is None:
            raise CalibrationError("no valid /robot_status received; EV3 may be offline")
        if node.status.get("motion_active"):
            raise CalibrationError("EV3 already reports active motion")

        start_status = copy.deepcopy(node.status)
        start_odom = copy.deepcopy(node.odom)
        motion_deadline = time.monotonic() + options.duration
        next_command_at = time.monotonic()
        while time.monotonic() < motion_deadline:
            now = time.monotonic()
            if now >= next_command_at:
                node.command(True)
                next_command_at = now + COMMAND_INTERVAL_SECONDS
            remaining = min(motion_deadline - now, next_command_at - now)
            rclpy.spin_once(node, timeout_sec=max(0.0, min(0.05, remaining)))

        status_count_before_stop = node.status_count
        stop_deadline = time.monotonic() + options.stop_timeout
        stopped = False
        next_stop_at = time.monotonic()
        while time.monotonic() < stop_deadline:
            now = time.monotonic()
            if now >= next_stop_at:
                node.command(False)
                next_stop_at = now + COMMAND_INTERVAL_SECONDS
            remaining = min(stop_deadline - now, next_stop_at - now)
            rclpy.spin_once(node, timeout_sec=max(0.0, min(0.05, remaining)))
            if (
                node.status_count > status_count_before_stop
                and not node.status.get("motion_active", True)
            ):
                stopped = True
                break
        if not stopped:
            raise CalibrationError("stop was sent but stopped status was not confirmed")

        end_status = copy.deepcopy(node.status)
        left_delta, right_delta = encoder_deltas(start_status, end_status)
        validate_encoder_motion(
            options.motion,
            left_delta,
            right_delta,
            options.minimum_encoder_counts,
        )

        report.update(
            {
                "outcome": "success",
                "finished_at_unix": time.time(),
                "encoder_start": {
                    "left": motor_positions(start_status)[0],
                    "right": motor_positions(start_status)[1],
                },
                "encoder_end": {
                    "left": motor_positions(end_status)[0],
                    "right": motor_positions(end_status)[1],
                },
                "encoder_delta": {"left": left_delta, "right": right_delta},
                "odom_start": start_odom,
                "odom_end": copy.deepcopy(node.odom),
                "stop_confirmed": True,
            }
        )
    except Exception as exc:
        report["finished_at_unix"] = time.time()
        report["error"] = str(exc)
        try:
            for _ in range(5):
                node.command(False)
                rclpy.spin_once(node, timeout_sec=0.05)
        except Exception:
            pass
    finally:
        write_report(options.report, report)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print_json(report)
    return 0 if report["outcome"] == "success" else 2


def calculate(options):
    report = load_report(options.report)
    result = calibration_result(
        report,
        measured_distance_m=options.measured_distance_m,
        measured_yaw_degrees=options.measured_yaw_degrees,
        wheel_radius_m=options.wheel_radius_m,
    )
    result["source_report"] = options.report
    result["measurement"] = (
        {"distance_m": options.measured_distance_m}
        if options.measured_distance_m is not None
        else {"yaw_degrees": options.measured_yaw_degrees}
    )
    print_json(result)
    return 0


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="action")
    subparsers.required = True

    capture_parser = subparsers.add_parser("capture")
    capture_parser.add_argument("--motion", choices=("straight", "turn"), required=True)
    capture_parser.add_argument("--speed", type=float, required=True)
    capture_parser.add_argument("--duration", type=float, required=True)
    capture_parser.add_argument("--report", default=DEFAULT_REPORT)
    capture_parser.add_argument("--wheel-radius-m", type=float, default=0.03)
    capture_parser.add_argument("--track-width-m", type=float, default=0.12)
    capture_parser.add_argument("--encoder-counts-per-rev", type=int, default=360)
    capture_parser.add_argument("--minimum-encoder-counts", type=int, default=10)
    capture_parser.add_argument("--wait-timeout", type=float, default=5.0)
    capture_parser.add_argument("--stop-timeout", type=float, default=2.0)

    calculate_parser = subparsers.add_parser("calculate")
    calculate_parser.add_argument("--report", default=DEFAULT_REPORT)
    measurement = calculate_parser.add_mutually_exclusive_group(required=True)
    measurement.add_argument("--measured-distance-m", type=float)
    measurement.add_argument("--measured-yaw-degrees", type=float)
    calculate_parser.add_argument(
        "--wheel-radius-m",
        type=float,
        help="override the capture radius when recalculating track width",
    )
    return parser.parse_args(argv)


def main(argv=None):
    options = parse_args(argv)
    try:
        if options.action == "capture":
            return capture(options)
        return calculate(options)
    except (CalibrationError, OSError, ValueError) as exc:
        print("error: {0}".format(exc), file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
