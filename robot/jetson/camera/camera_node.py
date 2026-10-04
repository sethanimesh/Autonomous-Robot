#!/usr/bin/env python3
"""ROS 2 USB camera source for the robot.

Publishes /camera/image_raw, /camera/camera_info and /camera/status from a
V4L2 USB camera. This node only reads the camera; it never sends a motor
command and holds no connection to the EV3.
"""

import array
import json
import os
import time

import cv2
import numpy
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from std_msgs.msg import String

from camera_calibration import resolve_calibration
from camera_config import CameraConfig
from camera_config import PARAMETER_DEFAULTS
from camera_recovery import CameraRestartWatchdog
from frame_health import CaptureHealth
from frame_health import STATE_OPENING
from frame_health import STATE_RECONNECTING
from frame_health import STATE_STOPPED
from frame_health import STATE_STREAMING
from frame_health import STATE_WARMING_UP
from frame_health import validate_frame

IMAGE_ENCODING = "bgr8"
FAILURE_LOG_INTERVAL = 30
HEALTH_SAMPLE_TARGET = 30


def fourcc_text(value):
    """Decode an OpenCV FOURCC integer into its four characters."""
    code = int(value)
    return "".join(chr((code >> (8 * index)) & 0xFF) for index in range(4))


class CameraNode(Node):
    def __init__(self):
        super().__init__("echora_camera")
        for name, default in sorted(PARAMETER_DEFAULTS.items()):
            self.declare_parameter(name, default)
        values = {name: self.get_parameter(name).value for name in PARAMETER_DEFAULTS}
        self.config = CameraConfig.from_mapping(values)

        self.calibration = resolve_calibration(
            self.config.calibration_file,
            self.config.image_width,
            self.config.image_height,
        )
        if self.calibration.is_calibrated:
            self.get_logger().info(
                "loaded camera calibration from {0}".format(self.calibration.source)
            )
        else:
            self.get_logger().warning(
                "publishing UNCALIBRATED CameraInfo ({0}); intrinsics are zeroed and "
                "must not be used for metric vision".format(self.calibration.describe())
            )

        self.health = CaptureHealth(
            self.config.max_read_failures, self.config.reconnect_interval_sec
        )
        self.capture = None
        self.negotiated = {}
        self.warmup_deadline = None
        self.warmup_frames = 0
        self.sample_stride = max(
            1, min(self.config.image_width, self.config.image_height) // HEALTH_SAMPLE_TARGET
        )

        self.image_publisher = self.create_publisher(
            Image, "/camera/image_raw", qos_profile_sensor_data
        )
        self.camera_info_publisher = self.create_publisher(
            CameraInfo, "/camera/camera_info", qos_profile_sensor_data
        )
        self.status_publisher = self.create_publisher(
            String,
            "/camera/status",
            QoSProfile(
                depth=1,
                history=QoSHistoryPolicy.KEEP_LAST,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.camera_info_message = self.build_camera_info()
        self.recovery = CameraRestartWatchdog(
            lambda: self.health._last_frame_time,
            timeout=max(self.config.restart_after_stall_sec, self.config.warmup_sec + 5.),
        )
        self.recovery.start()

        self.create_timer(self.config.capture_timer_period_sec(), self.on_capture_timer)
        self.create_timer(self.config.status_interval_sec, self.on_status_timer)
        self.get_logger().info(
            "camera node ready: device={0} {1}x{2} fourcc={3} requested_fps={4} "
            "frame_id={5}".format(
                self.config.video_device,
                self.config.image_width,
                self.config.image_height,
                self.config.fourcc,
                self.config.requested_fps,
                self.config.frame_id,
            )
        )

    def build_camera_info(self):
        """Build the CameraInfo message once; only its stamp changes per frame."""
        info = CameraInfo()
        info.header.frame_id = self.config.frame_id
        info.width = self.config.image_width
        info.height = self.config.image_height
        info.distortion_model = self.calibration.distortion_model
        info.d = list(self.calibration.distortion_coefficients)
        info.k = self.calibration.camera_matrix
        info.r = self.calibration.rectification_matrix
        info.p = self.calibration.projection_matrix
        return info

    def set_state(self, state):
        if self.health.set_state(state):
            self.publish_status()

    def open_camera(self):
        now = time.monotonic()
        self.health.note_open_attempt(now)
        capture = cv2.VideoCapture(self.config.capture_target(), cv2.CAP_V4L2)
        if not capture.isOpened():
            capture.release()
            self.health.note_open_failed("open_failed")
            return False

        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.config.fourcc))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.image_width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.image_height)
        capture.set(cv2.CAP_PROP_FPS, self.config.requested_fps)

        self.negotiated = {
            "width": int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
            "height": int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            "fps": round(float(capture.get(cv2.CAP_PROP_FPS)), 2),
            "fourcc": fourcc_text(capture.get(cv2.CAP_PROP_FOURCC)),
        }
        self.capture = capture
        self.health.note_opened()
        self.warmup_deadline = time.monotonic() + self.config.warmup_sec
        self.warmup_frames = 0

        if (
            self.negotiated["width"] != self.config.image_width
            or self.negotiated["height"] != self.config.image_height
        ):
            self.get_logger().error(
                "camera negotiated {0}x{1} but {2}x{3} was requested; frames will be "
                "rejected until the configuration matches a supported mode".format(
                    self.negotiated["width"],
                    self.negotiated["height"],
                    self.config.image_width,
                    self.config.image_height,
                )
            )
        elif self.negotiated["fourcc"] != self.config.fourcc:
            self.get_logger().warning(
                "camera negotiated fourcc {0} instead of {1}; the sustained rate may "
                "be lower than requested".format(
                    self.negotiated["fourcc"], self.config.fourcc
                )
            )
        self.get_logger().info(
            "opened {0} as {1} (open #{2}); discarding frames for {3:.1f}s while "
            "exposure settles".format(
                self.config.video_device,
                self.negotiated,
                self.health.open_count,
                self.config.warmup_sec,
            )
        )
        self.set_state(STATE_WARMING_UP)
        return True

    def close_camera(self):
        if self.capture is not None:
            try:
                self.capture.release()
            except Exception as exc:  # pragma: no cover - OpenCV teardown guard
                self.get_logger().warning("camera release raised: {0}".format(exc))
            self.capture = None
        self.warmup_deadline = None
        self.health.note_closed()

    def on_capture_timer(self):
        if self.capture is None:
            self.attempt_reconnect()
            return

        ok, frame = self.capture.read()
        if not ok or frame is None:
            self.health.record_read_failure("read_failed")
            self.after_failure()
            return

        if self.warmup_deadline is not None:
            self.warmup_frames += 1
            if time.monotonic() < self.warmup_deadline:
                return
            self.warmup_deadline = None
            # This frame ends the warm-up and is published, so it is not one of
            # the discarded ones.
            self.get_logger().info(
                "warm-up complete after {0} discarded frames; publishing".format(
                    self.warmup_frames - 1
                )
            )
            self.set_state(STATE_STREAMING)

        reason = validate_frame(frame, self.config.image_width, self.config.image_height)
        if reason is not None:
            self.health.record_rejected_frame(reason)
            self.after_failure()
            return

        self.publish_frame(frame)

    def attempt_reconnect(self):
        now = time.monotonic()
        if not self.health.reconnect_ready(now):
            time.sleep(min(0.05, self.health.seconds_until_reconnect(now)))
            return
        self.set_state(STATE_OPENING)
        if self.open_camera():
            return
        self.set_state(STATE_RECONNECTING)
        if self.health.open_failures == 1 or self.health.open_failures % FAILURE_LOG_INTERVAL == 0:
            self.get_logger().error(
                "cannot open {0} (attempt {1}); retrying every {2:.1f}s".format(
                    self.config.video_device,
                    self.health.open_failures,
                    self.config.reconnect_interval_sec,
                )
            )

    def after_failure(self):
        """React to a failed read or a rejected frame without spinning."""
        if self.health.needs_reopen():
            self.get_logger().error(
                "releasing {0} after {1} consecutive failures (last: {2})".format(
                    self.config.video_device,
                    self.health.consecutive_read_failures,
                    self.health.last_error,
                )
            )
            self.close_camera()
            self.set_state(STATE_RECONNECTING)
            return
        if self.health.read_failures % FAILURE_LOG_INTERVAL == 1:
            self.get_logger().warning(
                "camera read problem: {0} (consecutive {1}/{2})".format(
                    self.health.last_error,
                    self.health.consecutive_read_failures,
                    self.config.max_read_failures,
                )
            )
        if self.config.read_failure_pause_sec > 0.0:
            time.sleep(self.config.read_failure_pause_sec)

    def publish_frame(self, frame):
        stamp = self.get_clock().now().to_msg()

        message = Image()
        message.header.stamp = stamp
        message.header.frame_id = self.config.frame_id
        message.height = self.config.image_height
        message.width = self.config.image_width
        message.encoding = IMAGE_ENCODING
        message.is_bigendian = 0
        message.step = self.config.step_bytes()
        message.data = array.array("B", frame.tobytes())
        self.image_publisher.publish(message)

        self.camera_info_message.header.stamp = stamp
        self.camera_info_publisher.publish(self.camera_info_message)

        sample = frame[:: self.sample_stride, :: self.sample_stride, 1]
        self.health.record_frame(
            time.monotonic(),
            signature=sample.tobytes(),
            mean_intensity=float(numpy.mean(sample)),
        )

    def on_status_timer(self):
        self.publish_status()

    def publish_status(self):
        if not rclpy.ok():
            return
        payload = self.health.status(time.monotonic())
        payload.update(
            {
                "device": self.config.video_device,
                "width": self.config.image_width,
                "height": self.config.image_height,
                "encoding": IMAGE_ENCODING,
                "frame_id": self.config.frame_id,
                "requested_fps": self.config.requested_fps,
                "requested_fourcc": self.config.fourcc,
                "restart_after_stall_sec": self.config.restart_after_stall_sec,
                "negotiated": self.negotiated,
                "calibrated": self.calibration.is_calibrated,
                "calibration": self.calibration.describe(),
            }
        )
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
        self.status_publisher.publish(message)

    def shutdown(self):
        self.recovery.close()
        self.health.set_state(STATE_STOPPED)
        self.publish_status()
        self.close_camera()
        summary = (
            "camera released; published {0} frames, rejected {1}, read failures {2}, "
            "opens {3}".format(
                self.health.frames_published,
                self.health.frames_rejected,
                self.health.read_failures,
                self.health.open_count,
            )
        )
        # After Ctrl-C the rclpy signal handler has already invalidated the
        # context, so /rosout is gone. Fall back to stdout, which journald and
        # a foreground terminal both still capture.
        if rclpy.ok():
            self.get_logger().info(summary)
        else:
            print("[INFO] [echora_camera]: {0}".format(summary), flush=True)


def configure_camera_transport():
    """Avoid the stalled local transport unless an operator profile is set."""
    if any(name in os.environ for name in (
        'FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE'
    )):
        return
    profile = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'camera_transport.xml')
    if os.path.isfile(profile):
        os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = profile


def main(args=None):
    configure_camera_transport()
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
