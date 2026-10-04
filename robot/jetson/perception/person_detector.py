#!/usr/bin/env python3
"""ROS 2 person detector for the robot.

Subscribes to the live camera stream, runs a YOLOX person detector on the
Jetson GPU through TensorRT, and publishes standards-based detections, an
optional annotated image, and honest health information.

This node only looks. It performs generic person detection and nothing else:
no face detection, no recognition, no identity, no tracking across frames, no
following, and no motor command. It holds no connection to the EV3 and it
never writes a camera frame to disk.

The decision logic lives in sibling modules with no ROS, numpy, OpenCV, or
TensorRT imports, so it is covered by the repository test suite. This file
owns the pixels, the CUDA calls, and the ROS plumbing.
"""

import array
import json
import time

import cv2
import numpy
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import BoundingBox2D
from vision_msgs.msg import Detection2D
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import ObjectHypothesisWithPose

from detections import select_person_detections
from detector_config import DetectorConfig
from detector_config import PARAMETER_DEFAULTS
from detector_health import DetectorHealth
from detector_health import LatestFrameSlot
from detector_health import STATE_DETECTING
from detector_health import STATE_LOADING_MODEL
from detector_health import STATE_MODEL_ERROR
from detector_health import STATE_STALE_INPUT
from detector_health import STATE_STOPPED
from detector_health import STATE_WAITING_FOR_FRAMES
from image_subscription import ReconnectingImageSubscription, configure_perception_transport
from image_intake import REJECT_MALFORMED
from image_intake import REJECT_STALE_FRAME
from image_intake import SUPPORTED_ENCODINGS
from image_intake import frame_age_seconds
from image_intake import is_frame_too_old
from image_intake import validate_image_message
from inference import InferenceError
from inference import PersonDetector
from inference import open_backend
from messages import MessageFactories
from messages import annotation_plan
from messages import build_detection_array

IMAGE_ENCODING = "bgr8"
BOX_COLOUR = (0, 220, 0)
TEXT_COLOUR = (0, 0, 0)
FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE = 0.5
FONT_THICKNESS = 1
REJECT_LOG_INTERVAL = 100
ERROR_LOG_INTERVAL = 20


class PendingFrame(object):
    """One camera frame waiting for inference."""

    __slots__ = ("image", "stamp", "frame_id", "age_sec")

    def __init__(self, image, stamp, frame_id, age_sec):
        self.image = image
        self.stamp = stamp
        self.frame_id = frame_id
        self.age_sec = age_sec


class PersonDetectorNode(Node):
    def __init__(self):
        super().__init__("echora_person_detector")
        for name, default in sorted(PARAMETER_DEFAULTS.items()):
            self.declare_parameter(name, default)
        values = {name: self.get_parameter(name).value for name in PARAMETER_DEFAULTS}
        self.config = DetectorConfig.from_mapping(values)

        self.health = DetectorHealth(
            self.config.model_name,
            "none",
            latency_window=self.config.latency_window,
        )
        self.slot = LatestFrameSlot()
        self.detector = None
        self.fallback_reason = ""
        self.factories = MessageFactories(
            Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose
        )

        self.status_publisher = self.create_publisher(
            String,
            self.config.status_topic,
            QoSProfile(
                depth=1,
                history=QoSHistoryPolicy.KEEP_LAST,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.detections_publisher = self.create_publisher(
            Detection2DArray, self.config.detections_topic, 10
        )
        self.annotated_publisher = None
        if self.config.publish_annotated_image:
            self.annotated_publisher = self.create_publisher(
                Image,
                self.config.annotated_image_topic,
                QoSProfile(
                    depth=1,
                    history=QoSHistoryPolicy.KEEP_LAST,
                    reliability=QoSReliabilityPolicy.BEST_EFFORT,
                ),
            )

        self.set_state(STATE_LOADING_MODEL)
        self.load_model()

        # Depth 1 best-effort: the middleware keeps only the newest frame, and
        # the one-deep slot below guarantees it a second time.
        self.image_input = ReconnectingImageSubscription(
            self, Image,
            self.config.image_topic,
            self.on_image,
            QoSProfile(
                depth=1,
                history=QoSHistoryPolicy.KEEP_LAST,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
            ),
        )
        self.create_timer(self.config.inference_period_sec(), self.on_inference_timer)
        self.create_timer(self.config.status_interval_sec, self.on_status_timer)

        self.set_state(STATE_WAITING_FOR_FRAMES)
        self.get_logger().info(
            "person detector ready: model={0} provider={1} device={2} input={3}x{4} "
            "conf={5} iou={6} max_rate={7}Hz topic={8}".format(
                self.config.model_name,
                self.detector.provider,
                self.detector.device_name,
                self.config.model_input_width,
                self.config.model_input_height,
                self.config.confidence_threshold,
                self.config.nms_iou_threshold,
                self.config.max_inference_rate_hz,
                self.config.image_topic,
            )
        )

    # -- model ---------------------------------------------------------

    def load_model(self):
        started = time.monotonic()
        try:
            backend, fallback_reason = open_backend(
                self.config.inference_device,
                self.config.model_path,
                self.config.onnx_path,
                self.config.model_input_shape(),
                logger=self.get_logger().warning,
            )
            self.fallback_reason = fallback_reason
            self.detector = PersonDetector(
                backend,
                self.config.model_input_height,
                self.config.model_input_width,
                self.config.confidence_threshold,
            )
        except InferenceError as exc:
            self.set_state(STATE_MODEL_ERROR)
            self.health.last_error = str(exc)
            self.publish_status()
            self.get_logger().fatal("model load failed: {0}".format(exc))
            raise
        load_seconds = time.monotonic() - started
        self.health.note_model_loaded(
            self.detector.provider, load_seconds, self.detector.warmup_seconds
        )
        self.get_logger().info(
            "loaded {0} in {1:.2f}s (warm-up {2:.2f}s) on provider={3} device={4}".format(
                self.config.model_path,
                load_seconds,
                self.detector.warmup_seconds,
                self.detector.provider,
                self.detector.device_name,
            )
        )
        if self.fallback_reason:
            self.get_logger().warning(
                "running on the CPU fallback because TensorRT failed: {0}".format(
                    self.fallback_reason
                )
            )

    # -- input ---------------------------------------------------------

    def on_image(self, message):
        """Store the newest frame. Deliberately cheap: no inference happens here."""
        self.health.record_frame(time.monotonic())

        reason = validate_image_message(
            message.encoding,
            message.width,
            message.height,
            len(message.data),
            SUPPORTED_ENCODINGS,
        )
        if reason is not None:
            self.reject(
                reason,
                "rejecting frame: {0} (encoding={1!r} {2}x{3} payload={4}B)".format(
                    reason,
                    message.encoding,
                    message.width,
                    message.height,
                    len(message.data),
                ),
            )
            return

        try:
            image = numpy.frombuffer(message.data, dtype=numpy.uint8).reshape(
                message.height, message.width, 3
            )
        except (TypeError, ValueError) as exc:
            self.reject(REJECT_MALFORMED, "cannot shape image: {0}".format(exc))
            return

        if message.encoding == "rgb8":
            image = image[:, :, ::-1]

        age = frame_age_seconds(
            message.header.stamp.sec,
            message.header.stamp.nanosec,
            self.get_clock().now().nanoseconds * 1e-9,
        )
        self.slot.offer(
            PendingFrame(image, message.header.stamp, message.header.frame_id, age)
        )

    def reject(self, reason, detail):
        self.health.record_rejected_frame(reason)
        count = self.health.reject_reasons.get(reason, 0)
        if count == 1 or count % REJECT_LOG_INTERVAL == 0:
            self.get_logger().warning("{0} (occurrence {1})".format(detail, count))

    # -- inference -----------------------------------------------------

    def on_inference_timer(self):
        now = time.monotonic()

        if self.health.input_is_stale(now, self.config.frame_timeout_sec):
            if self.set_state(STATE_STALE_INPUT):
                self.get_logger().warning(
                    "no camera frame for {0:.1f}s on {1}; waiting for the stream to "
                    "return".format(
                        self.health.seconds_since_frame(now), self.config.image_topic
                    )
                )
            self.slot.clear()
            return

        frame = self.slot.take()
        if frame is None:
            return

        if is_frame_too_old(frame.age_sec, self.config.max_frame_age_sec):
            self.reject(
                REJECT_STALE_FRAME,
                "dropping a frame {0:.3f}s old; max_frame_age_sec is {1}".format(
                    frame.age_sec, self.config.max_frame_age_sec
                ),
            )
            return

        if self.set_state(STATE_DETECTING):
            self.get_logger().info(
                "camera frames are arriving; detecting on {0}".format(
                    self.config.image_topic
                )
            )
            self.health.note_input_resumed()

        started = time.monotonic()
        try:
            candidates, ratio = self.detector.infer_candidates(frame.image)
        except Exception as exc:  # noqa: BLE001 - one bad frame must not kill the node
            self.record_inference_error(exc)
            return
        latency = time.monotonic() - started

        height, width = frame.image.shape[:2]
        detections = select_person_detections(
            candidates,
            self.config.person_class_id,
            self.config.confidence_threshold,
            self.config.nms_iou_threshold,
            width,
            height,
            ratio,
            max_detections=self.config.max_detections,
            label=self.config.person_class_label,
        )

        self.health.record_inference(time.monotonic(), latency, len(detections))
        self.publish_detections(detections, frame)
        self.publish_annotated(detections, frame)

    def record_inference_error(self, exc):
        self.health.record_inference_error("{0}: {1}".format(type(exc).__name__, exc))
        if (
            self.health.inference_errors == 1
            or self.health.inference_errors % ERROR_LOG_INTERVAL == 0
        ):
            self.get_logger().error(
                "inference failed ({0} total): {1}: {2}".format(
                    self.health.inference_errors, type(exc).__name__, exc
                )
            )

    # -- output --------------------------------------------------------

    def publish_detections(self, detections, frame):
        message = build_detection_array(
            detections, frame.stamp, frame.frame_id, self.factories
        )
        self.detections_publisher.publish(message)

    def annotation_wanted(self):
        if self.annotated_publisher is None:
            return False
        if not self.config.annotate_only_when_subscribed:
            return True
        return self.annotated_publisher.get_subscription_count() > 0

    @staticmethod
    def measure_text(caption):
        (text_width, text_height), baseline = cv2.getTextSize(
            caption, FONT, FONT_SCALE, FONT_THICKNESS
        )
        return text_width + 4, text_height + baseline

    def publish_annotated(self, detections, frame):
        """Publish an annotated copy, skipping the work when nobody subscribes."""
        if not self.annotation_wanted():
            return

        height, width = frame.image.shape[:2]
        # The subscription buffer is read-only, so one writable copy is
        # unavoidable. It is made only when a subscriber actually exists.
        canvas = frame.image.copy()

        for box in annotation_plan(detections, width, height, self.measure_text):
            cv2.rectangle(canvas, (box.x1, box.y1), (box.x2, box.y2), BOX_COLOUR, 2)
            cv2.rectangle(
                canvas,
                (box.label_x, box.label_y),
                (box.label_x + box.label_width, box.label_y + box.label_height),
                BOX_COLOUR,
                -1,
            )
            cv2.putText(
                canvas,
                box.caption,
                (box.label_x + 2, box.label_y + box.label_height - 3),
                FONT,
                FONT_SCALE,
                TEXT_COLOUR,
                FONT_THICKNESS,
                cv2.LINE_AA,
            )

        message = Image()
        message.header.stamp = frame.stamp
        message.header.frame_id = frame.frame_id
        message.height = height
        message.width = width
        message.encoding = IMAGE_ENCODING
        message.is_bigendian = 0
        message.step = width * 3
        # array.array is required here: filling this field with bytes costs
        # ~152 ms per frame on this rclpy build, as measured for the camera node.
        message.data = array.array("B", canvas.tobytes())
        self.annotated_publisher.publish(message)
        self.health.record_annotated()

    # -- status --------------------------------------------------------

    def set_state(self, state):
        changed = self.health.set_state(state)
        if changed:
            self.publish_status()
        return changed

    def on_status_timer(self):
        now = time.monotonic()
        if self.image_input.refresh_if_stale(now, self.health.seconds_since_frame(now)):
            self.get_logger().warning('Reconnected camera input after ten seconds without frames')
        if self.health.state == STATE_DETECTING and self.health.input_is_stale(
            now, self.config.frame_timeout_sec
        ):
            self.set_state(STATE_STALE_INPUT)
        self.publish_status()

    def publish_status(self):
        if not rclpy.ok():
            return
        extra = {
            "image_subscription_reconnects": getattr(getattr(self, 'image_input', None), 'reconnects', 0),
            "image_topic": self.config.image_topic,
            "model_path": self.config.model_path,
            "input_size": "{0}x{1}".format(
                self.config.model_input_width, self.config.model_input_height
            ),
            "confidence_threshold": self.config.confidence_threshold,
            "nms_iou_threshold": self.config.nms_iou_threshold,
            "max_inference_rate_hz": self.config.max_inference_rate_hz,
            "max_frame_age_sec": self.config.max_frame_age_sec,
            "annotated_enabled": self.config.publish_annotated_image,
            "requested_device": self.config.inference_device,
        }
        if self.detector is not None:
            extra["device_name"] = self.detector.device_name
        if self.fallback_reason:
            extra["fallback_reason"] = self.fallback_reason

        payload = self.health.status(
            time.monotonic(), dropped_frames=self.slot.dropped, extra=extra
        )
        if not self.config.enable_timing_diagnostics:
            for key in ("mean_latency_ms", "p95_latency_ms", "worst_latency_ms"):
                payload.pop(key, None)
            payload["timing_diagnostics"] = "disabled"

        message = String()
        message.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
        self.status_publisher.publish(message)

    # -- teardown ------------------------------------------------------

    def shutdown(self):
        self.health.set_state(STATE_STOPPED)
        self.publish_status()
        if self.detector is not None:
            try:
                self.detector.close()
            except Exception as exc:  # pragma: no cover - teardown guard
                print("[WARN] releasing inference resources raised: {0}".format(exc))
            self.detector = None
        self.slot.clear()
        summary = (
            "person detector stopped; frames {0}, inferences {1}, rejected {2}, "
            "dropped {3}, errors {4}".format(
                self.health.frames_received,
                self.health.inferences,
                self.health.frames_rejected,
                self.slot.dropped,
                self.health.inference_errors,
            )
        )
        # After Ctrl-C the rclpy signal handler has already invalidated the
        # context, so /rosout is gone; stdout is still captured by journald.
        if rclpy.ok():
            self.get_logger().info(summary)
        else:
            print("[INFO] [echora_person_detector]: {0}".format(summary), flush=True)


def main(args=None):
    configure_perception_transport()
    rclpy.init(args=args)
    node = None
    try:
        node = PersonDetectorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # SIGINT invalidates the context underneath the executor, which
        # surfaces from the wait set as an RCLError rather than as
        # KeyboardInterrupt. Once the context is gone that is a normal
        # shutdown; while it is still valid the error is real, so re-raise.
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
