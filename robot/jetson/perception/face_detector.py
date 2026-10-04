#!/usr/bin/env python3
"""ROS 2 YuNet face detector gated by exact-frame YOLOX person boxes.

The node performs face detection only. It does not identify people, retain
frames, persist landmarks, control motors, or connect to the EV3.
"""

import array
import json
import time

import cv2
import numpy
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import BoundingBox2D, Detection2D, Detection2DArray, ObjectHypothesisWithPose

from detector_health import (
    DetectorHealth,
    LatestFrameSlot,
    STATE_DETECTING,
    STATE_LOADING_MODEL,
    STATE_MODEL_ERROR,
    STATE_STALE_INPUT,
    STATE_STOPPED,
    STATE_WAITING_FOR_FRAMES,
)
from face_config import FaceDetectorConfig, PARAMETER_DEFAULTS
from face_detections import map_face_to_source, select_faces
from face_inference import FaceInferenceError, TensorRTMultiOutputBackend, YuNetFaceDetector
from face_observations import serialize_face_observations
from face_sync import ExactPairMatcher, select_person_regions, stamp_key
from image_subscription import ReconnectingImageSubscription, configure_perception_transport
from image_intake import REJECT_MALFORMED, REJECT_STALE_FRAME, SUPPORTED_ENCODINGS, frame_age_seconds, is_frame_too_old, validate_image_message
from messages import MessageFactories, build_detection_array

BOX_COLOUR = (255, 180, 0)
LANDMARK_COLOURS = ((255, 0, 0), (0, 0, 255), (0, 255, 0), (255, 0, 255), (0, 255, 255))
FONT = cv2.FONT_HERSHEY_SIMPLEX
ERROR_LOG_INTERVAL = 20


class CameraFrame(object):
    __slots__ = ("image", "stamp", "frame_id")

    def __init__(self, image, stamp, frame_id):
        self.image = image
        self.stamp = stamp
        self.frame_id = frame_id


class PendingBatch(object):
    __slots__ = ("frame", "regions")

    def __init__(self, frame, regions):
        self.frame = frame
        self.regions = regions


class FaceDetectorNode(Node):
    def __init__(self):
        super().__init__("echora_face_detector")
        for name, default in sorted(PARAMETER_DEFAULTS.items()):
            self.declare_parameter(name, default)
        values = {name: self.get_parameter(name).value for name in PARAMETER_DEFAULTS}
        self.config = FaceDetectorConfig.from_mapping(values)

        self.health = DetectorHealth(
            self.config.model_name, "none", latency_window=self.config.latency_window
        )
        self.matcher = ExactPairMatcher(self.config.image_cache_size)
        self.slot = LatestFrameSlot()
        self.detector = None
        self.person_messages_received = 0
        self.matched_person_messages = 0
        self.no_person_batches = 0
        self.person_rois_processed = 0
        self.full_frame_fallbacks_run = 0
        self._last_full_frame_fallback_at = 0.0
        self._last_person_message_time = None
        self.latest_frame = None
        self._last_annotated_time = 0.0
        self.factories = MessageFactories(
            Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose
        )

        status_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        sensor_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.status_publisher = self.create_publisher(String, self.config.status_topic, status_qos)
        self.detections_publisher = self.create_publisher(
            Detection2DArray, self.config.face_detections_topic, 10
        )
        self.observations_publisher = self.create_publisher(
            String, self.config.face_observations_topic, 10
        )
        self.annotated_publisher = None
        if self.config.publish_annotated_image:
            self.annotated_publisher = self.create_publisher(
                Image, self.config.annotated_image_topic, sensor_qos
            )

        self.set_state(STATE_LOADING_MODEL)
        self.load_model()
        self.image_input = ReconnectingImageSubscription(self, Image, self.config.image_topic, self.on_image, sensor_qos)
        self.person_input = ReconnectingImageSubscription(
            self,
            Detection2DArray,
            self.config.person_detections_topic,
            self.on_person_detections,
            10,
        )
        self.create_timer(self.config.inference_period_sec(), self.on_inference_timer)
        self.create_timer(self.config.status_interval_sec, self.on_status_timer)
        self.set_state(STATE_WAITING_FOR_FRAMES)
        self.get_logger().info(
            "face detector ready: model={0} provider={1} device={2}; exact-frame person ROI gate on {3}".format(
                self.config.model_name,
                self.detector.provider,
                self.detector.device_name,
                self.config.person_detections_topic,
            )
        )

    def load_model(self):
        started = time.monotonic()
        try:
            backend = TensorRTMultiOutputBackend(
                self.config.model_path, self.config.model_input_shape()
            )
            self.detector = YuNetFaceDetector(backend, self.config.confidence_threshold)
        except Exception as exc:
            self.set_state(STATE_MODEL_ERROR)
            self.health.last_error = "{0}: {1}".format(type(exc).__name__, exc)
            self.publish_status()
            raise FaceInferenceError(str(exc))
        load_seconds = time.monotonic() - started
        self.health.note_model_loaded(
            self.detector.provider, load_seconds, self.detector.warmup_seconds
        )

    def on_image(self, message):
        now = time.monotonic()
        self.health.record_frame(now)
        reason = validate_image_message(
            message.encoding,
            message.width,
            message.height,
            len(message.data),
            SUPPORTED_ENCODINGS,
        )
        if reason is not None:
            self.health.record_rejected_frame(reason)
            return
        try:
            image = numpy.frombuffer(message.data, dtype=numpy.uint8).reshape(
                message.height, message.width, 3
            )
        except (TypeError, ValueError):
            self.health.record_rejected_frame(REJECT_MALFORMED)
            return
        if message.encoding == "rgb8":
            image = image[:, :, ::-1]
        frame = CameraFrame(image, message.header.stamp, message.header.frame_id)
        self.latest_frame = frame
        pair = self.matcher.offer_left(stamp_key(message.header.stamp), frame)
        if pair is not None:
            self.queue_pair(*pair)

    def on_person_detections(self, message):
        self.person_messages_received += 1
        self._last_person_message_time = time.monotonic()
        pair = self.matcher.offer_right(stamp_key(message.header.stamp), message)
        if pair is not None:
            self.queue_pair(*pair)

    def queue_pair(self, frame, message):
        if message.header.frame_id != frame.frame_id:
            self.health.record_rejected_frame("frame_id_mismatch")
            return
        self.matched_person_messages += 1
        height, width = frame.image.shape[:2]
        regions = select_person_regions(
            message.detections,
            width,
            height,
            self.config.max_person_rois,
            self.config.person_roi_padding,
            self.config.person_roi_height_fraction,
            self.config.minimum_person_roi_pixels,
        )
        self.slot.offer(PendingBatch(frame, regions))

    def on_inference_timer(self):
        now = time.monotonic()
        if self.health.input_is_stale(now, self.config.frame_timeout_sec):
            if self.set_state(STATE_STALE_INPUT):
                self.get_logger().warning("camera or person-detection input is stale; waiting")
            self.slot.clear()
            self.matcher.clear()
            return
        batch = self.slot.take()
        if self.person_input_is_stale(now) and self.config.enable_full_frame_fallback:
            if self.latest_frame is None or now-self._last_full_frame_fallback_at < 1./self.config.full_frame_fallback_rate_hz:
                return
            batch = PendingBatch(self.latest_frame, [])
        if batch is None:
            return
        age = frame_age_seconds(
            batch.frame.stamp.sec,
            batch.frame.stamp.nanosec,
            self.get_clock().now().nanoseconds * 1e-9,
        )
        if is_frame_too_old(age, self.config.max_frame_age_sec):
            self.health.record_rejected_frame(REJECT_STALE_FRAME)
            return
        if self.set_state(STATE_DETECTING):
            self.get_logger().info("matched camera/person frames are arriving; detecting faces")
            self.health.note_input_resumed()

        if not batch.regions:
            self.no_person_batches += 1
            self.health.recent_person_count = 0

        started = time.monotonic()
        mapped = []
        try:
            height, width = batch.frame.image.shape[:2]
            for region in batch.regions:
                crop = batch.frame.image[region.y1 : region.y2, region.x1 : region.x2]
                if crop.size == 0:
                    continue
                candidates = self.detector.infer(crop)
                self.person_rois_processed += 1
                for candidate in candidates:
                    mapped.append(
                        map_face_to_source(
                            candidate,
                            region,
                            region.width,
                            region.height,
                            width,
                            height,
                        )
                    )
            # Partial/occluded seated bodies can produce a valid YOLO region
            # whose crop excludes a clearly visible face.  When all bounded
            # person crops return no face, try the whole frame at a separately
            # capped rate.  This remains one region and never persists pixels.
            fallback_interval = 1.0 / self.config.full_frame_fallback_rate_hz
            if (
                not mapped
                and self.config.enable_full_frame_fallback
                and time.monotonic() - self._last_full_frame_fallback_at
                >= fallback_interval
            ):
                full_regions = select_person_regions(
                    (),
                    width,
                    height,
                    1,
                    0.0,
                    1.0,
                    self.config.minimum_person_roi_pixels,
                    fallback_full_frame=True,
                )
                if full_regions:
                    region = full_regions[0]
                    candidates = self.detector.infer(batch.frame.image)
                    self._last_full_frame_fallback_at = time.monotonic()
                    self.full_frame_fallbacks_run += 1
                    for candidate in candidates:
                        mapped.append(
                            map_face_to_source(
                                candidate,
                                region,
                                region.width,
                                region.height,
                                width,
                                height,
                            )
                        )
        except Exception as exc:  # one frame must never kill the managed service
            self.record_inference_error(exc)
            return
        faces = select_faces(mapped, self.config.nms_iou_threshold, self.config.max_faces)
        self.health.record_inference(time.monotonic(), time.monotonic() - started, len(faces))
        self.publish_detections(faces, batch.frame)
        self.publish_annotated(faces, batch.frame)

    def record_inference_error(self, exc):
        self.health.record_inference_error("{0}: {1}".format(type(exc).__name__, exc))
        if self.health.inference_errors == 1 or self.health.inference_errors % ERROR_LOG_INTERVAL == 0:
            self.get_logger().error("face inference failed: {0}: {1}".format(type(exc).__name__, exc))

    def publish_detections(self, faces, frame):
        self.detections_publisher.publish(
            build_detection_array(faces, frame.stamp, frame.frame_id, self.factories)
        )
        observations = String()
        observations.data = serialize_face_observations(
            faces, frame.stamp, frame.frame_id
        )
        self.observations_publisher.publish(observations)

    def annotation_wanted(self):
        if self.annotated_publisher is None:
            return False
        if time.monotonic() - self._last_annotated_time < 1.0 / self.config.max_annotated_rate_hz:
            return False
        return (
            not self.config.annotate_only_when_subscribed
            or self.annotated_publisher.get_subscription_count() > 0
        )

    def publish_annotated(self, faces, frame):
        if not self.annotation_wanted():
            return
        canvas = frame.image.copy()
        for face in faces:
            x1, y1, x2, y2 = [int(round(v)) for v in (face.x1, face.y1, face.x2, face.y2)]
            cv2.rectangle(canvas, (x1, y1), (x2, y2), BOX_COLOUR, 2)
            cv2.putText(
                canvas,
                "face {0:.2f}".format(face.score),
                (x1, max(18, y1 - 6)),
                FONT,
                0.5,
                BOX_COLOUR,
                1,
                cv2.LINE_AA,
            )
            for point, colour in zip(face.landmarks, LANDMARK_COLOURS):
                cv2.circle(canvas, (int(round(point[0])), int(round(point[1]))), 2, colour, -1)
        message = Image()
        message.header.stamp = frame.stamp
        message.header.frame_id = frame.frame_id
        message.height, message.width = canvas.shape[:2]
        message.encoding = "bgr8"
        message.is_bigendian = False
        message.step = message.width * 3
        message.data = array.array("B", canvas.reshape(-1))
        self.annotated_publisher.publish(message)
        self._last_annotated_time = time.monotonic()
        self.health.record_annotated()

    def person_input_is_stale(self, now):
        if self._last_person_message_time is None:
            return False
        return now - self._last_person_message_time > self.config.person_timeout_sec

    def set_state(self, state):
        changed = self.health.set_state(state)
        if changed:
            self.publish_status()
        return changed

    def on_status_timer(self):
        now = time.monotonic()
        if self.image_input.refresh_if_stale(now, self.health.seconds_since_frame(now)):
            self.get_logger().warning('Reconnected camera input after ten seconds without frames')
        person_age = None if self._last_person_message_time is None else now - self._last_person_message_time
        if self.person_input.refresh_if_stale(now, person_age):
            self.matcher.clear()
            self.slot.clear()
            self.get_logger().warning('Reconnected person detections after ten seconds without messages')
        if self.health.state == STATE_DETECTING and (
            self.health.input_is_stale(now, self.config.frame_timeout_sec)
            or self.person_input_is_stale(now)
        ):
            self.set_state(STATE_STALE_INPUT)
        self.publish_status()

    def publish_status(self):
        if not rclpy.ok():
            return
        payload = self.health.status(
            time.monotonic(),
            dropped_frames=self.slot.dropped,
            extra={
                "image_subscription_reconnects": getattr(getattr(self, 'image_input', None), 'reconnects', 0),
                "person_subscription_reconnects": getattr(getattr(self, 'person_input', None), 'reconnects', 0),
                "last_person_message_age_sec": None if self._last_person_message_time is None else round(time.monotonic() - self._last_person_message_time, 3),
                "image_topic": self.config.image_topic,
                "person_detections_topic": self.config.person_detections_topic,
                "face_detections_topic": self.config.face_detections_topic,
                "face_observations_topic": self.config.face_observations_topic,
                "model_path": self.config.model_path,
                "device_name": None if self.detector is None else self.detector.device_name,
                "input_size": "{0}x{1}".format(self.config.model_input_width, self.config.model_input_height),
                "confidence_threshold": self.config.confidence_threshold,
                "max_inference_rate_hz": self.config.max_inference_rate_hz,
                "max_annotated_rate_hz": self.config.max_annotated_rate_hz,
                "person_messages_received": self.person_messages_received,
                "matched_person_messages": self.matched_person_messages,
                "unmatched_messages_evicted": self.matcher.evicted,
                "cached_images_evicted": self.matcher.left_evicted,
                "cached_person_messages_evicted": self.matcher.right_evicted,
                "image_cache_depth": self.matcher.left_depth,
                "person_cache_depth": self.matcher.right_depth,
                "person_rois_processed": self.person_rois_processed,
                "no_person_batches": self.no_person_batches,
                "enable_full_frame_fallback": self.config.enable_full_frame_fallback,
                "full_frame_fallback_rate_hz": self.config.full_frame_fallback_rate_hz,
                "full_frame_fallbacks_run": self.full_frame_fallbacks_run,
                "recent_face_count": self.health.recent_person_count,
                "privacy": "no frames or biometric data persisted; landmarks are transient",
            },
        )
        payload.pop("recent_person_count", None)
        if not self.config.enable_timing_diagnostics:
            for key in ("mean_latency_ms", "p95_latency_ms", "worst_latency_ms"):
                payload.pop(key, None)
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
        self.status_publisher.publish(message)

    def shutdown(self):
        self.health.set_state(STATE_STOPPED)
        # The ROS context is commonly already invalid when systemd's SIGINT
        # unwinds rclpy.spin(). Publishing here caused a false failure on an
        # otherwise clean managed restart, and a transient status publisher
        # has no useful subscriber after this process exits anyway.
        if self.detector is not None:
            self.detector.close()
            self.detector = None
        self.matcher.clear()
        self.slot.clear()
        print(
            "[INFO] [echora_face_detector]: stopped; matched={0}, inferences={1}, faces_last={2}, errors={3}".format(
                self.matched_person_messages,
                self.health.inferences,
                self.health.recent_person_count,
                self.health.inference_errors,
            ),
            flush=True,
        )


def main(args=None):
    configure_perception_transport()
    rclpy.init(args=args)
    node = None
    try:
        node = FaceDetectorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
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
