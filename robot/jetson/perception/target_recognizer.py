#!/usr/bin/env python3
"""GPU target-person recognizer with multi-template, multi-frame confirmation."""

import array
import json
import os
import time

from family_store import FamilyTargetStore
from family_faces import FamilyFaceMatcher

import numpy
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import BoundingBox2D, Detection2D, Detection2DArray, ObjectHypothesisWithPose

from detections import Detection
from face_inference import TensorRTMultiOutputBackend
from face_observations import FaceObservationError, parse_face_observations
from face_sync import ExactPairMatcher, stamp_key
from image_subscription import ReconnectingImageSubscription, configure_perception_transport
from image_intake import SUPPORTED_ENCODINGS, frame_age_seconds, is_frame_too_old, validate_image_message
from messages import MessageFactories, build_detection_array
from recognition_config import PARAMETER_DEFAULTS, RecognitionConfig
from recognition_core import ConfirmationWindow, FaceEmbeddingRecognizer, TargetStore, aggregate_similarity


class Frame(object):
    __slots__ = ("image", "stamp", "frame_id")

    def __init__(self, image, stamp, frame_id):
        self.image = image
        self.stamp = stamp
        self.frame_id = frame_id


class TargetRecognizerNode(Node):
    def __init__(self):
        super().__init__("echora_target_recognizer")
        for name, default in sorted(PARAMETER_DEFAULTS.items()):
            self.declare_parameter(name, default)
        values = {name: self.get_parameter(name).value for name in PARAMETER_DEFAULTS}
        self.config = RecognitionConfig.from_mapping(values)
        self.matcher = ExactPairMatcher(self.config.image_cache_size)
        self.pending = None
        self.pending_dropped = 0
        self.last_input_time = None
        self.last_image_time = None
        self.last_observation_time = None
        self.last_error = None
        self.inferences = 0
        self.inference_errors = 0
        self.last_score = None
        self.last_latency_ms = None
        self.confirmed = False
        self.matches_published = 0
        self.last_match_frame = None
        self.started = time.monotonic()
        self._store_mtime = None
        self.target = None

        backend = TensorRTMultiOutputBackend(self.config.model_path, (112, 112))
        self.recognizer = FaceEmbeddingRecognizer(
            backend, self.config.input_mean, self.config.input_std
        )
        self.store = TargetStore(
            self.config.target_store_path, self.config.model_name, self.config.model_sha256
        )
        self.store = FamilyTargetStore(self.store)
        self.family_matcher = FamilyFaceMatcher(self.config.match_threshold, .05, self.config.confirmation_required, self.config.confirmation_window)
        self.history = ConfirmationWindow(
            self.config.confirmation_window,
            self.config.confirmation_required,
            self.config.match_threshold,
        )
        self.reload_target(force=True)

        reliable_latched = QoSProfile(
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
        self.status_publisher = self.create_publisher(String, self.config.status_topic, reliable_latched)
        self.family_publisher = self.create_publisher(String, "/perception/family_matches", 10)
        self.matches_publisher = self.create_publisher(Detection2DArray, self.config.matches_topic, 10)
        self.factories = MessageFactories(Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose)
        self.image_input = ReconnectingImageSubscription(self, Image, self.config.image_topic, self.on_image, sensor_qos)
        self.observation_input = ReconnectingImageSubscription(self, String, self.config.face_observations_topic, self.on_observations, 10)
        self.create_timer(self.config.inference_period_sec(), self.on_inference)
        self.create_timer(self.config.status_interval_sec, self.on_status)
        self.publish_status()
        self.get_logger().info(
            "target recognizer ready: model={0} provider={1}; target={2}".format(
                self.config.model_name,
                self.recognizer.provider,
                "loaded" if self.target else "not enrolled",
            )
        )

    def on_image(self, message):
        reason = validate_image_message(message.encoding, message.width, message.height, len(message.data), SUPPORTED_ENCODINGS)
        if reason is not None:
            return
        try:
            image = numpy.frombuffer(message.data, dtype=numpy.uint8).reshape(message.height, message.width, 3)
        except (TypeError, ValueError):
            return
        if message.encoding == "rgb8":
            image = image[:, :, ::-1]
        self.last_image_time = time.monotonic()
        frame = Frame(image, message.header.stamp, message.header.frame_id)
        pair = self.matcher.offer_left(stamp_key(message.header.stamp), frame)
        if pair is not None:
            self.queue_pair(*pair)

    def on_observations(self, message):
        self.last_observation_time = time.monotonic()
        try:
            observations = parse_face_observations(message.data)
        except FaceObservationError as exc:
            self.last_error = str(exc)
            return
        pair = self.matcher.offer_right(observations["key"], observations)
        if pair is not None:
            self.queue_pair(*pair)

    def queue_pair(self, frame, observations):
        if frame.frame_id != observations["frame_id"]:
            return
        self.last_input_time = time.monotonic()
        if self.pending is not None:
            self.pending_dropped += 1
        self.pending = (frame, observations)

    def on_inference(self):
        self.reload_target()
        batch = self.pending
        self.pending = None
        if batch is None:
            return
        frame, observations = batch
        age = frame_age_seconds(
            frame.stamp.sec,
            frame.stamp.nanosec,
            self.get_clock().now().nanoseconds * 1e-9,
        )
        if is_frame_too_old(age, self.config.max_frame_age_sec):
            return
        match_frame = dict(sec=int(frame.stamp.sec), nanosec=int(frame.stamp.nanosec),
                           frame_id=frame.frame_id)
        if match_frame == self.last_match_frame:
            return  # A repeated full-frame fallback is still one observation.
        selected = observations["faces"][: self.config.max_faces]
        detections = []
        best_score = None
        started = time.monotonic()
        family_faces = []
        profiles = [self.store.family.load(p["id"]) for p in self.store.family.profiles()] if hasattr(self.store, "family") else []
        try:
            if self.target is not None:
                templates = [sample["embedding"] for sample in self.target["samples"]]
                for face in selected:
                    embedding, _aligned = self.recognizer.embedding(frame.image, face["landmarks"])
                    scores = [dict(profile_id=p['profile_id'],revision=p['revision'],
                        score=aggregate_similarity(embedding,[s['embedding'] for s in p['samples']]))
                        for p in profiles if p['model'] == self.target['model']]
                    family_faces.append(dict(box=list(face['box']),scores=scores))
                    score = aggregate_similarity(embedding, templates)
                    if best_score is None or score > best_score:
                        best_score = score
                    if score >= self.config.match_threshold:
                        x1, y1, x2, y2 = face["box"]
                        detections.append(Detection(x1, y1, x2, y2, score, 0, "target_person"))
                self.inferences += len(selected)
        except Exception as exc:
            self.inference_errors += 1
            self.last_error = "{0}: {1}".format(type(exc).__name__, exc)
            if self.inference_errors == 1 or self.inference_errors % 20 == 0:
                self.get_logger().error("recognition failed: {0}".format(self.last_error))
            return
        if hasattr(self, 'family_publisher'):
            family_message = String()
            family_message.data = json.dumps(dict(frame_key=[frame.frame_id,frame.stamp.sec,frame.stamp.nanosec],
                faces=self.family_matcher.observe(family_faces,frame.stamp.sec+frame.stamp.nanosec/1e9)))
            self.family_publisher.publish(family_message)
        self.last_latency_ms = (time.monotonic() - started) * 1000.0
        self.last_score = best_score
        was_confirmed = self.confirmed
        self.confirmed = self.history.add(best_score) if self.target is not None else False
        self.matches_publisher.publish(build_detection_array(detections, frame.stamp, frame.frame_id, self.factories))
        self.matches_published += 1
        self.last_match_frame = match_frame
        # Deliver the transition with its completed match batch, without waiting
        # for the periodic health report or publishing status on every frame.
        if self.confirmed != was_confirmed:
            self.publish_status()

    def reload_target(self, force=False):
        try:
            mtime = os.path.getmtime(self.config.target_store_path)
        except OSError:
            mtime = None
        if not force and mtime == self._store_mtime and not hasattr(self.store, "family"):
            return
        try:
            target = self.store.load()
        except Exception as exc:
            self.last_error = "target load failed: {0}".format(exc)
            self.get_logger().error(self.last_error)
            return
        changed = (self.target or {}).get("created_utc") != (target or {}).get("created_utc")
        self.target = target
        self._store_mtime = mtime
        if changed or force:
            self.history.clear()
            self.confirmed = False
            self.last_match_frame = None
            self.get_logger().info("target enrollment {0}".format("loaded" if target else "not present"))
            if getattr(self, "status_publisher", None) is not None:
                self.publish_status()
                return True
        return False

    def on_status(self):
        now = time.monotonic()
        age = None if self.last_image_time is None else now - self.last_image_time
        if self.image_input.refresh_if_stale(now, age):
            self.get_logger().warning('Reconnected camera input after ten seconds without frames')
        observation_age = None if self.last_observation_time is None else now - self.last_observation_time
        if self.observation_input.refresh_if_stale(now, observation_age):
            self.matcher.clear()
            self.pending = None
            self.get_logger().warning('Reconnected face observations after ten seconds without messages')
        if not self.reload_target():
            self.publish_status()

    def publish_status(self):
        now = time.monotonic()
        stale = self.last_input_time is None or now - self.last_input_time > self.config.frame_timeout_sec
        if self.target is None:
            state = "not_enrolled"
        elif stale:
            state = "stale_input"
        elif self.confirmed:
            state = "target_confirmed"
        else:
            state = "searching"
        payload = {
            "image_subscription_reconnects": self.image_input.reconnects,
            "observation_subscription_reconnects": self.observation_input.reconnects,
            "last_observation_age_sec": None if self.last_observation_time is None else round(now - self.last_observation_time, 3),
            "state": state,
            "model": self.config.model_name,
            "provider": self.recognizer.provider,
            "device_name": self.recognizer.device_name,
            "target_label": None if self.target is None else self.target["label"],
            "target_revision": (self.target or {}).get("created_utc"),
            "match_frame": self.last_match_frame,
            "template_count": 0 if self.target is None else len(self.target["samples"]),
            "match_threshold": self.config.match_threshold,
            "confirmation": {
                "hits": self.history.hits,
                "required": self.config.confirmation_required,
                "window": self.config.confirmation_window,
                "confirmed": self.confirmed,
            },
            "last_score": self.last_score,
            "last_batch_latency_ms": self.last_latency_ms,
            "inferences": self.inferences,
            "inference_errors": self.inference_errors,
            "matches_published": self.matches_published,
            "pending_batches_dropped": self.pending_dropped,
            "last_error": self.last_error,
            "privacy": "embeddings only unless the operator explicitly retains aligned face crops",
            "uptime_sec": round(now - self.started, 3),
        }
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
        self.status_publisher.publish(message)

    def shutdown(self):
        if self.recognizer is not None:
            self.recognizer.close()
            self.recognizer = None


def main(args=None):
    configure_perception_transport()
    rclpy.init(args=args)
    node = None
    try:
        node = TargetRecognizerNode()
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
