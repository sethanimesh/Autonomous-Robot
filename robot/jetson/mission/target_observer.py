#!/usr/bin/env python3
"""Publish mission-ready target observations from recognition ROS topics."""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import CameraInfo
from rclpy.qos import qos_profile_sensor_data

from family_observer import FamilyObserver

from target_gate import TargetGateError, build_target_observation, target_box_position
from image_subscription import configure_perception_transport


class TargetObserverNode(Node):
    def __init__(self):
        super().__init__("echora_target_observer")
        self.recognition_status = None
        self.status_received_at = None
        self.match_received_at = None
        self.match_source_time = None
        self.match_frame_key = None
        self.box_heights = []
        self.boxes = []
        self.image_size = None
        self.image_info_at = None
        self.publisher = self.create_publisher(
            String, "/mission/target_observation", 10
        )
        self.create_subscription(
            String, "/perception/recognition_status", self.on_status, 10
        )
        self.create_subscription(
            Detection2DArray,
            "/perception/target_matches",
            self.on_matches,
            10,
        )
        self.create_subscription(
            CameraInfo, "/camera/camera_info", self.on_info, qos_profile_sensor_data
        )
        self.create_timer(0.2, self.publish_observation)

    def on_status(self, message):
        try:
            value = json.loads(message.data)
        except ValueError:
            return
        if isinstance(value, dict):
            self.recognition_status = value
            self.status_received_at = time.monotonic()
            self.publish_observation()

    def on_info(self, message):
        self.image_size = (message.width, message.height)
        self.image_info_at = time.monotonic()

    def on_matches(self, message):
        self.boxes = [
            (d.bbox.center.position.x, d.bbox.center.position.y,
             d.bbox.size_x, d.bbox.size_y)
            for d in message.detections
        ]
        self.box_heights = [
            float(detection.bbox.size_y)
            for detection in message.detections
            if detection.bbox.size_y > 0
        ]
        self.match_source_time = (
            float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) / 1000000000.0
        )
        self.match_received_at = time.monotonic()
        self.match_frame_key = (
            message.header.frame_id,
            int(message.header.stamp.sec),
            int(message.header.stamp.nanosec),
        )
        self.publish_observation()

    def matches_follow_confirmation(self):
        status = self.recognition_status or {}
        if "match_frame" not in status:
            return True  # Compatible with an older recognizer during rollout.
        frame = status["match_frame"]
        if not isinstance(frame, dict) or self.match_frame_key is None:
            return False
        sec, nanosec = frame.get("sec"), frame.get("nanosec")
        if (type(sec) is not int or type(nanosec) is not int
                or not 0 <= nanosec < 1000000000):
            return False
        return (
            self.match_frame_key[0] == frame.get("frame_id")
            and self.match_frame_key[1:] >= (sec, nanosec)
        )

    def publish_observation(self):
        now = time.monotonic()
        status_age = (
            999.0
            if self.status_received_at is None
            else now - self.status_received_at
        )
        if self.match_source_time is None:
            match_age = 999.0
        else:
            match_age = max(
                0.0,
                self.get_clock().now().nanoseconds / 1000000000.0
                - self.match_source_time,
            )
        try:
            observation = build_target_observation(
                self.recognition_status or {},
                status_age,
                match_age,
                self.box_heights if self.matches_follow_confirmation() else [],
                image_height_pixels=self.image_size[1] if self.image_size else 480,
            )
            payload = {
                "ok": True,
                "confirmed": observation.confirmed,
                "age_seconds": round(observation.age_seconds, 3),
                "box_height_fraction": round(
                    observation.box_height_fraction, 4
                ),
                "recognition_state": (self.recognition_status or {}).get(
                    "state", "unavailable"
                ),
                "target_label": (self.recognition_status or {}).get("target_label"),
                "target_revision": (self.recognition_status or {}).get("target_revision"),
            }
            if (observation.confirmed and self.image_size
                    and self.image_info_at is not None
                    and now - self.image_info_at <= 2.5):
                position = target_box_position(self.boxes, *self.image_size)
                if position:
                    payload.update(position)
        except TargetGateError as exc:
            payload = {"ok": False, "error": str(exc)}
        if hasattr(self, "family"):
            try: payload = self.family.enrich(payload)
            except Exception as exc: payload["wardrobe_error"] = type(exc).__name__
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
        self.publisher.publish(message)


def main(args=None):
    configure_perception_transport()
    rclpy.init(args=args)
    node = TargetObserverNode()
    node.family = FamilyObserver(node)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.family.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
