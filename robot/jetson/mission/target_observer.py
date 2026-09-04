#!/usr/bin/env python3
"""Publish mission-ready target observations from recognition ROS topics."""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

from target_gate import TargetGateError, build_target_observation


class TargetObserverNode(Node):
    def __init__(self):
        super().__init__("echora_target_observer")
        self.recognition_status = None
        self.status_received_at = None
        self.match_received_at = None
        self.match_source_time = None
        self.box_heights = []
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
        self.create_timer(0.2, self.publish_observation)

    def on_status(self, message):
        try:
            value = json.loads(message.data)
        except ValueError:
            return
        if isinstance(value, dict):
            self.recognition_status = value
            self.status_received_at = time.monotonic()

    def on_matches(self, message):
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
                self.box_heights,
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
            }
        except TargetGateError as exc:
            payload = {"ok": False, "error": str(exc)}
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = TargetObserverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
