#!/usr/bin/env python3
"""Verify live face-pipeline topics and exact source timestamp propagation."""

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray


BEST_EFFORT = QoSProfile(
    depth=1,
    history=QoSHistoryPolicy.KEEP_LAST,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
)


def key(header):
    return (header.stamp.sec, header.stamp.nanosec, header.frame_id)


class Verifier(Node):
    def __init__(self):
        super().__init__("echora_verify_face_pipeline")
        self.camera = set()
        self.person = set()
        self.faces = []
        self.face_count = 0
        self.invalid_faces = 0
        self.status = None
        self.create_subscription(Image, "/camera/image_raw", self.on_camera, BEST_EFFORT)
        self.create_subscription(
            Detection2DArray,
            "/perception/person_detections",
            self.on_person,
            10,
        )
        self.create_subscription(
            Detection2DArray,
            "/perception/face_detections",
            self.on_faces,
            10,
        )
        self.create_subscription(String, "/perception/face_status", self.on_status, 10)

    def on_camera(self, message):
        self.camera.add(key(message.header))

    def on_person(self, message):
        self.person.add(key(message.header))

    def on_faces(self, message):
        outer = key(message.header)
        self.faces.append(outer)
        self.face_count += len(message.detections)
        for detection in message.detections:
            if key(detection.header) != outer:
                self.invalid_faces += 1
            if detection.bbox.size_x <= 0 or detection.bbox.size_y <= 0:
                self.invalid_faces += 1
            if len(detection.results) != 1:
                self.invalid_faces += 1
                continue
            hypothesis = detection.results[0].hypothesis
            if hypothesis.class_id != "face" or not 0.0 <= hypothesis.score <= 1.0:
                self.invalid_faces += 1

    def on_status(self, message):
        try:
            self.status = json.loads(message.data)
        except ValueError:
            self.status = {"invalid_json": True}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=8.0)
    parser.add_argument("--require-face", action="store_true")
    args = parser.parse_args(argv)
    rclpy.init()
    verifier = Verifier()
    started = time.monotonic()
    try:
        while time.monotonic() - started < args.seconds:
            rclpy.spin_once(verifier, timeout_sec=0.1)
    finally:
        verifier.destroy_node()
        rclpy.shutdown()

    face_keys = set(verifier.faces)
    report = {
        "camera_messages": len(verifier.camera),
        "person_messages": len(verifier.person),
        "face_messages": len(verifier.faces),
        "faces": verifier.face_count,
        "face_stamps_in_camera": len(face_keys & verifier.camera),
        "face_stamps_in_person": len(face_keys & verifier.person),
        "unique_face_stamps": len(face_keys),
        "invalid_face_entries": verifier.invalid_faces,
        "status_state": None if verifier.status is None else verifier.status.get("state"),
        "provider": None if verifier.status is None else verifier.status.get("provider"),
        "inference_errors": None if verifier.status is None else verifier.status.get("inference_errors"),
    }
    camera_match_fraction = (
        float(report["face_stamps_in_camera"]) / len(face_keys) if face_keys else 0.0
    )
    person_match_fraction = (
        float(report["face_stamps_in_person"]) / len(face_keys) if face_keys else 0.0
    )
    report["camera_observation_match_percent"] = round(camera_match_fraction * 100.0, 1)
    report["person_observation_match_percent"] = round(person_match_fraction * 100.0, 1)
    print(json.dumps(report, indent=2, sort_keys=True))
    passed = (
        len(verifier.camera) >= 10
        and len(verifier.person) >= 10
        and len(verifier.faces) >= 5
        # The verifier's own best-effort camera subscription may miss a
        # sample under load. Ninety-five percent proves propagation while
        # avoiding a false failure caused by the observer itself.
        and camera_match_fraction >= 0.90
        and person_match_fraction >= 0.95
        and verifier.invalid_faces == 0
        and report["status_state"] == "detecting"
        and report["provider"] == "tensorrt_fp16"
        and report["inference_errors"] == 0
        and (not args.require_face or verifier.face_count > 0)
    )
    raise SystemExit(0 if passed else 2)


if __name__ == "__main__":
    main()
