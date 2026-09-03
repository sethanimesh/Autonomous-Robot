#!/usr/bin/env python3
"""Measure the false-positive rate on an empty scene.

    python3 check_empty_scene.py --settle 15 --seconds 90

The settling period matters. A first run of this test reported 5.14% false
positives; a captured frame showed a person's torso filling the image, so the
room simply was not empty yet and those detections were correct. Discarding
the first few seconds, and saving a frame for every apparent false positive so
it can be checked by eye, is what turns this from a plausible number into a
trustworthy one. Re-run properly, the same scene measured zero.

Any frame saved here is a detection the detector believes is a person. Look at
each one before reporting a false-positive rate.
"""

import argparse
import time

import cv2
import numpy
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

BEST_EFFORT = QoSProfile(
    depth=1,
    history=QoSHistoryPolicy.KEEP_LAST,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
)


class EmptySceneRecorder(Node):
    def __init__(self, settle, max_samples, sample_dir):
        super().__init__("echora_check_empty_scene")
        self.settle = float(settle)
        self.max_samples = int(max_samples)
        self.sample_dir = sample_dir.rstrip("/")
        self.started = time.time()
        self.total = 0
        self.positives = []
        self.saved = 0
        self.current = 0
        self.create_subscription(
            Detection2DArray, "/perception/person_detections", self.on_detections, 10
        )
        self.create_subscription(
            Image, "/perception/person_image", self.on_annotated, BEST_EFFORT
        )

    def measuring(self):
        return time.time() - self.started >= self.settle

    def on_detections(self, message):
        self.current = len(message.detections)
        if not self.measuring():
            return
        self.total += 1
        if message.detections:
            self.positives.append(
                [
                    (
                        d.bbox.size_x,
                        d.bbox.size_y,
                        d.results[0].hypothesis.score if d.results else 0.0,
                    )
                    for d in message.detections
                ]
            )

    def on_annotated(self, message):
        if not self.measuring() or self.current == 0:
            return
        if self.saved >= self.max_samples or message.encoding != "bgr8":
            return
        if len(message.data) != message.height * message.width * 3:
            return
        frame = numpy.frombuffer(message.data, numpy.uint8).reshape(
            message.height, message.width, 3
        )
        path = "{0}/false_positive_{1}.jpg".format(self.sample_dir, self.saved)
        cv2.imwrite(path, frame, [cv2.IMWRITE_JPEG_QUALITY, 88])
        print("  saved {0} -- CHECK IT: the scene may not have been empty".format(path))
        self.saved += 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--settle", type=float, default=15.0, help="discarded lead-in")
    parser.add_argument("--seconds", type=float, default=90.0, help="measured window")
    parser.add_argument("--samples", type=int, default=3, help="frames to save")
    parser.add_argument("--sample-dir", default="/tmp")
    args = parser.parse_args(argv)

    rclpy.init()
    recorder = EmptySceneRecorder(args.settle, args.samples, args.sample_dir)
    print("settling {0:.0f}s -- clear the frame now ...".format(args.settle), flush=True)
    try:
        while time.time() - recorder.started < args.settle + args.seconds:
            rclpy.spin_once(recorder, timeout_sec=0.05)

        print(
            "EMPTY SCENE: {0:.0f}s measured after a {1:.0f}s settle".format(
                args.seconds, args.settle
            )
        )
        print("  detection messages: {0}".format(recorder.total))
        if not recorder.total:
            print("  none received; is the detector running?")
        elif recorder.positives:
            count = len(recorder.positives)
            scores = sorted(b[2] for p in recorder.positives for b in p)
            print(
                "  FALSE POSITIVES: {0} / {1} = {2:.2f}%".format(
                    count, recorder.total, 100.0 * count / recorder.total
                )
            )
            print(
                "  scores: min={0:.2f} med={1:.2f} max={2:.2f}".format(
                    scores[0], scores[len(scores) // 2], scores[-1]
                )
            )
            print("  inspect the saved frames before reporting this as a rate")
        else:
            print("  ZERO false positives")
    except KeyboardInterrupt:
        pass
    finally:
        recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
