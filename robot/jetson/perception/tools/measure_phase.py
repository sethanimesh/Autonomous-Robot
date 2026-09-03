#!/usr/bin/env python3
"""Measure detection performance for one held pose.

One pose per run, rather than a single long scripted sequence: it keeps the
operator in control, and each phase gets a clean number instead of one that
depends on hitting a timing cue.

    python3 measure_phase.py near --seconds 20 --expect 1
    python3 measure_phase.py two_people --expect 2
    python3 measure_phase.py empty --expect 0

With --expect 0 the run is scored as a false-positive test instead. Use
check_empty_scene.py for that rather than this, because it adds the settling
period that stops someone walking out of frame from contaminating the result.

A sample annotated frame is saved unless --no-sample is given. Always look at
it: a detection rate without a picture hides both a misaimed camera and a
scene that was not as empty as assumed.
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


class PhaseRecorder(Node):
    def __init__(self, expect):
        super().__init__("echora_measure_phase")
        self.expect = int(expect)
        self.messages = []
        self.camera_frames = 0
        self.sample = None
        self.current = 0
        self.create_subscription(
            Detection2DArray, "/perception/person_detections", self.on_detections, 10
        )
        self.create_subscription(Image, "/camera/image_raw", self.on_camera, BEST_EFFORT)
        self.create_subscription(
            Image, "/perception/person_image", self.on_annotated, BEST_EFFORT
        )

    def on_camera(self, message):
        self.camera_frames += 1

    def on_detections(self, message):
        self.current = len(message.detections)
        self.messages.append(
            [
                (
                    d.bbox.center.position.x,
                    d.bbox.center.position.y,
                    d.bbox.size_x,
                    d.bbox.size_y,
                    d.results[0].hypothesis.score if d.results else 0.0,
                )
                for d in message.detections
            ]
        )

    def on_annotated(self, message):
        if self.sample is not None or message.encoding != "bgr8":
            return
        if self.current != self.expect:
            return
        if len(message.data) != message.height * message.width * 3:
            return
        self.sample = numpy.frombuffer(message.data, numpy.uint8).reshape(
            message.height, message.width, 3
        ).copy()


def summarise(recorder, label, elapsed):
    total = len(recorder.messages)
    print("PHASE {0}  ({1:.0f}s, expecting {2})".format(label, elapsed, recorder.expect))
    print(
        "  camera {0} ({1:.1f} Hz) | detections {2} ({3:.1f} Hz)".format(
            recorder.camera_frames, recorder.camera_frames / elapsed, total, total / elapsed
        )
    )
    if not total:
        print("  no detection messages received; is the detector running?")
        return

    if recorder.expect == 0:
        wrong = [m for m in recorder.messages if m]
        print(
            "  FALSE POSITIVES: {0} / {1} = {2:.2f}%".format(
                len(wrong), total, 100.0 * len(wrong) / total
            )
        )
        if wrong:
            scores = sorted(box[4] for m in wrong for box in m)
            print("  scores: min={0:.2f} max={1:.2f}".format(scores[0], scores[-1]))
        return

    hits = [m for m in recorder.messages if len(m) >= recorder.expect]
    print(
        "  frames with >={0} person: {1} / {2} = {3:.1f}%".format(
            recorder.expect, len(hits), total, 100.0 * len(hits) / total
        )
    )
    counts = {}
    for message in recorder.messages:
        counts[len(message)] = counts.get(len(message), 0) + 1
    print("  person-count histogram: {0}".format(dict(sorted(counts.items()))))

    boxes = [box for m in recorder.messages for box in m]
    if boxes:
        scores = sorted(box[4] for box in boxes)
        heights = sorted(box[3] for box in boxes)
        widths = sorted(box[2] for box in boxes)
        print(
            "  score  min={0:.2f} med={1:.2f} max={2:.2f}".format(
                scores[0], scores[len(scores) // 2], scores[-1]
            )
        )
        print(
            "  box    h med={0:.0f} px  w med={1:.0f} px".format(
                heights[len(heights) // 2], widths[len(widths) // 2]
            )
        )


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("label", help="name for this phase, e.g. near or two_people")
    parser.add_argument("--seconds", type=float, default=20.0)
    parser.add_argument("--expect", type=int, default=1, help="people expected in view")
    parser.add_argument("--no-sample", action="store_true", help="do not save a frame")
    parser.add_argument("--sample-dir", default="/tmp")
    args = parser.parse_args(argv)

    rclpy.init()
    recorder = PhaseRecorder(args.expect)
    started = time.time()
    try:
        while time.time() - started < args.seconds:
            rclpy.spin_once(recorder, timeout_sec=0.05)
        elapsed = time.time() - started
        summarise(recorder, args.label, elapsed)
        if recorder.sample is not None and not args.no_sample:
            path = "{0}/phase_{1}.jpg".format(args.sample_dir.rstrip("/"), args.label)
            cv2.imwrite(path, recorder.sample, [cv2.IMWRITE_JPEG_QUALITY, 85])
            print("  saved {0}".format(path))
    except KeyboardInterrupt:
        pass
    finally:
        recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
