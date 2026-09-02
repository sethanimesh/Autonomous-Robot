#!/usr/bin/env python3
"""Assembly of the detector's output messages.

The ROS message classes are injected rather than imported, so the assembly
rules -- which stamp and frame ID go where, how a box becomes a centre and a
size, what an empty result looks like -- are all testable with plain fakes on
a machine with no ROS installed.
"""


class MessageFactories(object):
    """The four message constructors the detector needs."""

    __slots__ = ("array", "detection", "bounding_box", "hypothesis")

    def __init__(self, array, detection, bounding_box, hypothesis):
        self.array = array
        self.detection = detection
        self.bounding_box = bounding_box
        self.hypothesis = hypothesis


def build_detection_array(detections, stamp, frame_id, factories):
    """Build a Detection2DArray from accepted person boxes.

    Every detection carries the *source image's* stamp and frame ID, not the
    time inference finished, so a consumer can always relate a detection back
    to the exact frame it came from. With no detections the array is still
    published, empty, which is how a subscriber distinguishes "nobody here"
    from "the detector has stopped".

    The hypothesis pose is deliberately left at its zero default: this camera
    is uncalibrated, so no metric position exists to report.
    """
    message = factories.array()
    message.header.stamp = stamp
    message.header.frame_id = frame_id

    for detection in detections:
        entry = factories.detection()
        entry.header.stamp = stamp
        entry.header.frame_id = frame_id
        entry.id = detection.label

        box = factories.bounding_box()
        box.center.position.x = detection.center_x
        box.center.position.y = detection.center_y
        box.center.theta = 0.0
        box.size_x = detection.width
        box.size_y = detection.height
        entry.bbox = box

        hypothesis = factories.hypothesis()
        hypothesis.hypothesis.class_id = detection.label
        hypothesis.hypothesis.score = detection.score
        entry.results.append(hypothesis)

        message.detections.append(entry)

    return message


class BoxAnnotation(object):
    """Integer draw instructions for one detection."""

    __slots__ = (
        "x1",
        "y1",
        "x2",
        "y2",
        "caption",
        "label_x",
        "label_y",
        "label_width",
        "label_height",
    )

    def __init__(self, x1, y1, x2, y2, caption, label_x, label_y, label_width, label_height):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.caption = caption
        self.label_x = label_x
        self.label_y = label_y
        self.label_width = label_width
        self.label_height = label_height

    def __repr__(self):  # pragma: no cover - debugging aid
        return "BoxAnnotation({0},{1},{2},{3},{4!r})".format(
            self.x1, self.y1, self.x2, self.y2, self.caption
        )


def annotation_plan(detections, image_width, image_height, measure_text):
    """Turn detections into integer draw instructions inside the image.

    ``measure_text`` is the text-size function (OpenCV's ``getTextSize`` in
    production) so the placement arithmetic can be tested with a predictable
    stand-in. Every coordinate returned is already clamped to the image, so
    the drawing code never has to think about edges.
    """
    plan = []
    width = int(image_width)
    height = int(image_height)
    for detection in detections:
        x1 = min(max(int(round(detection.x1)), 0), width)
        y1 = min(max(int(round(detection.y1)), 0), height)
        x2 = min(max(int(round(detection.x2)), 0), width)
        y2 = min(max(int(round(detection.y2)), 0), height)
        caption = "{0} {1:.2f}".format(detection.label, detection.score)
        text_width, text_height = measure_text(caption)

        # Prefer a label above the box; drop it inside when the box touches
        # the top edge, so the caption is never clipped off-screen.
        label_y = y1 - text_height
        if label_y < 0:
            label_y = y1
        label_y = min(label_y, max(height - text_height, 0))
        label_x = min(x1, max(width - text_width, 0))
        label_x = max(label_x, 0)

        plan.append(
            BoxAnnotation(
                x1,
                y1,
                x2,
                y2,
                caption,
                label_x,
                label_y,
                min(text_width, width - label_x),
                min(text_height, height - label_y),
            )
        )
    return plan
