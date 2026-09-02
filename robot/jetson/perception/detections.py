#!/usr/bin/env python3
"""Detection geometry, class filtering, and non-maximum suppression.

The inference backend does the bulk numeric work on the GPU and hands this
module a short list of plain candidate tuples. Everything from that point on
is ordinary Python arithmetic, so class filtering, threshold behaviour, box
conversion, clipping, and NMS are all covered by the repository test suite on
a machine with no numpy, no CUDA, and no camera.
"""

YOLOX_STRIDES = (8, 16, 32)


class Detection(object):
    """One accepted person box in source-image pixel coordinates."""

    __slots__ = ("x1", "y1", "x2", "y2", "score", "class_id", "label")

    def __init__(self, x1, y1, x2, y2, score, class_id, label):
        self.x1 = float(x1)
        self.y1 = float(y1)
        self.x2 = float(x2)
        self.y2 = float(y2)
        self.score = float(score)
        self.class_id = int(class_id)
        self.label = str(label)

    @property
    def width(self):
        return self.x2 - self.x1

    @property
    def height(self):
        return self.y2 - self.y1

    @property
    def center_x(self):
        return (self.x1 + self.x2) / 2.0

    @property
    def center_y(self):
        return (self.y1 + self.y2) / 2.0

    @property
    def area(self):
        return self.width * self.height

    def as_tuple(self):
        return (self.x1, self.y1, self.x2, self.y2, self.score, self.class_id)

    def __repr__(self):  # pragma: no cover - debugging aid
        return "Detection({0:.1f},{1:.1f},{2:.1f},{3:.1f},score={4:.3f},class={5})".format(
            self.x1, self.y1, self.x2, self.y2, self.score, self.class_id
        )


def build_grid_strides(input_height, input_width, strides=YOLOX_STRIDES):
    """Return the YOLOX anchor grid as (grid_x, grid_y, stride) triples.

    The order matches the row-major layout of the exported ONNX output, so
    row i of the model output corresponds to entry i of this list.
    """
    if input_height <= 0 or input_width <= 0:
        raise ValueError("model input size must be positive")
    grid = []
    for stride in strides:
        if input_height % stride or input_width % stride:
            raise ValueError(
                "model input {0}x{1} is not divisible by stride {2}".format(
                    input_width, input_height, stride
                )
            )
        rows = input_height // stride
        columns = input_width // stride
        for row in range(rows):
            for column in range(columns):
                grid.append((column, row, stride))
    return grid


def letterbox_ratio(source_width, source_height, input_width, input_height):
    """Scale factor for a padded resize that preserves aspect ratio."""
    if source_width <= 0 or source_height <= 0:
        raise ValueError("source image size must be positive")
    return min(
        float(input_width) / float(source_width),
        float(input_height) / float(source_height),
    )


def center_to_corners(center_x, center_y, width, height):
    """Convert a YOLOX cx/cy/w/h box to x1/y1/x2/y2."""
    half_width = width / 2.0
    half_height = height / 2.0
    return (
        center_x - half_width,
        center_y - half_height,
        center_x + half_width,
        center_y + half_height,
    )


def clip_box(x1, y1, x2, y2, image_width, image_height):
    """Clamp a box to the image, keeping x1<=x2 and y1<=y2."""
    x1 = min(max(x1, 0.0), float(image_width))
    y1 = min(max(y1, 0.0), float(image_height))
    x2 = min(max(x2, 0.0), float(image_width))
    y2 = min(max(y2, 0.0), float(image_height))
    if x2 < x1:
        x1, x2 = x2, x1
    if y2 < y1:
        y1, y2 = y2, y1
    return (x1, y1, x2, y2)


def intersection_over_union(first, second):
    """IoU of two (x1, y1, x2, y2) boxes. Zero when either has no area."""
    left = max(first[0], second[0])
    top = max(first[1], second[1])
    right = min(first[2], second[2])
    bottom = min(first[3], second[3])
    if right <= left or bottom <= top:
        return 0.0
    overlap = (right - left) * (bottom - top)
    first_area = (first[2] - first[0]) * (first[3] - first[1])
    second_area = (second[2] - second[0]) * (second[3] - second[1])
    union = first_area + second_area - overlap
    if union <= 0.0:
        return 0.0
    return overlap / union


def non_maximum_suppression(detections, iou_threshold):
    """Greedy NMS over already-filtered detections, highest score first."""
    ordered = sorted(detections, key=lambda item: item.score, reverse=True)
    kept = []
    for candidate in ordered:
        box = (candidate.x1, candidate.y1, candidate.x2, candidate.y2)
        suppressed = False
        for keeper in kept:
            existing = (keeper.x1, keeper.y1, keeper.x2, keeper.y2)
            if intersection_over_union(box, existing) > iou_threshold:
                suppressed = True
                break
        if not suppressed:
            kept.append(candidate)
    return kept


def select_person_detections(
    candidates,
    person_class_id,
    confidence_threshold,
    iou_threshold,
    image_width,
    image_height,
    ratio,
    max_detections=50,
    label="person",
):
    """Turn raw model candidates into accepted person boxes.

    ``candidates`` is a sequence of ``(cx, cy, w, h, score, class_id)`` in
    model-input pixel space. ``ratio`` is the letterbox scale that was applied
    to the source image, so dividing by it maps boxes back to source pixels.

    Non-person classes are removed, low scores are removed, boxes are
    converted, rescaled, clipped to the image, degenerate boxes are dropped,
    overlapping boxes are suppressed, and at most ``max_detections`` survive.
    """
    if ratio <= 0.0:
        raise ValueError("letterbox ratio must be positive")
    if image_width <= 0 or image_height <= 0:
        raise ValueError("image size must be positive")

    accepted = []
    for candidate in candidates:
        center_x, center_y, width, height, score, class_id = candidate
        if int(class_id) != int(person_class_id):
            continue
        if float(score) < confidence_threshold:
            continue
        x1, y1, x2, y2 = center_to_corners(
            float(center_x), float(center_y), float(width), float(height)
        )
        x1, y1, x2, y2 = (x1 / ratio, y1 / ratio, x2 / ratio, y2 / ratio)
        x1, y1, x2, y2 = clip_box(x1, y1, x2, y2, image_width, image_height)
        detection = Detection(x1, y1, x2, y2, score, person_class_id, label)
        if detection.width <= 0.0 or detection.height <= 0.0:
            continue
        accepted.append(detection)

    kept = non_maximum_suppression(accepted, iou_threshold)
    return kept[:max_detections]
