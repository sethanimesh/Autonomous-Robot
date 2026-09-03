#!/usr/bin/env python3
"""Face geometry, landmark mapping, and global duplicate suppression."""

try:
    from detections import Detection, clip_box, non_maximum_suppression
except ImportError:  # pragma: no cover
    from .detections import Detection, clip_box, non_maximum_suppression


class FaceDetection(Detection):
    __slots__ = ("landmarks",)

    def __init__(self, x1, y1, x2, y2, score, landmarks=()):
        Detection.__init__(self, x1, y1, x2, y2, score, 0, "face")
        self.landmarks = tuple((float(x), float(y)) for x, y in landmarks)


def map_face_to_source(candidate, region, model_width, model_height, image_width, image_height):
    """Map one YuNet crop-space result back into the full camera frame."""
    x1, y1, x2, y2, score, landmarks = candidate
    scale_x = float(region.width) / float(model_width)
    scale_y = float(region.height) / float(model_height)
    mapped = (
        region.x1 + float(x1) * scale_x,
        region.y1 + float(y1) * scale_y,
        region.x1 + float(x2) * scale_x,
        region.y1 + float(y2) * scale_y,
    )
    mapped = clip_box(*mapped, image_width, image_height)
    points = []
    for x, y in landmarks:
        points.append(
            (
                min(max(region.x1 + float(x) * scale_x, 0.0), float(image_width)),
                min(max(region.y1 + float(y) * scale_y, 0.0), float(image_height)),
            )
        )
    return FaceDetection(*mapped, score, points)


def select_faces(faces, iou_threshold, max_faces):
    valid = [face for face in faces if face.width > 0.0 and face.height > 0.0]
    return non_maximum_suppression(valid, iou_threshold)[: int(max_faces)]
