#!/usr/bin/env python3
"""Exact timestamp matching and person-region selection for face detection."""

from collections import OrderedDict


def stamp_key(stamp):
    return (int(stamp.sec), int(stamp.nanosec))


class TimestampedImageCache(object):
    """Small bounded cache used to match YOLOX output to its source frame."""

    def __init__(self, capacity):
        if int(capacity) < 1:
            raise ValueError("capacity must be positive")
        self.capacity = int(capacity)
        self._images = OrderedDict()
        self.evicted = 0
        self.misses = 0

    def offer(self, key, image):
        if key in self._images:
            del self._images[key]
        self._images[key] = image
        while len(self._images) > self.capacity:
            self._images.popitem(last=False)
            self.evicted += 1

    def take(self, key):
        image = self._images.pop(key, None)
        if image is None:
            self.misses += 1
        return image

    def clear(self):
        self._images.clear()

    def __len__(self):
        return len(self._images)


class ExactPairMatcher(object):
    """Match two timestamped streams regardless of callback arrival order."""

    def __init__(self, capacity):
        if int(capacity) < 1:
            raise ValueError("capacity must be positive")
        self.capacity = int(capacity)
        self._left = OrderedDict()
        self._right = OrderedDict()
        self.left_evicted = 0
        self.right_evicted = 0
        self.matched = 0

    def _offer(self, own, other, key, value, side):
        partner = other.pop(key, None)
        if partner is not None:
            self.matched += 1
            return (value, partner) if side == "left" else (partner, value)
        if key in own:
            del own[key]
        own[key] = value
        while len(own) > self.capacity:
            own.popitem(last=False)
            if side == "left":
                self.left_evicted += 1
            else:
                self.right_evicted += 1
        return None

    def offer_left(self, key, value):
        return self._offer(self._left, self._right, key, value, "left")

    def offer_right(self, key, value):
        return self._offer(self._right, self._left, key, value, "right")

    @property
    def left_depth(self):
        return len(self._left)

    @property
    def right_depth(self):
        return len(self._right)

    @property
    def evicted(self):
        return self.left_evicted + self.right_evicted

    def clear(self):
        self._left.clear()
        self._right.clear()


class PersonRegion(object):
    __slots__ = ("x1", "y1", "x2", "y2", "person_score")

    def __init__(self, x1, y1, x2, y2, person_score=0.0):
        self.x1 = int(x1)
        self.y1 = int(y1)
        self.x2 = int(x2)
        self.y2 = int(y2)
        self.person_score = float(person_score)

    @property
    def width(self):
        return self.x2 - self.x1

    @property
    def height(self):
        return self.y2 - self.y1

    @property
    def area(self):
        return self.width * self.height


def _person_score(detection):
    if not detection.results:
        return 0.0
    hypothesis = detection.results[0].hypothesis
    if hypothesis.class_id != "person":
        return 0.0
    return float(hypothesis.score)


def select_person_regions(
    detections,
    image_width,
    image_height,
    max_regions,
    padding_fraction,
    height_fraction,
    minimum_pixels,
):
    """Return largest valid upper-person crops in source-image coordinates."""
    candidates = []
    for detection in detections:
        score = _person_score(detection)
        if score <= 0.0:
            continue
        box = detection.bbox
        width = float(box.size_x)
        height = float(box.size_y)
        if width <= 0.0 or height <= 0.0:
            continue
        x1 = float(box.center.position.x) - width / 2.0
        y1 = float(box.center.position.y) - height / 2.0
        x2 = x1 + width
        y2 = y1 + height * float(height_fraction)
        pad_x = width * float(padding_fraction)
        pad_y = height * float(padding_fraction)
        region = PersonRegion(
            max(0, round(x1 - pad_x)),
            max(0, round(y1 - pad_y)),
            min(int(image_width), round(x2 + pad_x)),
            min(int(image_height), round(y2 + pad_y)),
            score,
        )
        if region.width < minimum_pixels or region.height < minimum_pixels:
            continue
        candidates.append(region)
    candidates.sort(key=lambda item: item.area, reverse=True)
    return candidates[: int(max_regions)]
