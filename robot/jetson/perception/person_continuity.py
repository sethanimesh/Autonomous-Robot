"""Short-lived body continuity anchored by repeated face matches.

Experimental, stationary observation only. The result is not a fresh face
confirmation and must not authorize an approach or estimate standoff distance.
"""

import math


def valid_box(box):
    try:
        if len(box) != 4 or any(isinstance(x, bool) for x in box):
            return False
        x1, y1, x2, y2 = box
        return all(math.isfinite(x) for x in box) and 0 <= x1 < x2 <= 1 and 0 <= y1 < y2 <= 1
    except (TypeError, ValueError):
        return False


def intersection(a, b):
    return max(0., min(a[2], b[2]) - max(a[0], b[0])) * max(0., min(a[3], b[3]) - max(a[1], b[1]))


def area(box):
    return (box[2] - box[0]) * (box[3] - box[1])


def appearance_similarity(a, b):
    if not a or len(a) != len(b):
        return 0.
    return sum(math.sqrt(x*y) for x, y in zip(a, b))


def clean_person(person):
    try:
        box = tuple(person['box'])
        histogram = tuple(float(x) for x in person['appearance'])
        if not valid_box(box) or not histogram or not all(math.isfinite(x) and x >= 0 for x in histogram):
            return None
        total = sum(histogram)
        if total <= 0 or not math.isfinite(total):
            return None
        return {'box': box, 'appearance': tuple(x/total for x in histogram)}
    except (KeyError, TypeError, ValueError):
        return None


class PersonContinuity:
    """Repeated face/body matches seed a track; body evidence never renews identity."""

    def __init__(self, maximum_face_age_seconds=10., required_face_hits=3):
        if type(required_face_hits) is not int or not 2 <= required_face_hits <= 5:
            raise ValueError("face anchors require two to five hits")
        self.required_face_hits = required_face_hits
        if (isinstance(maximum_face_age_seconds, bool)
                or not isinstance(maximum_face_age_seconds, (int, float))
                or not math.isfinite(maximum_face_age_seconds)
                or not 0 < maximum_face_age_seconds <= 30):
            raise ValueError('face evidence lifetime must be positive and at most 30 seconds')
        self.maximum_face_age_seconds = float(maximum_face_age_seconds)
        self.reset()

    def reset(self):
        self.person = None
        self.face_hits = 0
        self.face_at = None
        self.seen_at = None
        self.last_stamp = None

    def _lose(self, reason):
        self.person = None
        self.face_hits = 0
        self.face_at = self.seen_at = None
        return {'state': 'lost', 'reason': reason, 'face_hits': 0}

    def _consistent(self, person):
        old, new = self.person['box'], person['box']
        overlap = intersection(old, new)
        iou = overlap / (area(old) + area(new) - overlap)
        return iou >= .2 and appearance_similarity(self.person['appearance'], person['appearance']) >= .8

    def observe(self, people, matched_faces, stamp, now):
        if (isinstance(stamp, bool) or isinstance(now, bool)
                or not isinstance(stamp, (int, float)) or not isinstance(now, (int, float))
                or not math.isfinite(stamp) or not math.isfinite(now)):
            return self._lose('invalid_time')
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return {'state': 'ignored', 'reason': 'unordered_frame', 'face_hits': self.face_hits}
        self.last_stamp = stamp
        if not -.05 <= now - stamp <= .75:
            return self._lose('stale_frame')
        if self.person is not None:
            if now - self.seen_at > .75:
                return self._lose('observation_gap')
            if self.face_hits >= self.required_face_hits and now - self.face_at > self.maximum_face_age_seconds:
                return self._lose('face_anchor_expired')
            if self.face_hits < self.required_face_hits and now - self.face_at > .75:
                return self._lose('pending_face_expired')
        persons = [p for p in (clean_person(p) for p in people) if p is not None]
        if any(not valid_box(f) for f in matched_faces):
            return self._lose('invalid_face_box')
        if len(matched_faces) > 1:
            return self._lose('ambiguous_face_matches')
        if self.person is not None and not persons:
            # A missed detection is missing evidence. Preserve the internal
            # anchor briefly, but never expose its old position as current.
            # seen_at and face_at deliberately do not advance here.
            return {'state': 'temporarily_missing', 'reason': 'no_usable_body',
                    'face_hits': self.face_hits,
                    'face_age_seconds': max(0., now-self.face_at)}
        if matched_faces:
            face = matched_faces[0]
            associated = [p for p in persons if intersection(face, p['box']) / area(face) >= .8]
            if len(associated) != 1:
                return self._lose('ambiguous_face_body')
            selected = associated[0]
            if self.person is not None and not self._consistent(selected):
                return self._lose('face_changed_person')
            self.face_hits = min(self.required_face_hits, self.face_hits + 1)
            self.person = selected
            self.face_at = self.seen_at = stamp
            return {'state': 'face_confirmed' if self.face_hits >= self.required_face_hits else 'confirming_face',
                    'face_hits': self.face_hits, 'face_age_seconds': max(0., now-stamp),
                    'body_box': list(selected['box'])}
        if self.person is None:
            self._lose('no_face_anchor')
            return {'state': 'waiting_face', 'reason': 'no_face_anchor', 'face_hits': 0}
        candidates = [p for p in persons if self._consistent(p)]
        if len(candidates) != 1:
            return self._lose('ambiguous_body' if candidates else 'body_lost')
        # Move only the spatial box. Never train the reference on unverified
        # body matches: repeated small mistakes would drift to someone else.
        self.person = dict(self.person, box=candidates[0]['box'])
        self.seen_at = stamp
        return {'state': 'body_continuity' if self.face_hits >= self.required_face_hits else 'confirming_face',
                'face_hits': self.face_hits,
                'face_age_seconds': max(0., now-self.face_at),
                'body_box': list(self.person['box'])}


def person_appearance(image, box):
    """Small HSV histogram from the inner torso; pixels never leave RAM.

    This is a cheap experimental continuity cue, not a re-identification model.
    Cropped bodies are allowed; a complete standing body is not required.
    """
    import cv2
    import numpy as np
    if not valid_box(box):
        return None
    height, width = image.shape[:2]
    x1, y1, x2, y2 = box
    dx, dy = x2-x1, y2-y1
    crop = image[int((y1+.2*dy)*height):int((y1+.7*dy)*height),
                 int((x1+.2*dx)*width):int((x2-.2*dx)*width)]
    if crop.size == 0 or min(crop.shape[:2]) < 8:
        return None
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    if float(hsv[:, :, 2].mean()) < 12:
        return None
    histogram = cv2.calcHist([hsv], [0, 1], None, [12, 4], [0, 180, 0, 256]).reshape(-1)
    return (histogram / max(float(np.sum(histogram)), 1.)).tolist()
