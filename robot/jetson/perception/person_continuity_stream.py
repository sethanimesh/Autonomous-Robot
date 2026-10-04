"""Exact-frame input adapter for stopped, face-anchored person retention.

This module has no ROS imports or motor commands. A returned body hint is not
fresh face confirmation, a distance estimate, or permission to move.
"""

from collections import OrderedDict
import math

try:
    from robot.jetson.perception.person_continuity import (
        PersonContinuity, person_appearance, valid_box,
    )
except ImportError:
    from person_continuity import PersonContinuity, person_appearance, valid_box


def _finite_number(value):
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value))


def _source_key(message):
    header = message.header
    sec, nanosec = header.stamp.sec, header.stamp.nanosec
    if (not isinstance(header.frame_id, str) or not header.frame_id
            or not isinstance(sec, int) or isinstance(sec, bool) or sec < 0
            or not isinstance(nanosec, int) or isinstance(nanosec, bool)
            or not 0 <= nanosec < 1_000_000_000):
        raise ValueError('invalid frame identity')
    return header.frame_id, sec, nanosec


def _boxes(message, width, height, class_id):
    boxes = []
    for detection in message.detections:
        try:
            if not any(result.hypothesis.class_id == class_id
                       for result in detection.results):
                continue
            box = detection.bbox
            x, y = box.center.position.x, box.center.position.y
            w, h = box.size_x, box.size_y
            if not all(_finite_number(value) for value in (x, y, w, h)) or min(w, h) <= 0:
                continue
            normalized = (max(0., (x-w/2)/width), max(0., (y-h/2)/height),
                          min(1., (x+w/2)/width), min(1., (y+h/2)/height))
            if valid_box(normalized):
                boxes.append(normalized)
        except (AttributeError, TypeError, ValueError):
            continue
    return boxes


class PersonContinuityStream:
    """Join camera, people and target matches independently of arrival order.

    Call ``reset(now)`` on movement, camera reference changes or target changes.
    ``now`` must use the camera's source clock, rather than monotonic time. A
    reset rejects delayed frames captured at or before that reset boundary.
    Histograms and the bounded image cache remain in memory only.
    """

    def __init__(self, maximum_face_age_seconds=10., capacity=24, required_face_hits=3):
        if isinstance(capacity, bool) or not isinstance(capacity, int) or capacity < 1:
            raise ValueError('cache capacity must be a positive integer')
        self.capacity = capacity
        self.tracker = PersonContinuity(maximum_face_age_seconds, required_face_hits)
        self.cache = {name: OrderedDict() for name in ('image', 'people', 'matches')}
        self.not_before = float('-inf')
        self.layout = None
        self.latest = None

    def reset(self, now):
        if not _finite_number(now):
            raise ValueError('reset requires a finite source time')
        self.not_before = max(self.not_before, now)
        self.tracker.reset()
        self.latest = None
        self.layout = None
        for cache in self.cache.values():
            cache.clear()

    def offer(self, name, message, now):
        """Offer one image, people array or matches array; return a joined result.

        Missing streams do not reuse old detections. Inspect ``observed(now)``
        for retention: processing a frame does not itself imply usable evidence.
        """
        if name not in self.cache:
            raise ValueError('unknown continuity stream')
        if not _finite_number(now):
            raise ValueError('observation requires a finite source time')
        try:
            key = _source_key(message)
        except (AttributeError, TypeError, ValueError):
            self.reset(now)
            return None
        stamp = key[1] + key[2] / 1e9
        if stamp <= self.not_before:
            return None
        own = self.cache[name]
        own.pop(key, None)
        own[key] = message
        while len(own) > self.capacity:
            own.popitem(last=False)
        if not all(key in cache for cache in self.cache.values()):
            return None
        frame, people, matches = (self.cache[kind].pop(key)
                                  for kind in ('image', 'people', 'matches'))
        try:
            layout = (key[0], frame.width, frame.height, frame.encoding)
            if self.layout is not None and layout != self.layout:
                self.reset(now)
                return None
            self.layout = layout
            if (not isinstance(frame.width, int) or not isinstance(frame.height, int)
                    or isinstance(frame.width, bool) or isinstance(frame.height, bool)
                    or min(frame.width, frame.height) <= 0
                    or not isinstance(frame.step, int) or isinstance(frame.step, bool)
                    or frame.step < frame.width * 3 or frame.encoding not in ('rgb8', 'bgr8')
                    or len(frame.data) != frame.height * frame.step):
                raise ValueError('invalid image layout')
            if not -.05 <= now-stamp <= .75:
                result = self.tracker.observe([], [], stamp, now)
                if result['state'] != 'ignored':
                    self.latest = dict(result, source_time=stamp)
                return dict(result, source_time=stamp)
            import numpy as np

            pixels = np.frombuffer(frame.data, np.uint8).reshape(frame.height, frame.step)
            pixels = pixels[:, :frame.width * 3].reshape(frame.height, frame.width, 3)
            if frame.encoding == 'rgb8':
                pixels = pixels[:, :, ::-1]
            persons = []
            for box in _boxes(people, frame.width, frame.height, 'person'):
                descriptor = person_appearance(pixels, box)
                if descriptor is not None:
                    persons.append({'box': box, 'appearance': descriptor})
            faces = _boxes(matches, frame.width, frame.height, 'target_person')
        except (AttributeError, TypeError, ValueError):
            self.reset(now)
            return None
        result = self.tracker.observe(persons, faces, stamp, now)
        if result['state'] != 'ignored':
            self.latest = dict(result, source_time=stamp)
        return dict(result, source_time=stamp)

    def observed(self, now):
        """Return only an established, current body hint; never a face box.

        Freshness is checked on every read, including while a stream is silent.
        Missing/ambiguous bodies expose no old position. The original face
        anchor expires even when body observations continue arriving.
        """
        if not _finite_number(now) or self.latest is None:
            return None
        if self.latest['state'] not in ('face_confirmed', 'body_continuity'):
            return None
        stamp, face_at = self.latest['source_time'], self.tracker.face_at
        if face_at is None or not -.05 <= now-stamp <= .75:
            return None
        if not -.05 <= now-face_at <= self.tracker.maximum_face_age_seconds:
            return None
        return {
            'state': self.latest['state'],
            'source_time': stamp,
            'source_age_seconds': max(0., now-stamp),
            'face_age_seconds': max(0., now-face_at),
            'body_box': list(self.latest['body_box']),
        }

    def retained(self, now):
        """Return stopped-hold evidence, including a brief missing-body frame.

        A missing-body result contains no position and cannot be used for
        centering. Its gap expires from the last valid body, so repeated empty
        detections cannot prolong the allowance.
        """
        current = self.observed(now)
        if current is not None:
            return current
        if not _finite_number(now) or self.latest is None:
            return None
        if self.latest['state'] != 'temporarily_missing' or self.tracker.face_hits < self.tracker.required_face_hits:
            return None
        face_at, seen_at = self.tracker.face_at, self.tracker.seen_at
        stamp = self.latest['source_time']
        if (face_at is None or seen_at is None
                or not -.05 <= now-stamp <= .75
                or not -.05 <= now-seen_at <= .75
                or not -.05 <= now-face_at <= self.tracker.maximum_face_age_seconds):
            return None
        return {
            'state': 'temporarily_missing',
            'source_time': stamp,
            'source_age_seconds': max(0., now-stamp),
            'face_age_seconds': max(0., now-face_at),
            'body_gap_seconds': max(0., now-seen_at),
        }
