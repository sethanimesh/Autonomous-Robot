import unittest

from robot.jetson.perception.face_sync import TimestampedImageCache
from robot.jetson.perception.face_sync import ExactPairMatcher
from robot.jetson.perception.face_sync import select_person_regions
from robot.jetson.perception.face_sync import stamp_key


class FakeStamp(object):
    def __init__(self, sec, nanosec):
        self.sec = sec
        self.nanosec = nanosec


class Value(object):
    pass


def detection(cx, cy, width, height, score=0.9, label="person"):
    item = Value()
    item.bbox = Value()
    item.bbox.center = Value()
    item.bbox.center.position = Value()
    item.bbox.center.position.x = cx
    item.bbox.center.position.y = cy
    item.bbox.size_x = width
    item.bbox.size_y = height
    result = Value()
    result.hypothesis = Value()
    result.hypothesis.class_id = label
    result.hypothesis.score = score
    item.results = [result]
    return item


class TimestampedImageCacheTests(unittest.TestCase):
    def test_stamp_key_is_exact_to_nanoseconds(self):
        self.assertEqual((12, 345), stamp_key(FakeStamp(12, 345)))

    def test_exact_key_returns_and_removes_frame(self):
        cache = TimestampedImageCache(3)
        cache.offer((1, 2), "frame")

        self.assertEqual("frame", cache.take((1, 2)))
        self.assertIsNone(cache.take((1, 2)))

    def test_oldest_frame_is_evicted_at_capacity(self):
        cache = TimestampedImageCache(2)
        cache.offer((1, 0), "a")
        cache.offer((2, 0), "b")
        cache.offer((3, 0), "c")

        self.assertIsNone(cache.take((1, 0)))
        self.assertEqual("b", cache.take((2, 0)))
        self.assertEqual(1, cache.evicted)

    def test_a_miss_is_counted(self):
        cache = TimestampedImageCache(2)
        cache.take((99, 0))

        self.assertEqual(1, cache.misses)


class ExactPairMatcherTests(unittest.TestCase):
    def test_matches_when_left_arrives_first(self):
        matcher = ExactPairMatcher(3)

        self.assertIsNone(matcher.offer_left((1, 2), "image"))
        self.assertEqual(("image", "detections"), matcher.offer_right((1, 2), "detections"))

    def test_matches_when_right_arrives_first(self):
        matcher = ExactPairMatcher(3)

        self.assertIsNone(matcher.offer_right((1, 2), "detections"))
        self.assertEqual(("image", "detections"), matcher.offer_left((1, 2), "image"))

    def test_nearby_but_unequal_timestamps_do_not_match(self):
        matcher = ExactPairMatcher(3)
        matcher.offer_left((1, 2), "image")

        self.assertIsNone(matcher.offer_right((1, 3), "detections"))
        self.assertEqual(0, matcher.matched)

    def test_both_sides_are_bounded_and_evictions_are_counted(self):
        matcher = ExactPairMatcher(1)
        matcher.offer_left((1, 0), "a")
        matcher.offer_left((2, 0), "b")
        matcher.offer_right((3, 0), "c")
        matcher.offer_right((4, 0), "d")

        self.assertEqual(1, matcher.left_depth)
        self.assertEqual(1, matcher.right_depth)
        self.assertEqual(2, matcher.evicted)


class PersonRegionTests(unittest.TestCase):
    def select(self, items, **overrides):
        values = dict(
            image_width=640,
            image_height=480,
            max_regions=3,
            padding_fraction=0.1,
            height_fraction=0.7,
            minimum_pixels=24,
        )
        values.update(overrides)
        return select_person_regions(items, **values)

    def test_upper_body_region_is_selected_and_padded(self):
        regions = self.select([detection(200, 250, 100, 300)])

        self.assertEqual(1, len(regions))
        self.assertEqual((140, 70, 260, 340), (regions[0].x1, regions[0].y1, regions[0].x2, regions[0].y2))

    def test_region_is_clipped_to_image(self):
        region = self.select([detection(20, 50, 100, 200)])[0]

        self.assertEqual(0, region.x1)
        self.assertEqual(0, region.y1)

    def test_non_person_hypothesis_is_ignored(self):
        self.assertEqual([], self.select([detection(100, 100, 80, 200, label="dog")]))

    def test_tiny_regions_are_skipped(self):
        self.assertEqual(
            [],
            self.select([detection(100, 100, 10, 10)], minimum_pixels=24),
        )

    def test_largest_regions_win_the_bounded_budget(self):
        items = [
            detection(50, 100, 30, 100),
            detection(150, 100, 60, 200),
            detection(300, 100, 90, 250),
        ]
        regions = self.select(items, max_regions=2)

        self.assertEqual(2, len(regions))
        self.assertGreaterEqual(regions[0].area, regions[1].area)

    def test_full_frame_fallback_is_single_and_explicit(self):
        self.assertEqual([], self.select([]))

        regions = self.select([], fallback_full_frame=True)

        self.assertEqual(1, len(regions))
        self.assertEqual((0, 0, 640, 480), (
            regions[0].x1,
            regions[0].y1,
            regions[0].x2,
            regions[0].y2,
        ))

    def test_full_frame_fallback_rejects_too_small_images(self):
        self.assertEqual(
            [],
            self.select(
                [],
                image_width=20,
                image_height=20,
                minimum_pixels=24,
                fallback_full_frame=True,
            ),
        )
