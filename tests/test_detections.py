import unittest

from robot.jetson.perception.detections import Detection
from robot.jetson.perception.detections import build_grid_strides
from robot.jetson.perception.detections import center_to_corners
from robot.jetson.perception.detections import clip_box
from robot.jetson.perception.detections import intersection_over_union
from robot.jetson.perception.detections import letterbox_ratio
from robot.jetson.perception.detections import non_maximum_suppression
from robot.jetson.perception.detections import select_person_detections

PERSON = 0
CHAIR = 56


def person(center_x, center_y, width, height, score):
    return (center_x, center_y, width, height, score, PERSON)


class GridStrideTests(unittest.TestCase):
    """The anchor grid must line up exactly with the exported ONNX output."""

    def test_416_grid_matches_the_yolox_tiny_output(self):
        self.assertEqual(3549, len(build_grid_strides(416, 416)))

    def test_640_grid_matches_the_yolox_s_output(self):
        self.assertEqual(8400, len(build_grid_strides(640, 640)))

    def test_grid_starts_at_the_finest_stride(self):
        self.assertEqual((0, 0, 8), build_grid_strides(416, 416)[0])

    def test_grid_ends_at_the_coarsest_stride(self):
        self.assertEqual((12, 12, 32), build_grid_strides(416, 416)[-1])

    def test_grid_is_row_major_within_a_stride(self):
        grid = build_grid_strides(64, 64)

        self.assertEqual((0, 0, 8), grid[0])
        self.assertEqual((1, 0, 8), grid[1])
        self.assertEqual((0, 1, 8), grid[8])

    def test_indivisible_input_is_rejected(self):
        with self.assertRaises(ValueError):
            build_grid_strides(100, 100)

    def test_non_positive_input_is_rejected(self):
        with self.assertRaises(ValueError):
            build_grid_strides(0, 416)


class LetterboxRatioTests(unittest.TestCase):
    def test_wide_image_is_limited_by_width(self):
        self.assertAlmostEqual(0.65, letterbox_ratio(640, 480, 416, 416))

    def test_square_image_uses_the_full_input(self):
        self.assertAlmostEqual(1.0, letterbox_ratio(416, 416, 416, 416))

    def test_tall_image_is_limited_by_height(self):
        self.assertAlmostEqual(0.5, letterbox_ratio(400, 832, 416, 416))

    def test_zero_size_is_rejected(self):
        with self.assertRaises(ValueError):
            letterbox_ratio(0, 480, 416, 416)


class BoundingBoxConversionTests(unittest.TestCase):
    def test_centre_form_becomes_corner_form(self):
        self.assertEqual((75.0, 150.0, 125.0, 250.0), center_to_corners(100, 200, 50, 100))

    def test_conversion_round_trips_through_the_centre(self):
        x1, y1, x2, y2 = center_to_corners(100, 200, 50, 100)

        self.assertAlmostEqual(100.0, (x1 + x2) / 2.0)
        self.assertAlmostEqual(200.0, (y1 + y2) / 2.0)

    def test_zero_size_box_collapses_to_a_point(self):
        self.assertEqual((10.0, 10.0, 10.0, 10.0), center_to_corners(10, 10, 0, 0))

    def test_detection_reports_derived_geometry(self):
        detection = Detection(10, 20, 110, 220, 0.9, PERSON, "person")

        self.assertEqual(100.0, detection.width)
        self.assertEqual(200.0, detection.height)
        self.assertEqual(60.0, detection.center_x)
        self.assertEqual(120.0, detection.center_y)
        self.assertEqual(20000.0, detection.area)


class ClippingTests(unittest.TestCase):
    def test_a_box_inside_the_image_is_untouched(self):
        self.assertEqual((10.0, 20.0, 30.0, 40.0), clip_box(10, 20, 30, 40, 640, 480))

    def test_negative_corner_is_clamped_to_zero(self):
        self.assertEqual((0.0, 0.0, 30.0, 40.0), clip_box(-50, -10, 30, 40, 640, 480))

    def test_corner_beyond_the_right_edge_is_clamped(self):
        self.assertEqual((10.0, 20.0, 640.0, 480.0), clip_box(10, 20, 900, 700, 640, 480))

    def test_a_box_entirely_left_of_the_image_collapses(self):
        x1, y1, x2, y2 = clip_box(-200, 10, -100, 50, 640, 480)

        self.assertEqual(0.0, x2 - x1)

    def test_a_box_entirely_below_the_image_collapses(self):
        x1, y1, x2, y2 = clip_box(10, 700, 50, 900, 640, 480)

        self.assertEqual(0.0, y2 - y1)

    def test_inverted_corners_are_ordered(self):
        self.assertEqual((10.0, 20.0, 30.0, 40.0), clip_box(30, 40, 10, 20, 640, 480))

    def test_a_box_exactly_on_the_boundary_is_kept(self):
        self.assertEqual((0.0, 0.0, 640.0, 480.0), clip_box(0, 0, 640, 480, 640, 480))


class IntersectionOverUnionTests(unittest.TestCase):
    def test_identical_boxes_overlap_completely(self):
        self.assertAlmostEqual(1.0, intersection_over_union((0, 0, 10, 10), (0, 0, 10, 10)))

    def test_disjoint_boxes_do_not_overlap(self):
        self.assertEqual(0.0, intersection_over_union((0, 0, 10, 10), (20, 20, 30, 30)))

    def test_touching_edges_do_not_overlap(self):
        self.assertEqual(0.0, intersection_over_union((0, 0, 10, 10), (10, 0, 20, 10)))

    def test_half_overlap_is_one_third(self):
        # Two 10x10 boxes sharing a 5x10 strip: 50 / (100 + 100 - 50).
        self.assertAlmostEqual(
            1.0 / 3.0, intersection_over_union((0, 0, 10, 10), (5, 0, 15, 10))
        )

    def test_a_zero_area_box_overlaps_nothing(self):
        self.assertEqual(0.0, intersection_over_union((5, 5, 5, 5), (0, 0, 10, 10)))


class NonMaximumSuppressionTests(unittest.TestCase):
    def test_overlapping_boxes_collapse_to_the_best(self):
        boxes = [
            Detection(0, 0, 100, 200, 0.7, PERSON, "person"),
            Detection(5, 5, 105, 205, 0.9, PERSON, "person"),
        ]

        kept = non_maximum_suppression(boxes, 0.45)

        self.assertEqual(1, len(kept))
        self.assertAlmostEqual(0.9, kept[0].score)

    def test_separated_boxes_both_survive(self):
        boxes = [
            Detection(0, 0, 100, 200, 0.7, PERSON, "person"),
            Detection(300, 0, 400, 200, 0.6, PERSON, "person"),
        ]

        self.assertEqual(2, len(non_maximum_suppression(boxes, 0.45)))

    def test_results_are_ordered_by_descending_score(self):
        boxes = [
            Detection(0, 0, 100, 200, 0.5, PERSON, "person"),
            Detection(300, 0, 400, 200, 0.9, PERSON, "person"),
            Detection(600, 0, 700, 200, 0.7, PERSON, "person"),
        ]

        scores = [item.score for item in non_maximum_suppression(boxes, 0.45)]

        self.assertEqual([0.9, 0.7, 0.5], scores)

    def test_an_empty_input_yields_an_empty_result(self):
        self.assertEqual([], non_maximum_suppression([], 0.45))

    def test_a_high_threshold_keeps_both_overlapping_boxes(self):
        boxes = [
            Detection(0, 0, 100, 200, 0.7, PERSON, "person"),
            Detection(5, 5, 105, 205, 0.9, PERSON, "person"),
        ]

        self.assertEqual(2, len(non_maximum_suppression(boxes, 0.99)))


class PersonSelectionTests(unittest.TestCase):
    """End-to-end candidate filtering, the path every real frame takes."""

    def select(self, candidates, **kwargs):
        options = {
            "person_class_id": PERSON,
            "confidence_threshold": 0.45,
            "iou_threshold": 0.45,
            "image_width": 640,
            "image_height": 480,
            "ratio": 1.0,
        }
        options.update(kwargs)
        return select_person_detections(candidates, **options)

    def test_no_candidates_produce_no_detections(self):
        self.assertEqual([], self.select([]))

    def test_a_scene_with_no_person_produces_no_detections(self):
        chair = (100, 100, 50, 50, 0.98, CHAIR)
        table = (300, 300, 80, 80, 0.91, 60)

        self.assertEqual([], self.select([chair, table]))

    def test_non_person_classes_are_removed_even_at_high_confidence(self):
        candidates = [(100, 100, 50, 100, 0.99, CHAIR), person(200, 200, 50, 100, 0.50)]

        kept = self.select(candidates)

        self.assertEqual(1, len(kept))
        self.assertEqual(PERSON, kept[0].class_id)
        self.assertAlmostEqual(0.50, kept[0].score)

    def test_low_confidence_people_are_removed(self):
        self.assertEqual([], self.select([person(100, 100, 50, 100, 0.30)]))

    def test_a_score_exactly_at_the_threshold_is_kept(self):
        self.assertEqual(1, len(self.select([person(100, 100, 50, 100, 0.45)])))

    def test_one_person_is_converted_to_corner_form(self):
        kept = self.select([person(100, 200, 50, 100, 0.9)])

        self.assertEqual(1, len(kept))
        self.assertAlmostEqual(75.0, kept[0].x1)
        self.assertAlmostEqual(150.0, kept[0].y1)
        self.assertAlmostEqual(125.0, kept[0].x2)
        self.assertAlmostEqual(250.0, kept[0].y2)

    def test_the_letterbox_ratio_is_undone(self):
        # A box found at model scale 0.5 is twice as big in the source image.
        kept = self.select([person(100, 200, 50, 100, 0.9)], ratio=0.5)

        self.assertAlmostEqual(150.0, kept[0].x1)
        self.assertAlmostEqual(300.0, kept[0].y1)
        self.assertAlmostEqual(250.0, kept[0].x2)
        self.assertAlmostEqual(480.0, kept[0].y2)

    def test_two_separate_people_both_survive(self):
        candidates = [person(100, 200, 50, 150, 0.90), person(450, 200, 50, 150, 0.80)]

        kept = self.select(candidates)

        self.assertEqual(2, len(kept))
        self.assertAlmostEqual(0.90, kept[0].score)
        self.assertAlmostEqual(0.80, kept[1].score)

    def test_duplicate_boxes_on_one_person_collapse(self):
        candidates = [
            person(100, 200, 50, 150, 0.90),
            person(102, 203, 52, 148, 0.85),
            person(99, 198, 49, 152, 0.70),
        ]

        self.assertEqual(1, len(self.select(candidates)))

    def test_a_person_partly_outside_the_frame_is_clipped(self):
        kept = self.select([person(20, 200, 100, 150, 0.9)])

        self.assertEqual(1, len(kept))
        self.assertEqual(0.0, kept[0].x1)
        self.assertAlmostEqual(70.0, kept[0].x2)

    def test_a_person_beyond_the_frame_is_dropped_not_published_as_a_sliver(self):
        self.assertEqual([], self.select([person(-200, 200, 50, 150, 0.9)]))

    def test_detections_never_leave_the_image(self):
        candidates = [person(630, 470, 200, 200, 0.9)]

        kept = self.select(candidates)

        self.assertEqual(1, len(kept))
        self.assertLessEqual(kept[0].x2, 640.0)
        self.assertLessEqual(kept[0].y2, 480.0)
        self.assertGreaterEqual(kept[0].x1, 0.0)
        self.assertGreaterEqual(kept[0].y1, 0.0)

    def test_the_label_is_applied_to_every_detection(self):
        kept = self.select([person(100, 200, 50, 150, 0.9)], label="person")

        self.assertEqual("person", kept[0].label)

    def test_max_detections_caps_the_output(self):
        candidates = [person(60 * i + 40, 200, 40, 100, 0.9) for i in range(10)]

        kept = self.select(candidates, max_detections=3)

        self.assertEqual(3, len(kept))

    def test_a_non_default_person_class_id_is_honoured(self):
        candidates = [(100, 200, 50, 150, 0.9, 7)]

        kept = self.select(candidates, person_class_id=7)

        self.assertEqual(1, len(kept))
        self.assertEqual(7, kept[0].class_id)

    def test_a_zero_ratio_is_rejected(self):
        with self.assertRaises(ValueError):
            self.select([person(100, 200, 50, 150, 0.9)], ratio=0.0)

    def test_a_zero_size_image_is_rejected(self):
        with self.assertRaises(ValueError):
            self.select([person(100, 200, 50, 150, 0.9)], image_width=0)
