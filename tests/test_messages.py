import unittest

from robot.jetson.perception.detections import Detection
from robot.jetson.perception.messages import MessageFactories
from robot.jetson.perception.messages import annotation_plan
from robot.jetson.perception.messages import build_detection_array


class FakeStamp(object):
    def __init__(self, sec, nanosec):
        self.sec = sec
        self.nanosec = nanosec

    def __eq__(self, other):
        return (self.sec, self.nanosec) == (other.sec, other.nanosec)

    def __repr__(self):
        return "FakeStamp({0},{1})".format(self.sec, self.nanosec)


class FakeHeader(object):
    def __init__(self):
        self.stamp = None
        self.frame_id = ""


class FakePoint(object):
    def __init__(self):
        self.x = 0.0
        self.y = 0.0


class FakePose2D(object):
    def __init__(self):
        self.position = FakePoint()
        self.theta = 0.0


class FakeBoundingBox(object):
    def __init__(self):
        self.center = FakePose2D()
        self.size_x = 0.0
        self.size_y = 0.0


class FakeHypothesisCore(object):
    def __init__(self):
        self.class_id = ""
        self.score = 0.0


class FakeHypothesis(object):
    def __init__(self):
        self.hypothesis = FakeHypothesisCore()
        self.pose = None


class FakeDetection(object):
    def __init__(self):
        self.header = FakeHeader()
        self.results = []
        self.bbox = None
        self.id = ""


class FakeDetectionArray(object):
    def __init__(self):
        self.header = FakeHeader()
        self.detections = []


FACTORIES = MessageFactories(
    FakeDetectionArray, FakeDetection, FakeBoundingBox, FakeHypothesis
)

STAMP = FakeStamp(1772000000, 123456789)
FRAME_ID = "camera_optical_frame"


def person(x1, y1, x2, y2, score=0.9):
    return Detection(x1, y1, x2, y2, score, 0, "person")


class EmptyDetectionArrayTests(unittest.TestCase):
    """An empty array is still published: it means "nobody here", not "stopped"."""

    def test_an_empty_result_still_produces_a_message(self):
        message = build_detection_array([], STAMP, FRAME_ID, FACTORIES)

        self.assertEqual([], message.detections)

    def test_an_empty_result_still_carries_the_source_stamp(self):
        message = build_detection_array([], STAMP, FRAME_ID, FACTORIES)

        self.assertEqual(STAMP, message.header.stamp)
        self.assertEqual(FRAME_ID, message.header.frame_id)


class DetectionArrayTests(unittest.TestCase):
    def test_one_person_produces_one_detection(self):
        message = build_detection_array([person(10, 20, 110, 220)], STAMP, FRAME_ID, FACTORIES)

        self.assertEqual(1, len(message.detections))

    def test_multiple_people_produce_multiple_detections(self):
        people = [person(10, 20, 110, 220), person(300, 20, 400, 220), person(500, 20, 600, 220)]

        message = build_detection_array(people, STAMP, FRAME_ID, FACTORIES)

        self.assertEqual(3, len(message.detections))

    def test_the_box_is_published_as_a_centre_and_a_size(self):
        message = build_detection_array([person(10, 20, 110, 220)], STAMP, FRAME_ID, FACTORIES)
        box = message.detections[0].bbox

        self.assertAlmostEqual(60.0, box.center.position.x)
        self.assertAlmostEqual(120.0, box.center.position.y)
        self.assertAlmostEqual(100.0, box.size_x)
        self.assertAlmostEqual(200.0, box.size_y)

    def test_the_box_has_no_rotation(self):
        message = build_detection_array([person(10, 20, 110, 220)], STAMP, FRAME_ID, FACTORIES)

        self.assertEqual(0.0, message.detections[0].bbox.center.theta)

    def test_the_class_and_score_are_published(self):
        message = build_detection_array(
            [person(10, 20, 110, 220, score=0.87)], STAMP, FRAME_ID, FACTORIES
        )
        hypothesis = message.detections[0].results[0]

        self.assertEqual("person", hypothesis.hypothesis.class_id)
        self.assertAlmostEqual(0.87, hypothesis.hypothesis.score)

    def test_each_detection_carries_exactly_one_hypothesis(self):
        message = build_detection_array([person(10, 20, 110, 220)], STAMP, FRAME_ID, FACTORIES)

        self.assertEqual(1, len(message.detections[0].results))

    def test_the_detection_id_names_the_class(self):
        message = build_detection_array([person(10, 20, 110, 220)], STAMP, FRAME_ID, FACTORIES)

        self.assertEqual("person", message.detections[0].id)

    def test_the_pose_is_left_unset_because_the_camera_is_uncalibrated(self):
        message = build_detection_array([person(10, 20, 110, 220)], STAMP, FRAME_ID, FACTORIES)

        self.assertIsNone(message.detections[0].results[0].pose)


class TimestampPropagationTests(unittest.TestCase):
    """Every detection must be traceable to the exact source frame."""

    def test_the_array_header_uses_the_source_stamp(self):
        message = build_detection_array([person(10, 20, 110, 220)], STAMP, FRAME_ID, FACTORIES)

        self.assertEqual(STAMP, message.header.stamp)

    def test_each_detection_uses_the_source_stamp(self):
        people = [person(10, 20, 110, 220), person(300, 20, 400, 220)]

        message = build_detection_array(people, STAMP, FRAME_ID, FACTORIES)

        for detection in message.detections:
            self.assertEqual(STAMP, detection.header.stamp)

    def test_each_detection_uses_the_source_frame_id(self):
        people = [person(10, 20, 110, 220), person(300, 20, 400, 220)]

        message = build_detection_array(people, STAMP, FRAME_ID, FACTORIES)

        for detection in message.detections:
            self.assertEqual(FRAME_ID, detection.header.frame_id)

    def test_a_different_frame_id_is_carried_through(self):
        message = build_detection_array([person(10, 20, 110, 220)], STAMP, "other_frame", FACTORIES)

        self.assertEqual("other_frame", message.header.frame_id)
        self.assertEqual("other_frame", message.detections[0].header.frame_id)

    def test_the_stamp_is_not_replaced_by_a_fresh_clock_reading(self):
        old = FakeStamp(1, 0)

        message = build_detection_array([person(10, 20, 110, 220)], old, FRAME_ID, FACTORIES)

        self.assertEqual(1, message.header.stamp.sec)


def measure(caption):
    """Predictable stand-in for OpenCV's getTextSize."""
    return (8 * len(caption), 14)


class AnnotationPlanTests(unittest.TestCase):
    def test_no_detections_produce_no_drawing(self):
        self.assertEqual([], annotation_plan([], 640, 480, measure))

    def test_one_detection_produces_one_box(self):
        plan = annotation_plan([person(10, 20, 110, 220)], 640, 480, measure)

        self.assertEqual(1, len(plan))
        self.assertEqual((10, 20, 110, 220), (plan[0].x1, plan[0].y1, plan[0].x2, plan[0].y2))

    def test_multiple_detections_produce_multiple_boxes(self):
        people = [person(10, 20, 110, 220), person(300, 20, 400, 220)]

        self.assertEqual(2, len(annotation_plan(people, 640, 480, measure)))

    def test_the_caption_shows_the_label_and_confidence(self):
        plan = annotation_plan([person(10, 20, 110, 220, score=0.876)], 640, 480, measure)

        self.assertEqual("person 0.88", plan[0].caption)

    def test_coordinates_are_rounded_to_whole_pixels(self):
        plan = annotation_plan([person(10.4, 20.6, 110.5, 219.5)], 640, 480, measure)

        self.assertIsInstance(plan[0].x1, int)
        self.assertEqual(10, plan[0].x1)
        self.assertEqual(21, plan[0].y1)

    def test_a_box_is_clamped_to_the_image(self):
        plan = annotation_plan([person(-50, -20, 900, 700)], 640, 480, measure)

        self.assertEqual((0, 0, 640, 480), (plan[0].x1, plan[0].y1, plan[0].x2, plan[0].y2))

    def test_a_label_at_the_top_edge_moves_inside_the_box(self):
        plan = annotation_plan([person(10, 2, 110, 220)], 640, 480, measure)

        self.assertGreaterEqual(plan[0].label_y, 0)

    def test_a_label_normally_sits_above_the_box(self):
        plan = annotation_plan([person(10, 100, 110, 220)], 640, 480, measure)

        self.assertEqual(100 - 14, plan[0].label_y)

    def test_a_label_near_the_right_edge_is_pulled_inside(self):
        plan = annotation_plan([person(600, 100, 640, 220)], 640, 480, measure)

        self.assertLessEqual(plan[0].label_x + plan[0].label_width, 640)

    def test_every_label_stays_inside_the_image(self):
        people = [
            person(0, 0, 20, 20),
            person(620, 460, 640, 480),
            person(300, 240, 340, 280),
        ]

        for box in annotation_plan(people, 640, 480, measure):
            self.assertGreaterEqual(box.label_x, 0)
            self.assertGreaterEqual(box.label_y, 0)
            self.assertLessEqual(box.label_x + box.label_width, 640)
            self.assertLessEqual(box.label_y + box.label_height, 480)
