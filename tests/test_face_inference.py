import unittest

try:
    import numpy
except ImportError:  # The Mac test environment intentionally has no heavy ML stack.
    numpy = None

from robot.jetson.perception.face_inference import YuNetFaceDetector


def empty_outputs():
    outputs = {}
    for stride, cells in ((8, 6400), (16, 1600), (32, 400)):
        outputs["cls_{0}".format(stride)] = numpy.zeros((1, cells, 1), numpy.float32)
        outputs["obj_{0}".format(stride)] = numpy.zeros((1, cells, 1), numpy.float32)
        outputs["bbox_{0}".format(stride)] = numpy.zeros((1, cells, 4), numpy.float32)
        outputs["kps_{0}".format(stride)] = numpy.zeros((1, cells, 10), numpy.float32)
    return outputs


def decoder(threshold=0.75):
    item = YuNetFaceDetector.__new__(YuNetFaceDetector)
    item.input_width = 640
    item.input_height = 640
    item.confidence_threshold = threshold
    return item


@unittest.skipUnless(numpy is not None, "numpy is exercised on the Jetson")
class YuNetDecodeTests(unittest.TestCase):
    def test_empty_outputs_produce_no_candidates(self):
        self.assertEqual([], decoder().decode(empty_outputs()))

    def test_score_is_geometric_mean_of_class_and_objectness(self):
        outputs = empty_outputs()
        outputs["cls_8"][0, 0, 0] = 0.81
        outputs["obj_8"][0, 0, 0] = 1.0

        face = decoder().decode(outputs)[0]

        self.assertAlmostEqual(0.9, face[4], places=5)

    def test_box_and_landmarks_follow_official_yunet_decode(self):
        outputs = empty_outputs()
        index = 2 * 80 + 3
        outputs["cls_8"][0, index, 0] = 1.0
        outputs["obj_8"][0, index, 0] = 1.0
        outputs["bbox_8"][0, index] = (0.5, 0.5, 0.0, 0.0)
        outputs["kps_8"][0, index, :4] = (0.25, 0.5, 0.75, 0.5)

        face = decoder().decode(outputs)[0]

        self.assertEqual((24.0, 16.0, 32.0, 24.0), face[:4])
        self.assertEqual((26.0, 20.0), face[5][0])
        self.assertEqual((30.0, 20.0), face[5][1])

    def test_values_below_threshold_are_removed(self):
        outputs = empty_outputs()
        outputs["cls_8"][0, 0, 0] = 0.25
        outputs["obj_8"][0, 0, 0] = 1.0

        self.assertEqual([], decoder(0.75).decode(outputs))
