from pathlib import Path
import unittest
try:
    import cv2
    import numpy
    AVAILABLE = True
except ImportError:
    AVAILABLE = False
from robot.mac.view_reference import match_view_reference


@unittest.skipUnless(AVAILABLE, 'OpenCV and NumPy are required')
class ViewReferenceTests(unittest.TestCase):
    def test_confirmed_overhead_repeats_match_and_floor_room_views_do_not(self):
        root = Path(__file__).resolve().parents[1] / 'docs/calibration/view_references'
        reference = (root / 'overhead_confirmed.jpg').read_bytes()
        # A distant upper view may legitimately be unknown: the check is for
        # matching appearance, not a general-purpose ceiling classifier.
        for name in ('overhead_confirmed.jpg', 'overhead_repeat_b.jpg'):
            with self.subTest(name=name):
                self.assertTrue(match_view_reference((root/name).read_bytes(), reference)['verified'])
        for name in ('room_negative.jpg', 'floor_negative.jpg', 'floor_other_heading.jpg'):
            with self.subTest(name=name):
                self.assertFalse(match_view_reference((root/name).read_bytes(), reference)['verified'])

    def test_blank_or_invalid_images_cannot_match(self):
        root = Path(__file__).resolve().parents[1] / 'docs/calibration/view_references'
        reference = (root/'overhead_confirmed.jpg').read_bytes()
        _, blank = cv2.imencode('.jpg', numpy.full((480,640,3), 128, numpy.uint8))
        self.assertFalse(match_view_reference(blank.tobytes(), reference)['verified'])
        with self.assertRaises(ValueError):
            match_view_reference(b'not an image', reference)
