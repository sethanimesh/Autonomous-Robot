import unittest

from robot.jetson.mission.camera_motion_feedback import CameraMotionFeedbackError
from robot.jetson.mission.camera_motion_feedback import compare_fingerprints
from robot.jetson.mission.camera_motion_feedback import fingerprint_quality
from robot.jetson.mission.camera_motion_feedback import frame_fingerprint
from robot.jetson.mission.camera_motion_feedback import verify_camera_step


def fingerprint(values, columns=4):
    return {
        "columns": columns,
        "rows": len(values) // columns,
        "values": tuple(values),
    }


class FrameFingerprintTests(unittest.TestCase):
    def test_bgr_frame_is_sampled_as_luminance(self):
        # Two BGR pixels: black and white.
        value = frame_fingerprint(bytes([0, 0, 0, 255, 255, 255]), 2, 1, 6, columns=2, rows=1)

        self.assertEqual((0, 255), value["values"])

    def test_truncated_or_compressed_frames_are_rejected(self):
        with self.assertRaises(ValueError):
            frame_fingerprint(b"short", 10, 10, 30)
        with self.assertRaises(ValueError):
            frame_fingerprint(bytes(300), 10, 10, 30, encoding="jpeg")

    def test_quality_rejects_black_or_flat_views(self):
        self.assertFalse(fingerprint_quality(fingerprint([0] * 16))["usable"])
        self.assertFalse(fingerprint_quality(fingerprint([100] * 16))["usable"])
        self.assertTrue(
            fingerprint_quality(fingerprint([30, 200, 40, 190] * 4))["usable"]
        )

    def test_quality_rejects_a_smooth_high_contrast_closeup(self):
        smooth_gradient = [
            40, 70, 100, 130,
            45, 75, 105, 135,
            50, 80, 110, 140,
            55, 85, 115, 145,
        ]

        quality = fingerprint_quality(fingerprint(smooth_gradient))

        self.assertGreater(quality["mean_absolute_contrast"], 20)
        self.assertFalse(quality["usable"])


class MotionComparisonTests(unittest.TestCase):
    def test_semantically_verified_ceiling_still_requires_motion_and_stability(self):
        textured = fingerprint([30, 200, 40, 190] * 4)
        ceiling = fingerprint([160] * 16)
        with self.assertRaises(CameraMotionFeedbackError):
            verify_camera_step(textured, ceiling, ceiling)
        self.assertTrue(verify_camera_step(textured, ceiling, ceiling, allow_textureless_view=True)["verified"])
        for before, after, held in ((ceiling, ceiling, ceiling), (textured, fingerprint([0] * 16), fingerprint([0] * 16)), (textured, ceiling, textured)):
            with self.assertRaises(CameraMotionFeedbackError):
                verify_camera_step(before, after, held, allow_textureless_view=True)

    def test_uniform_exposure_change_is_not_camera_motion(self):
        before = fingerprint([20, 40, 60, 80] * 4)
        after = fingerprint([40, 60, 80, 100] * 4)

        result = compare_fingerprints(before, after)

        self.assertEqual(0.0, result["changed_fraction"])
        self.assertEqual(20.0, result["exposure_delta"])

    def test_scene_wide_change_then_stability_is_verified(self):
        before = fingerprint([30, 200, 40, 190] * 4)
        after = fingerprint([200, 30, 190, 40] * 4)
        settled = fingerprint([201, 31, 191, 41] * 4)

        report = verify_camera_step(before, after, settled)

        self.assertTrue(report["verified"])
        self.assertGreater(report["transition"]["changed_fraction"], 0.9)

    def test_encoder_only_motion_is_rejected_when_view_does_not_change(self):
        view = fingerprint([30, 200, 40, 190] * 4)

        with self.assertRaisesRegex(CameraMotionFeedbackError, "do not prove"):
            verify_camera_step(view, view, view)

    def test_a_camera_that_keeps_falling_is_rejected(self):
        before = fingerprint([30, 200, 40, 190] * 4)
        after = fingerprint([200, 30, 190, 40] * 4)
        drifting = fingerprint([40, 190, 30, 200] * 4)

        with self.assertRaisesRegex(CameraMotionFeedbackError, "continued moving"):
            verify_camera_step(before, after, drifting)


if __name__ == "__main__":
    unittest.main()
