"""Model-loading and inference failure handling.

These tests run on a machine with no CUDA, no TensorRT, and no OpenCV, so
they cover the paths that must behave correctly when inference is unavailable
or fails -- which is exactly when the node most needs to stay alive.
"""

import unittest

from robot.jetson.perception.detector_health import DetectorHealth
from robot.jetson.perception.inference import InferenceError
from robot.jetson.perception.inference import open_backend


class MissingModelTests(unittest.TestCase):
    def test_a_missing_engine_is_reported_clearly(self):
        with self.assertRaises(InferenceError) as caught:
            open_backend("tensorrt", "/no/such.engine", "", (416, 416))

        self.assertIn("/no/such.engine", str(caught.exception))

    def test_a_missing_onnx_is_reported_clearly(self):
        with self.assertRaises(InferenceError) as caught:
            open_backend("cpu", "", "/no/such.onnx", (416, 416))

        self.assertIn("/no/such.onnx", str(caught.exception))

    def test_an_explicit_tensorrt_request_is_not_silently_downgraded(self):
        # Asking for the GPU and quietly getting the CPU would make the
        # reported provider a lie, so a strict request must raise instead.
        with self.assertRaises(InferenceError):
            open_backend("tensorrt", "/no/such.engine", "/no/such.onnx", (416, 416))

    def test_auto_reports_the_tensorrt_failure_before_falling_back(self):
        messages = []

        with self.assertRaises(InferenceError):
            open_backend(
                "auto", "/no/such.engine", "/no/such.onnx", (416, 416), logger=messages.append
            )

        self.assertTrue(messages)
        self.assertIn("/no/such.engine", messages[0])

    def test_auto_with_no_model_at_all_still_raises(self):
        with self.assertRaises(InferenceError):
            open_backend("auto", "/no/such.engine", "/no/such.onnx", (416, 416))


class FailingBackend(object):
    """A backend whose forward pass always fails."""

    provider = "failing"
    device_name = "none"

    def __init__(self, exception):
        self.exception = exception
        self.calls = 0
        self.closed = False

    def infer(self, tensor):
        self.calls += 1
        raise self.exception

    def close(self):
        self.closed = True


class InferenceErrorHandlingTests(unittest.TestCase):
    """One bad frame must be counted and survived, never fatal."""

    def test_an_inference_error_is_counted(self):
        health = DetectorHealth("m", "p")

        health.record_inference_error("InferenceError: execute_async_v3 returned false")

        self.assertEqual(1, health.inference_errors)

    def test_repeated_errors_accumulate(self):
        health = DetectorHealth("m", "p")
        for _ in range(5):
            health.record_inference_error("boom")

        self.assertEqual(5, health.inference_errors)
        self.assertEqual(5, health.status(1.0)["inference_errors"])

    def test_the_last_error_is_reported_in_status(self):
        health = DetectorHealth("m", "p")
        health.record_inference_error("InferenceError: cudaMemcpyAsync failed")

        self.assertIn("cudaMemcpyAsync", health.status(1.0)["last_error"])

    def test_errors_do_not_advance_the_inference_rate(self):
        health = DetectorHealth("m", "p")
        for _ in range(10):
            health.record_inference_error("boom")

        self.assertEqual(0.0, health.inference_rate())
        self.assertEqual(0, health.inferences)

    def test_a_recovered_inference_is_still_recorded_after_errors(self):
        health = DetectorHealth("m", "p")
        health.record_inference_error("boom")
        health.record_inference(1.0, 0.02, 1)
        health.record_inference(1.1, 0.02, 1)

        self.assertEqual(1, health.inference_errors)
        self.assertEqual(2, health.inferences)

    def test_a_failing_backend_raises_rather_than_returning_garbage(self):
        backend = FailingBackend(InferenceError("cuda launch failed"))

        with self.assertRaises(InferenceError):
            backend.infer(None)

        self.assertEqual(1, backend.calls)

    def test_a_backend_can_fail_with_an_unexpected_exception_type(self):
        backend = FailingBackend(MemoryError("out of device memory"))

        with self.assertRaises(MemoryError):
            backend.infer(None)

    def test_closing_a_backend_releases_it(self):
        backend = FailingBackend(InferenceError("boom"))

        backend.close()

        self.assertTrue(backend.closed)
