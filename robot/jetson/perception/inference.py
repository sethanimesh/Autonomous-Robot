#!/usr/bin/env python3
"""Person-detection inference backends for the Jetson.

Two backends run the same YOLOX ONNX graph:

``TensorRTBackend``
    The real Jetson accelerator. A serialised TensorRT engine is executed on
    the GPU through the CUDA runtime. This is the intended path.

``OpenCVDnnBackend``
    A CPU fallback that reads the identical ONNX file through OpenCV's DNN
    module. It exists so a TensorRT failure degrades to something that still
    works and still reports honestly what it is.

Both report a ``provider`` string that names what actually executed, so
``/perception/status`` never claims acceleration it does not have.

Heavy imports (numpy, cv2, tensorrt, cuda) happen at call time or inside the
backend constructors, so this module can be imported for inspection on a
machine that has none of them.
"""

import json
import os
import time

# The Jetson deploys these modules flat in one directory; the test suite
# imports them as a package. Support both so the failure paths in
# open_backend() can be exercised on a development machine.
try:
    from detections import build_grid_strides
    from detections import letterbox_ratio
except ImportError:  # pragma: no cover - package-import path
    from .detections import build_grid_strides
    from .detections import letterbox_ratio

PAD_VALUE = 114
PROVIDER_TENSORRT = "tensorrt"
PROVIDER_OPENCV_CPU = "opencv_dnn_cpu"


class InferenceError(RuntimeError):
    """Raised when a model cannot be loaded or a forward pass fails."""


# ---------------------------------------------------------------------------
# TensorRT
# ---------------------------------------------------------------------------


class TensorRTBackend(object):
    """Executes a serialised TensorRT engine on the Jetson GPU."""

    def __init__(self, engine_path, expected_shape=None):
        if not os.path.isfile(engine_path):
            raise InferenceError("TensorRT engine not found: {0}".format(engine_path))

        try:
            import numpy
            import tensorrt as trt
            from cuda import cudart
        except ImportError as exc:
            raise InferenceError("TensorRT backend unavailable: {0}".format(exc))

        self._numpy = numpy
        self._cudart = cudart
        self._trt = trt

        self._logger = trt.Logger(trt.Logger.ERROR)
        self._runtime = trt.Runtime(self._logger)
        with open(engine_path, "rb") as handle:
            self._engine = self._runtime.deserialize_cuda_engine(handle.read())
        if self._engine is None:
            raise InferenceError(
                "failed to deserialise {0}; an engine is specific to the GPU and "
                "TensorRT version that built it".format(engine_path)
            )
        self._context = self._engine.create_execution_context()
        if self._context is None:
            raise InferenceError("failed to create a TensorRT execution context")

        self._input_name = None
        self._output_name = None
        for index in range(self._engine.num_io_tensors):
            name = self._engine.get_tensor_name(index)
            if self._engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
                self._input_name = name
            else:
                self._output_name = name
        if self._input_name is None or self._output_name is None:
            raise InferenceError("engine does not expose one input and one output")

        self.input_shape = tuple(self._engine.get_tensor_shape(self._input_name))
        self.output_shape = tuple(self._engine.get_tensor_shape(self._output_name))
        if len(self.input_shape) != 4:
            raise InferenceError(
                "expected a 4D NCHW input, got {0}".format(self.input_shape)
            )
        self.input_height = int(self.input_shape[2])
        self.input_width = int(self.input_shape[3])
        if expected_shape is not None:
            wanted = (int(expected_shape[0]), int(expected_shape[1]))
            if (self.input_height, self.input_width) != wanted:
                raise InferenceError(
                    "engine input is {0}x{1} but the configuration asks for {2}x{3}; "
                    "rebuild the engine or fix model_input_width/height".format(
                        self.input_width, self.input_height, wanted[1], wanted[0]
                    )
                )

        self._input_dtype = self._trt_dtype_to_numpy(
            self._engine.get_tensor_dtype(self._input_name)
        )
        self._output_dtype = self._trt_dtype_to_numpy(
            self._engine.get_tensor_dtype(self._output_name)
        )

        self._host_output = numpy.empty(self.output_shape, dtype=self._output_dtype)
        self._input_bytes = int(
            numpy.prod(self.input_shape) * numpy.dtype(self._input_dtype).itemsize
        )
        self._output_bytes = int(self._host_output.nbytes)

        self._device_input = self._malloc(self._input_bytes)
        self._device_output = self._malloc(self._output_bytes)
        err, self._stream = cudart.cudaStreamCreate()
        self._check(err, "cudaStreamCreate")

        self._context.set_tensor_address(self._input_name, int(self._device_input))
        self._context.set_tensor_address(self._output_name, int(self._device_output))

        self.precision = self._read_precision(engine_path)
        self.provider = PROVIDER_TENSORRT
        if self.precision != "unknown":
            self.provider = "{0}_{1}".format(PROVIDER_TENSORRT, self.precision)
        self.device_name = self._device_name()
        self._closed = False

    def _read_precision(self, engine_path):
        """Report the precision the engine actually runs at.

        The metadata sidecar written by build_engine.py is preferred because
        it was produced from the built artifact's own tensor datatypes. When
        it is missing the engine is inspected directly. If neither can answer,
        this returns "unknown" and the provider string simply says
        "tensorrt" -- it never guesses a precision it has not confirmed.
        """
        sidecar = engine_path + ".json"
        if os.path.isfile(sidecar):
            try:
                with open(sidecar) as handle:
                    recorded = json.load(handle).get("measured_precision")
                if recorded in ("fp16", "fp32", "int8"):
                    return recorded
            except (ValueError, OSError):
                pass
        try:
            import build_engine
        except ImportError:  # pragma: no cover - package-import path
            try:
                from . import build_engine
            except ImportError:
                return "unknown"
        try:
            return build_engine.measured_precision(
                build_engine.tensor_precisions(self._engine, self._trt)
            )
        except Exception:  # pragma: no cover - inspector is advisory only
            return "unknown"

    def _device_name(self):
        try:
            err, props = self._cudart.cudaGetDeviceProperties(0)
            self._check(err, "cudaGetDeviceProperties")
            return props.name.decode("utf-8", "replace")
        except Exception:  # pragma: no cover
            return "unknown"

    def _trt_dtype_to_numpy(self, dtype):
        numpy = self._numpy
        trt = self._trt
        mapping = {
            trt.DataType.FLOAT: numpy.float32,
            trt.DataType.HALF: numpy.float16,
            trt.DataType.INT32: numpy.int32,
            trt.DataType.INT8: numpy.int8,
        }
        if dtype not in mapping:
            raise InferenceError("unsupported TensorRT tensor dtype {0}".format(dtype))
        return mapping[dtype]

    def _check(self, err, what):
        cudart = self._cudart
        if err != cudart.cudaError_t.cudaSuccess:
            raise InferenceError("{0} failed: {1}".format(what, err))

    def _malloc(self, nbytes):
        err, pointer = self._cudart.cudaMalloc(nbytes)
        self._check(err, "cudaMalloc({0})".format(nbytes))
        return pointer

    def infer(self, tensor):
        """Run one forward pass on an NCHW float32 array, returning the output."""
        if self._closed:
            raise InferenceError("backend is closed")
        numpy = self._numpy
        cudart = self._cudart

        contiguous = numpy.ascontiguousarray(tensor, dtype=self._input_dtype)
        if contiguous.nbytes != self._input_bytes:
            raise InferenceError(
                "input is {0} bytes but the engine expects {1}".format(
                    contiguous.nbytes, self._input_bytes
                )
            )

        err = cudart.cudaMemcpyAsync(
            self._device_input,
            contiguous.ctypes.data,
            self._input_bytes,
            cudart.cudaMemcpyKind.cudaMemcpyHostToDevice,
            self._stream,
        )[0]
        self._check(err, "cudaMemcpyAsync host->device")

        if not self._context.execute_async_v3(stream_handle=int(self._stream)):
            raise InferenceError("TensorRT execute_async_v3 returned false")

        err = cudart.cudaMemcpyAsync(
            self._host_output.ctypes.data,
            self._device_output,
            self._output_bytes,
            cudart.cudaMemcpyKind.cudaMemcpyDeviceToHost,
            self._stream,
        )[0]
        self._check(err, "cudaMemcpyAsync device->host")

        self._check(cudart.cudaStreamSynchronize(self._stream)[0], "cudaStreamSynchronize")
        return self._host_output

    def close(self):
        if getattr(self, "_closed", True):
            return
        self._closed = True
        cudart = self._cudart
        for pointer in (self._device_input, self._device_output):
            try:
                cudart.cudaFree(pointer)
            except Exception:  # pragma: no cover - teardown guard
                pass
        try:
            cudart.cudaStreamDestroy(self._stream)
        except Exception:  # pragma: no cover
            pass
        self._context = None
        self._engine = None
        self._runtime = None


# ---------------------------------------------------------------------------
# OpenCV DNN fallback
# ---------------------------------------------------------------------------


class OpenCVDnnBackend(object):
    """CPU fallback that runs the same ONNX graph through OpenCV."""

    def __init__(self, onnx_path, expected_shape):
        if not os.path.isfile(onnx_path):
            raise InferenceError("ONNX model not found: {0}".format(onnx_path))
        try:
            import cv2
            import numpy
        except ImportError as exc:
            raise InferenceError("OpenCV backend unavailable: {0}".format(exc))

        self._cv2 = cv2
        self._numpy = numpy
        try:
            self._net = cv2.dnn.readNetFromONNX(onnx_path)
        except Exception as exc:
            raise InferenceError("OpenCV could not read {0}: {1}".format(onnx_path, exc))
        self._net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self._net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

        self.input_height = int(expected_shape[0])
        self.input_width = int(expected_shape[1])
        self.input_shape = (1, 3, self.input_height, self.input_width)
        self.provider = PROVIDER_OPENCV_CPU
        self.device_name = "cpu"
        self._closed = False

    def infer(self, tensor):
        if self._closed:
            raise InferenceError("backend is closed")
        numpy = self._numpy
        contiguous = numpy.ascontiguousarray(tensor, dtype=numpy.float32)
        self._net.setInput(contiguous)
        try:
            return self._net.forward()
        except Exception as exc:
            raise InferenceError("OpenCV DNN forward failed: {0}".format(exc))

    def close(self):
        self._closed = True
        self._net = None


# ---------------------------------------------------------------------------
# Detector
# ---------------------------------------------------------------------------


def open_backend(device, engine_path, onnx_path, expected_shape, logger=None):
    """Select a backend, honouring an explicit request and reporting failures.

    ``auto`` tries TensorRT and falls back to OpenCV on the CPU, recording why.
    ``tensorrt`` and ``cpu`` are strict: if the requested backend cannot open,
    the error is raised rather than silently downgraded.
    """

    def note(message):
        if logger is not None:
            logger(message)

    if device == "cpu":
        return OpenCVDnnBackend(onnx_path, expected_shape), ""

    try:
        return TensorRTBackend(engine_path, expected_shape), ""
    except InferenceError as exc:
        if device == "tensorrt":
            raise
        note("TensorRT backend unavailable, falling back to CPU: {0}".format(exc))
        backend = OpenCVDnnBackend(onnx_path, expected_shape)
        return backend, str(exc)


class PersonDetector(object):
    """Preprocess, run one backend, and decode YOLOX output into candidates.

    The model is loaded and warmed up exactly once, in the constructor. Frame
    processing never reloads it.
    """

    def __init__(
        self,
        backend,
        input_height,
        input_width,
        confidence_threshold,
        warmup_iterations=5,
    ):
        import numpy

        self._numpy = numpy
        self.backend = backend
        self.input_height = int(input_height)
        self.input_width = int(input_width)
        self.confidence_threshold = float(confidence_threshold)
        self.provider = backend.provider
        self.device_name = getattr(backend, "device_name", "unknown")

        grid = build_grid_strides(self.input_height, self.input_width)
        self._grid_x = numpy.array([item[0] for item in grid], dtype=numpy.float32)
        self._grid_y = numpy.array([item[1] for item in grid], dtype=numpy.float32)
        self._strides = numpy.array([item[2] for item in grid], dtype=numpy.float32)
        self._anchor_count = len(grid)

        self.warmup_seconds = self._warm_up(warmup_iterations)

    def _warm_up(self, iterations):
        """Run the model on synthetic input so the first real frame is not slow."""
        numpy = self._numpy
        blank = numpy.full(
            (1, 3, self.input_height, self.input_width), float(PAD_VALUE), dtype=numpy.float32
        )
        started = time.monotonic()
        for _ in range(max(1, int(iterations))):
            output = self.backend.infer(blank)
        elapsed = time.monotonic() - started
        self._validate_output(output)
        return elapsed

    def _validate_output(self, output):
        numpy = self._numpy
        array = numpy.asarray(output)
        if array.ndim == 3:
            anchors, attributes = array.shape[1], array.shape[2]
        elif array.ndim == 2:
            anchors, attributes = array.shape[0], array.shape[1]
        else:
            raise InferenceError("unexpected model output rank {0}".format(array.ndim))
        if anchors != self._anchor_count:
            raise InferenceError(
                "model produced {0} anchors but a {1}x{2} YOLOX grid has {3}".format(
                    anchors, self.input_width, self.input_height, self._anchor_count
                )
            )
        if attributes < 6:
            raise InferenceError(
                "model output has {0} attributes; expected 5 + class scores".format(
                    attributes
                )
            )
        self._class_count = attributes - 5

    def preprocess(self, image):
        """Letterbox a BGR image into an NCHW float32 tensor.

        Returns the tensor and the scale factor that was applied, which the
        caller divides by to map boxes back to source pixels. YOLOX is trained
        on raw 0-255 BGR values, so no mean subtraction or scaling is applied.
        """
        import cv2

        numpy = self._numpy
        source_height, source_width = image.shape[:2]
        ratio = letterbox_ratio(
            source_width, source_height, self.input_width, self.input_height
        )
        new_width = int(source_width * ratio)
        new_height = int(source_height * ratio)

        canvas = numpy.full(
            (self.input_height, self.input_width, 3), PAD_VALUE, dtype=numpy.uint8
        )
        if new_width > 0 and new_height > 0:
            canvas[:new_height, :new_width] = cv2.resize(
                image, (new_width, new_height), interpolation=cv2.INTER_LINEAR
            )
        tensor = canvas.transpose(2, 0, 1)[numpy.newaxis, ...]
        return numpy.ascontiguousarray(tensor, dtype=numpy.float32), ratio

    def decode(self, output):
        """Turn raw YOLOX output into (cx, cy, w, h, score, class_id) tuples.

        Scores are thresholded on the GPU-sized arrays first, so only the
        handful of surviving rows are ever converted to Python objects.
        """
        numpy = self._numpy
        predictions = numpy.asarray(output)
        if predictions.ndim == 3:
            predictions = predictions[0]

        objectness = predictions[:, 4]
        class_scores = predictions[:, 5:]
        best_class = class_scores.argmax(axis=1)
        best_score = class_scores[numpy.arange(class_scores.shape[0]), best_class]
        scores = objectness * best_score

        keep = scores >= self.confidence_threshold
        if not keep.any():
            return []

        boxes = predictions[keep, :4]
        grid_x = self._grid_x[keep]
        grid_y = self._grid_y[keep]
        strides = self._strides[keep]

        center_x = (boxes[:, 0] + grid_x) * strides
        center_y = (boxes[:, 1] + grid_y) * strides
        width = numpy.exp(boxes[:, 2]) * strides
        height = numpy.exp(boxes[:, 3]) * strides

        return list(
            zip(
                center_x.tolist(),
                center_y.tolist(),
                width.tolist(),
                height.tolist(),
                scores[keep].tolist(),
                best_class[keep].tolist(),
            )
        )

    def infer_candidates(self, image):
        """Full forward pass for one BGR image: returns (candidates, ratio)."""
        tensor, ratio = self.preprocess(image)
        output = self.backend.infer(tensor)
        return self.decode(output), ratio

    def close(self):
        if self.backend is not None:
            self.backend.close()
            self.backend = None
