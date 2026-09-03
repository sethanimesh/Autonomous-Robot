#!/usr/bin/env python3
"""TensorRT FP16 execution and YuNet decoding for face detection."""

import json
import os
import time


class FaceInferenceError(RuntimeError):
    pass


YUNET_STRIDES = (8, 16, 32)


class TensorRTMultiOutputBackend(object):
    """TensorRT runner for one fixed-shape input and any number of outputs."""

    def __init__(self, engine_path, expected_shape):
        if not os.path.isfile(engine_path):
            raise FaceInferenceError("TensorRT engine not found: {0}".format(engine_path))
        try:
            import numpy
            import tensorrt as trt
            from cuda import cudart
        except ImportError as exc:
            raise FaceInferenceError("TensorRT backend unavailable: {0}".format(exc))

        self._numpy = numpy
        self._trt = trt
        self._cudart = cudart
        self._logger = trt.Logger(trt.Logger.ERROR)
        self._runtime = trt.Runtime(self._logger)
        with open(engine_path, "rb") as handle:
            self._engine = self._runtime.deserialize_cuda_engine(handle.read())
        if self._engine is None:
            raise FaceInferenceError("failed to deserialize {0}".format(engine_path))
        self._context = self._engine.create_execution_context()
        if self._context is None:
            raise FaceInferenceError("failed to create TensorRT execution context")

        self._input_name = None
        self._output_names = []
        for index in range(self._engine.num_io_tensors):
            name = self._engine.get_tensor_name(index)
            if self._engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
                if self._input_name is not None:
                    raise FaceInferenceError("face engine must have exactly one input")
                self._input_name = name
            else:
                self._output_names.append(name)
        if self._input_name is None or not self._output_names:
            raise FaceInferenceError("face engine input or outputs are missing")

        self.input_shape = tuple(self._engine.get_tensor_shape(self._input_name))
        if len(self.input_shape) != 4 or min(self.input_shape) <= 0:
            raise FaceInferenceError(
                "face engine needs a fixed NCHW input, got {0}".format(self.input_shape)
            )
        self.input_height = int(self.input_shape[2])
        self.input_width = int(self.input_shape[3])
        wanted = (int(expected_shape[0]), int(expected_shape[1]))
        if (self.input_height, self.input_width) != wanted:
            raise FaceInferenceError(
                "engine input is {0}x{1}, configuration asks for {2}x{3}".format(
                    self.input_width, self.input_height, wanted[1], wanted[0]
                )
            )

        input_dtype = self._dtype(self._engine.get_tensor_dtype(self._input_name))
        self._input_bytes = int(
            numpy.prod(self.input_shape) * numpy.dtype(input_dtype).itemsize
        )
        self._input_dtype = input_dtype
        self._device_input = self._malloc(self._input_bytes)
        self._context.set_tensor_address(self._input_name, int(self._device_input))

        self._host_outputs = {}
        self._device_outputs = {}
        self._output_bytes = {}
        for name in self._output_names:
            shape = tuple(self._engine.get_tensor_shape(name))
            if not shape or min(shape) <= 0:
                raise FaceInferenceError(
                    "face engine output {0} is not fixed: {1}".format(name, shape)
                )
            dtype = self._dtype(self._engine.get_tensor_dtype(name))
            host = numpy.empty(shape, dtype=dtype)
            device = self._malloc(host.nbytes)
            self._host_outputs[name] = host
            self._device_outputs[name] = device
            self._output_bytes[name] = int(host.nbytes)
            self._context.set_tensor_address(name, int(device))

        error, self._stream = cudart.cudaStreamCreate()
        self._check(error, "cudaStreamCreate")
        self.precision = self._precision(engine_path)
        self.provider = "tensorrt_{0}".format(self.precision)
        if self.precision == "unknown":
            self.provider = "tensorrt"
        self.device_name = self._device_name()
        self._closed = False

    def _dtype(self, dtype):
        mapping = {
            self._trt.DataType.FLOAT: self._numpy.float32,
            self._trt.DataType.HALF: self._numpy.float16,
            self._trt.DataType.INT32: self._numpy.int32,
            self._trt.DataType.INT8: self._numpy.int8,
        }
        if dtype not in mapping:
            raise FaceInferenceError("unsupported TensorRT datatype {0}".format(dtype))
        return mapping[dtype]

    @staticmethod
    def _precision(engine_path):
        try:
            with open(engine_path + ".json") as handle:
                value = json.load(handle).get("measured_precision", "unknown")
            if value in ("fp16", "fp32", "int8"):
                return value
        except (OSError, ValueError):
            pass
        return "unknown"

    def _device_name(self):
        try:
            error, properties = self._cudart.cudaGetDeviceProperties(0)
            self._check(error, "cudaGetDeviceProperties")
            return properties.name.decode("utf-8", "replace")
        except Exception:  # pragma: no cover - diagnostic only
            return "unknown"

    def _check(self, error, operation):
        if error != self._cudart.cudaError_t.cudaSuccess:
            raise FaceInferenceError("{0} failed: {1}".format(operation, error))

    def _malloc(self, size):
        error, pointer = self._cudart.cudaMalloc(int(size))
        self._check(error, "cudaMalloc({0})".format(size))
        return pointer

    @property
    def output_names(self):
        return tuple(self._output_names)

    def infer(self, tensor):
        if self._closed:
            raise FaceInferenceError("backend is closed")
        numpy = self._numpy
        cudart = self._cudart
        contiguous = numpy.ascontiguousarray(tensor, dtype=self._input_dtype)
        if contiguous.nbytes != self._input_bytes:
            raise FaceInferenceError(
                "input is {0} bytes, engine expects {1}".format(
                    contiguous.nbytes, self._input_bytes
                )
            )
        error = cudart.cudaMemcpyAsync(
            self._device_input,
            contiguous.ctypes.data,
            self._input_bytes,
            cudart.cudaMemcpyKind.cudaMemcpyHostToDevice,
            self._stream,
        )[0]
        self._check(error, "cudaMemcpyAsync host->device")
        if not self._context.execute_async_v3(stream_handle=int(self._stream)):
            raise FaceInferenceError("TensorRT execute_async_v3 returned false")
        for name in self._output_names:
            error = cudart.cudaMemcpyAsync(
                self._host_outputs[name].ctypes.data,
                self._device_outputs[name],
                self._output_bytes[name],
                cudart.cudaMemcpyKind.cudaMemcpyDeviceToHost,
                self._stream,
            )[0]
            self._check(error, "cudaMemcpyAsync device->host {0}".format(name))
        self._check(
            cudart.cudaStreamSynchronize(self._stream)[0], "cudaStreamSynchronize"
        )
        return self._host_outputs

    def close(self):
        if getattr(self, "_closed", True):
            return
        self._closed = True
        for pointer in [self._device_input] + list(self._device_outputs.values()):
            try:
                self._cudart.cudaFree(pointer)
            except Exception:  # pragma: no cover
                pass
        try:
            self._cudart.cudaStreamDestroy(self._stream)
        except Exception:  # pragma: no cover
            pass
        self._context = None
        self._engine = None
        self._runtime = None


class YuNetFaceDetector(object):
    STRIDES = YUNET_STRIDES
    REQUIRED_OUTPUTS = tuple(
        "{0}_{1}".format(kind, stride)
        for kind in ("cls", "obj", "bbox", "kps")
        for stride in YUNET_STRIDES
    )

    def __init__(self, backend, confidence_threshold):
        self.backend = backend
        self.provider = backend.provider
        self.device_name = backend.device_name
        self.input_height = backend.input_height
        self.input_width = backend.input_width
        self.confidence_threshold = float(confidence_threshold)
        missing = sorted(set(self.REQUIRED_OUTPUTS) - set(backend.output_names))
        if missing:
            raise FaceInferenceError(
                "YuNet engine is missing outputs: {0}".format(", ".join(missing))
            )
        import numpy

        blank = numpy.zeros(
            (1, 3, self.input_height, self.input_width), dtype=numpy.float32
        )
        started = time.monotonic()
        backend.infer(blank)
        self.warmup_seconds = time.monotonic() - started

    def preprocess(self, image):
        import cv2
        import numpy

        source_height, source_width = image.shape[:2]
        if source_width <= 0 or source_height <= 0:
            raise FaceInferenceError("cannot preprocess an empty face ROI")
        ratio = min(
            float(self.input_width) / float(source_width),
            float(self.input_height) / float(source_height),
        )
        resized_width = max(1, int(round(source_width * ratio)))
        resized_height = max(1, int(round(source_height * ratio)))
        resized = cv2.resize(
            image, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR
        )
        # YuNet accepts raw BGR 0-255. Top-left letterboxing preserves the
        # person crop's aspect ratio; directly stretching a wide upper-body
        # crop to a square makes faces unnaturally narrow and loses detections.
        canvas = numpy.zeros(
            (self.input_height, self.input_width, 3), dtype=numpy.uint8
        )
        canvas[:resized_height, :resized_width] = resized
        tensor = numpy.ascontiguousarray(
            canvas.transpose(2, 0, 1)[None, :, :, :], dtype=numpy.float32
        )
        return tensor, ratio

    def infer(self, image):
        tensor, ratio = self.preprocess(image)
        candidates = self.decode(self.backend.infer(tensor))
        mapped = []
        for x1, y1, x2, y2, score, landmarks in candidates:
            mapped.append(
                (
                    x1 / ratio,
                    y1 / ratio,
                    x2 / ratio,
                    y2 / ratio,
                    score,
                    tuple((x / ratio, y / ratio) for x, y in landmarks),
                )
            )
        return mapped

    def decode(self, outputs):
        """Return x1,y1,x2,y2,score,landmarks in model-input pixels."""
        import numpy

        candidates = []
        for stride in self.STRIDES:
            cls = numpy.asarray(outputs["cls_{0}".format(stride)]).reshape(-1)
            obj = numpy.asarray(outputs["obj_{0}".format(stride)]).reshape(-1)
            bbox = numpy.asarray(outputs["bbox_{0}".format(stride)]).reshape(-1, 4)
            kps = numpy.asarray(outputs["kps_{0}".format(stride)]).reshape(-1, 10)
            scores = numpy.sqrt(numpy.clip(cls, 0.0, 1.0) * numpy.clip(obj, 0.0, 1.0))
            columns = self.input_width // stride
            for index in numpy.flatnonzero(scores >= self.confidence_threshold):
                row = int(index) // columns
                column = int(index) % columns
                values = bbox[index].astype(numpy.float32)
                center_x = (column + float(values[0])) * stride
                center_y = (row + float(values[1])) * stride
                width = float(numpy.exp(numpy.clip(values[2], -10.0, 10.0))) * stride
                height = float(numpy.exp(numpy.clip(values[3], -10.0, 10.0))) * stride
                points = []
                landmark_values = kps[index]
                for point in range(5):
                    points.append(
                        (
                            (column + float(landmark_values[point * 2])) * stride,
                            (row + float(landmark_values[point * 2 + 1])) * stride,
                        )
                    )
                candidates.append(
                    (
                        center_x - width / 2.0,
                        center_y - height / 2.0,
                        center_x + width / 2.0,
                        center_y + height / 2.0,
                        float(scores[index]),
                        tuple(points),
                    )
                )
        return candidates

    def close(self):
        self.backend.close()
