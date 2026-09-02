#!/usr/bin/env python3
"""Compile a YOLOX ONNX file into a TensorRT engine on the Jetson.

Run this once per model and precision. A TensorRT engine is specific to the
GPU, the driver, and the TensorRT version that built it, so it is always built
on the machine that will run it and is never committed to Git.

    python3 build_engine.py models/yolox_tiny.onnx models/yolox_tiny_fp16.engine --fp16
    python3 build_engine.py --inspect models/yolox_tiny_fp16.engine

The engine is built with detailed profiling verbosity so the precision it
actually runs at can be read back from the built artifact rather than assumed
from the flag we passed. A sidecar ``.json`` records the evidence: requested
and measured precision, the tensor-datatype histogram behind that measurement,
TensorRT version, GPU, and source-ONNX checksum.

TensorRT is imported only when this script runs, so the rest of the perception
package stays importable on a development machine.
"""

import argparse
import collections
import hashlib
import json
import os
import sys
import time

WORKSPACE_BYTES = 1 << 30
SIDECAR_SUFFIX = ".json"
KNOWN_DATATYPES = ("INT8", "FP16", "BF16", "FP32", "INT32")


def sha256_of(path, chunk_size=1 << 20):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(chunk_size), b""):
            digest.update(chunk)
    return digest.hexdigest()


def sidecar_path(engine_path):
    return engine_path + SIDECAR_SUFFIX


def tensor_precisions(engine, trt):
    """Histogram of the datatypes TensorRT actually chose for tensors.

    TensorRT reports precision through each tensor's ``Format/Datatype``
    string rather than as a layer field, so this counts those. It is the only
    way to confirm from the built artifact -- rather than from the build flag
    we passed -- that FP16 kernels are really in use.
    """
    try:
        inspector = engine.create_engine_inspector()
        raw = inspector.get_engine_information(trt.LayerInformationFormat.JSON)
        layers = json.loads(raw).get("Layers", [])
    except Exception:  # pragma: no cover - inspector is advisory
        return None
    if not layers or not isinstance(layers[0], dict):
        return None

    histogram = collections.Counter()
    for layer in layers:
        for side in ("Inputs", "Outputs"):
            for tensor in layer.get(side, []) or []:
                if not isinstance(tensor, dict):
                    continue
                text = str(tensor.get("Format/Datatype", ""))
                for name in KNOWN_DATATYPES:
                    if name in text:
                        histogram[name] += 1
                        break
                else:
                    histogram["OTHER"] += 1
    return dict(histogram)


def measured_precision(histogram):
    """Name the precision the engine actually runs at, or 'unknown'.

    The network input and output stay FP32 even in an FP16 engine, so the
    presence of any reduced-precision tensor is what distinguishes them.
    """
    if not histogram:
        return "unknown"
    if histogram.get("INT8"):
        return "int8"
    if histogram.get("FP16"):
        return "fp16"
    if histogram.get("FP32"):
        return "fp32"
    return "unknown"


def device_name():
    try:
        from cuda import cudart

        err, props = cudart.cudaGetDeviceProperties(0)
        if err == cudart.cudaError_t.cudaSuccess:
            return props.name.decode("utf-8", "replace")
    except Exception:  # pragma: no cover
        pass
    return "unknown"


def describe_engine(engine_path, extra=None):
    """Deserialise an engine and write or refresh its metadata sidecar."""
    import tensorrt as trt

    runtime = trt.Runtime(trt.Logger(trt.Logger.ERROR))
    with open(engine_path, "rb") as handle:
        engine = runtime.deserialize_cuda_engine(handle.read())
    if engine is None:
        raise SystemExit("could not deserialise {0}".format(engine_path))

    histogram = tensor_precisions(engine, trt)
    metadata = {
        "engine": os.path.basename(engine_path),
        "engine_sha256": sha256_of(engine_path),
        "engine_bytes": os.path.getsize(engine_path),
        "tensorrt_version": trt.__version__,
        "device": device_name(),
        "tensor_precisions": histogram,
        "measured_precision": measured_precision(histogram),
    }
    if extra:
        metadata.update(extra)

    existing = {}
    if os.path.isfile(sidecar_path(engine_path)):
        try:
            with open(sidecar_path(engine_path)) as handle:
                existing = json.load(handle)
        except (ValueError, OSError):
            existing = {}
    existing.update(metadata)
    with open(sidecar_path(engine_path), "w") as handle:
        json.dump(existing, handle, indent=2, sort_keys=True)
    return existing


def build(onnx_path, engine_path, fp16, workspace_bytes=WORKSPACE_BYTES):
    import tensorrt as trt

    if not os.path.isfile(onnx_path):
        raise SystemExit("ONNX file not found: {0}".format(onnx_path))

    logger = trt.Logger(trt.Logger.INFO)
    builder = trt.Builder(logger)
    flags = 1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
    network = builder.create_network(flags)
    parser = trt.OnnxParser(network, logger)

    with open(onnx_path, "rb") as handle:
        if not parser.parse(handle.read()):
            for index in range(parser.num_errors):
                print("parse error: {0}".format(parser.get_error(index)), file=sys.stderr)
            raise SystemExit("failed to parse {0}".format(onnx_path))

    config = builder.create_builder_config()
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, workspace_bytes)
    # Needed so the tensor datatypes can be read back from the built engine.
    # Without it the inspector reports only layer names and the precision
    # claim would be unverifiable.
    config.profiling_verbosity = trt.ProfilingVerbosity.DETAILED

    if fp16:
        if not builder.platform_has_fast_fp16:
            raise SystemExit("FP16 requested but this platform reports no fast FP16")
        config.set_flag(trt.BuilderFlag.FP16)

    shapes = {"inputs": [], "outputs": []}
    for index in range(network.num_inputs):
        tensor = network.get_input(index)
        shapes["inputs"].append([tensor.name, list(tensor.shape)])
        print("input  {0}: {1} {2}".format(index, tensor.name, tuple(tensor.shape)))
    for index in range(network.num_outputs):
        tensor = network.get_output(index)
        shapes["outputs"].append([tensor.name, list(tensor.shape)])
        print("output {0}: {1} {2}".format(index, tensor.name, tuple(tensor.shape)))

    print("building {0} engine; this takes several minutes".format("FP16" if fp16 else "FP32"))
    started = time.time()
    serialized = builder.build_serialized_network(network, config)
    if serialized is None:
        raise SystemExit("engine build failed")
    elapsed = time.time() - started

    with open(engine_path, "wb") as handle:
        handle.write(serialized)

    metadata = describe_engine(
        engine_path,
        extra={
            "source_onnx": os.path.basename(onnx_path),
            "source_onnx_sha256": sha256_of(onnx_path),
            "source_onnx_bytes": os.path.getsize(onnx_path),
            "requested_precision": "fp16" if fp16 else "fp32",
            "build_seconds": round(elapsed, 1),
            "shapes": shapes,
        },
    )

    size_mb = os.path.getsize(engine_path) / (1024.0 * 1024.0)
    print("wrote {0} ({1:.1f} MB) in {2:.1f}s".format(engine_path, size_mb, elapsed))
    print("requested precision: {0}".format(metadata["requested_precision"]))
    print("measured precision:  {0}".format(metadata["measured_precision"]))
    print("tensor datatypes:    {0}".format(metadata["tensor_precisions"]))
    print("metadata: {0}".format(sidecar_path(engine_path)))
    return engine_path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--inspect",
        metavar="ENGINE",
        help="refresh the metadata sidecar of an existing engine and exit",
    )
    parser.add_argument("onnx", nargs="?", help="source ONNX file")
    parser.add_argument("engine", nargs="?", help="destination TensorRT engine file")
    parser.add_argument("--fp16", action="store_true", help="enable FP16 kernels")
    parser.add_argument(
        "--workspace-mb", type=int, default=1024, help="builder workspace in MiB"
    )
    args = parser.parse_args(argv)

    if args.inspect:
        print(json.dumps(describe_engine(args.inspect), indent=2, sort_keys=True))
        return
    if not args.onnx or not args.engine:
        parser.error("onnx and engine are required unless --inspect is given")
    build(args.onnx, args.engine, args.fp16, args.workspace_mb * 1024 * 1024)


if __name__ == "__main__":
    main()
