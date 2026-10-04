#!/usr/bin/env python3
"""Compare face detection on captured frames; no ROS, motors, or image writes.

Optional SCRFD uses the official InsightFace model_zoo/scrfd.py supplied by
--scrfd-code. Review that file before use. Scores are detector confidence,
not identity confidence; this small replay is not a recognition benchmark.
"""

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import time

import cv2
import numpy as np


def variants(image):
    yield "raw", image
    yield "gamma_0.5", np.uint8(np.sqrt(image.astype(np.float32) / 255) * 255)
    ycc = cv2.cvtColor(image, cv2.COLOR_BGR2YCrCb)
    ycc[:, :, 0] = cv2.createCLAHE(2.0, (8, 8)).apply(ycc[:, :, 0])
    yield "clahe", cv2.cvtColor(ycc, cv2.COLOR_YCrCb2BGR)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("images", nargs="+")
    parser.add_argument("--yunet", required=True)
    parser.add_argument("--scrfd")
    parser.add_argument("--scrfd-code")
    parser.add_argument("--report", required=True)
    args = parser.parse_args()
    if bool(args.scrfd) != bool(args.scrfd_code):
        parser.error("supply both --scrfd and --scrfd-code")
    detector = cv2.FaceDetectorYN_create(args.yunet, "", (640, 480), .5, .3, 5000)
    scrfd = None
    models = {"yunet_sha256": digest(args.yunet)}
    if args.scrfd:
        import onnxruntime as ort
        spec = importlib.util.spec_from_file_location("scrfd_eval", args.scrfd_code)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        options = ort.SessionOptions()
        options.intra_op_num_threads = 4
        session = ort.InferenceSession(args.scrfd, options, providers=["CPUExecutionProvider"])
        scrfd = module.SCRFD(args.scrfd, session=session)
        scrfd.prepare(-1, input_size=(640, 640), det_thresh=.5)
        models.update(scrfd_sha256=digest(args.scrfd), scrfd_code_sha256=digest(args.scrfd_code))
    results = []
    for filename in args.images:
        original = cv2.imread(filename)
        if original is None:
            raise ValueError("Cannot read " + filename)
        for variant, frame in variants(original):
            detector.setInputSize((frame.shape[1], frame.shape[0]))
            for name in ("yunet", "scrfd") if scrfd else ("yunet",):
                started = time.perf_counter()
                if name == "yunet":
                    _, faces = detector.detect(frame)
                    detections = [] if faces is None else [
                        {"box_xyxy": [float(f[0]), float(f[1]), float(f[0]+f[2]), float(f[1]+f[3])],
                         "score": float(f[-1])} for f in faces]
                else:
                    faces, _ = scrfd.detect(frame)
                    detections = [{"box_xyxy": f[:4].tolist(), "score": float(f[4])} for f in faces]
                results.append({"image": Path(filename).name, "image_sha256": digest(filename),
                                "variant": variant, "detector": name,
                                "elapsed_ms": (time.perf_counter()-started)*1000,
                                "faces": detections})
    report = {"scope": "Small captured-image detection replay; not identity or live latency validation",
              "threshold": .5, "opencv": cv2.__version__, "models": models, "results": results}
    Path(args.report).write_text(json.dumps(report, indent=2) + "\n")
    for result in results:
        print(result["image"], result["variant"], result["detector"],
              [round(f["score"], 3) for f in result["faces"]])


if __name__ == "__main__":
    main()
