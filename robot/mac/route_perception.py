#!/usr/bin/env python3
"""Serve conservative local-route decisions from live Jetson camera frames."""

import argparse
from dataclasses import asdict
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
import math
import time
from urllib.request import Request, urlopen

from robot.jetson.navigation.image_corridors import estimate_floor_horizon
from robot.jetson.navigation.image_corridors import semantic_route_candidates
from robot.jetson.navigation.local_planner import LocalRoutePlanner


DEFAULT_SNAPSHOT_URL = "http://192.168.1.48:8080/snapshot.jpg"
DEFAULT_CAMERA_STATUS_URL = "http://192.168.1.48:8080/api/status"
DEFAULT_MODEL = "nvidia/segformer-b0-finetuned-ade-512-512"


def fetch_snapshot(url, timeout_seconds=2.0):
    request = Request(url, headers={"Cache-Control": "no-cache"})
    with urlopen(request, timeout=timeout_seconds) as response:
        content_type = response.headers.get("Content-Type", "")
        payload = response.read()
    if "image/jpeg" not in content_type.lower():
        raise RuntimeError("camera snapshot did not return JPEG")
    if len(payload) < 1000:
        raise RuntimeError("camera snapshot is unexpectedly small")
    return payload


def fetch_camera_status(url, timeout_seconds=2.0):
    request = Request(url, headers={"Cache-Control": "no-cache"})
    with urlopen(request, timeout=timeout_seconds) as response:
        status = json.loads(response.read().decode("utf-8"))
    if not isinstance(status, dict) or not status.get("camera_ready", False):
        raise RuntimeError("Jetson camera is not currently producing live frames")
    return status


class RoutePerceptionEngine:
    """Lazy-loaded SegFormer floor segmentation plus deterministic planning."""

    def __init__(
        self,
        model_name=DEFAULT_MODEL,
        device="mps",
        floor_ids=(3, 21, 28),
    ):
        self.model_name = model_name
        self.device = device
        self.floor_ids = tuple(int(value) for value in floor_ids)
        self.processor = None
        self.model = None
        self.torch = None
        self.numpy = None
        self.image_type = None
        self.planner = LocalRoutePlanner()

    def load(self):
        if self.model is not None:
            return
        import numpy as np
        from PIL import Image
        import torch
        from transformers import AutoImageProcessor, SegformerForSemanticSegmentation

        if self.device == "mps" and not torch.backends.mps.is_available():
            raise RuntimeError("Apple MPS is unavailable")
        self.processor = AutoImageProcessor.from_pretrained(self.model_name)
        self.model = SegformerForSemanticSegmentation.from_pretrained(
            self.model_name
        ).to(self.device)
        self.model.eval()
        self.torch = torch
        self.numpy = np
        self.image_type = Image

    def infer(self, jpeg_payload):
        self.load()
        image = self.image_type.open(io.BytesIO(jpeg_payload)).convert("RGB")
        started = time.perf_counter()
        values = self.processor(images=image, return_tensors="pt")
        pixels = values["pixel_values"].to(self.device)
        with self.torch.inference_mode():
            logits = self.model(pixel_values=pixels).logits
            logits = self.torch.nn.functional.interpolate(
                logits,
                size=(image.height, image.width),
                mode="bilinear",
                align_corners=False,
            )
            labels = logits.argmax(dim=1)[0].cpu().numpy()
        inference_ms = (time.perf_counter() - started) * 1000.0
        floor_mask = self.numpy.isin(labels, self.floor_ids)
        floor_horizon = estimate_floor_horizon(floor_mask.tolist())
        candidates, evidence = semantic_route_candidates(
            floor_mask.tolist(), stride=2
        )
        decision = self.planner.choose(candidates)
        decision_value = asdict(decision)
        if not math.isfinite(decision_value["score"]):
            decision_value["score"] = None
        return {
            "ok": True,
            "generated_at_unix": time.time(),
            "frame_sha256": hashlib.sha256(jpeg_payload).hexdigest(),
            "image": {"width": image.width, "height": image.height},
            "model": self.model_name,
            "device": self.device,
            "inference_ms": round(inference_ms, 1),
            "floor_label_ids": list(self.floor_ids),
            "floor_horizon_y": floor_horizon,
            "evidence": [asdict(value) for value in evidence],
            "decision": decision_value,
        }

    def warmup(self):
        """Load weights and compile the device path before serving live results."""
        self.load()
        image = self.image_type.new("RGB", (640, 480), (128, 128, 128))
        buffer = io.BytesIO()
        image.save(buffer, format="JPEG")
        self.infer(buffer.getvalue())


class RouteService:
    def __init__(
        self,
        engine,
        snapshot_url=DEFAULT_SNAPSHOT_URL,
        camera_status_url=DEFAULT_CAMERA_STATUS_URL,
    ):
        self.engine = engine
        self.snapshot_url = snapshot_url
        self.camera_status_url = camera_status_url
        self.started_at = time.time()

    def route(self):
        fetch_camera_status(self.camera_status_url)
        payload = fetch_snapshot(self.snapshot_url)
        camera_status = fetch_camera_status(self.camera_status_url)
        captured_at = time.time()
        result = self.engine.infer(payload)
        result["camera_received_at_unix"] = captured_at
        result["result_age_seconds"] = max(0.0, time.time() - captured_at)
        result["camera_head"] = camera_status.get("camera_head", {})
        return result

    def health(self):
        return {
            "ok": True,
            "model_loaded": self.engine.model is not None,
            "snapshot_url": self.snapshot_url,
            "camera_status_url": self.camera_status_url,
            "uptime_seconds": round(time.time() - self.started_at, 1),
        }


def make_handler(service):
    class Handler(BaseHTTPRequestHandler):
        def send_json(self, status, value):
            payload = json.dumps(value, separators=(",", ":")).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(payload)

        def do_GET(self):
            try:
                if self.path.split("?", 1)[0] == "/health":
                    self.send_json(200, service.health())
                elif self.path.split("?", 1)[0] == "/route":
                    self.send_json(200, service.route())
                else:
                    self.send_json(404, {"ok": False, "error": "not found"})
            except Exception as exc:
                self.send_json(503, {"ok": False, "error": str(exc)})

        def log_message(self, message, *args):
            return

    return Handler


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot-url", default=DEFAULT_SNAPSHOT_URL)
    parser.add_argument("--camera-status-url", default=DEFAULT_CAMERA_STATUS_URL)
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--device", default="mps")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8091)
    parser.add_argument("--once", action="store_true")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    service = RouteService(
        RoutePerceptionEngine(args.model, args.device),
        args.snapshot_url,
        args.camera_status_url,
    )
    if args.once:
        print(json.dumps(service.route(), indent=2, sort_keys=True))
        return 0
    service.engine.warmup()
    server = ThreadingHTTPServer((args.host, args.port), make_handler(service))
    print("route perception listening on {0}:{1}".format(args.host, args.port))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
