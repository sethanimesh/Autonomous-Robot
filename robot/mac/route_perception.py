#!/usr/bin/env python3
"""Serve conservative local-route decisions from live Jetson camera frames."""

import argparse
import base64
from dataclasses import asdict
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
import math
from pathlib import Path
import time
import threading
from urllib.request import Request, urlopen
from robot.cloud_models import GEMINI_MODEL, GEMINI_THINKING_LEVEL

from robot.jetson.navigation.image_corridors import estimate_floor_horizon
from robot.jetson.navigation.image_corridors import semantic_route_candidates
from robot.jetson.navigation.local_planner import LocalRoutePlanner
from robot.jetson.navigation.navigation_reasoning import fuse_route_evidence


DEFAULT_SNAPSHOT_URL = "http://192.168.1.48:8080/snapshot.jpg"
DEFAULT_CAMERA_STATUS_URL = "http://192.168.1.48:8080/api/status"
DEFAULT_MODEL = "nvidia/segformer-b0-finetuned-ade-512-512"


def validate_floor_label_ids(id2label, floor_ids):
    """Require named support surfaces; ADE20K water is not traversable floor."""
    if not floor_ids or len(set(floor_ids)) != len(floor_ids):
        raise ValueError("Floor label IDs must be nonempty and unique")
    for label_id in floor_ids:
        label = id2label.get(label_id, id2label.get(str(label_id)))
        if label not in ("floor", "rug"):
            raise ValueError("Configured floor label {0} maps to {1!r}, not floor/rug".format(label_id, label))


def same_head_pose(first, second, tolerance=0):
    try:
        return (bool(first.get('reference_id')) and first['reference_id']==second.get('reference_id')
                and abs(int(first['position'])-int(second['position'])) <= tolerance)
    except (KeyError,ValueError,TypeError):
        return False


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
        floor_ids=(3, 28),
    ):
        self.model_name = model_name
        self.device = device
        self.floor_ids = tuple(int(value) for value in floor_ids)
        self.processor = None
        self.model = None
        self.torch = None
        self.numpy = None
        self.image_type = None
        self.planner = LocalRoutePlanner(maximum_step_m=0.05)
        self.minimum_pixel_confidence = 0.65

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
        validate_floor_label_ids(self.model.config.id2label, self.floor_ids)
        self.model.eval()
        self.torch = torch
        self.numpy = np
        self.image_type = Image

    def infer(self, jpeg_payload, return_masks=False):
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
            confidence, labels = logits.softmax(dim=1).max(dim=1)
            labels = labels[0].cpu().numpy()
            confidence = confidence[0].cpu().numpy()
        inference_ms = (time.perf_counter() - started) * 1000.0
        floor_mask = self.numpy.isin(labels, self.floor_ids)
        known = confidence >= self.minimum_pixel_confidence
        if return_masks:
            person_ids = [int(k) for k,v in self.model.config.id2label.items() if v == "person"]
            return {"person": self.numpy.isin(labels, person_ids) & known, "floor": floor_mask & known}
        scene = {
            "floor_fraction": float(self.numpy.mean(floor_mask & known)),
            "ceiling_fraction": float(self.numpy.mean((labels == 5) & known)),
            "known_fraction": float(self.numpy.mean(known)),
            "minimum_pixel_confidence": self.minimum_pixel_confidence,
            "bottom_floor_fraction": float(self.numpy.mean((floor_mask & known)[3 * image.height // 4:, :])),
            "bottom_known_fraction": float(self.numpy.mean(known[3 * image.height // 4:, :])),
        }
        # An obstacle can cover the center while floor remains visible beside it.
        # These describe the camera view, never route clearance.
        for side, columns in (("left", slice(0, image.width // 3)),
                              ("right", slice(2 * image.width // 3, image.width))):
            scene["bottom_" + side + "_floor_fraction"] = float(self.numpy.mean((floor_mask & known)[3 * image.height // 4:, columns]))
            scene["bottom_" + side + "_known_fraction"] = float(self.numpy.mean(known[3 * image.height // 4:, columns]))
        # Preserve uncertainty instead of reporting every argmax pixel as known.
        mask = floor_mask.astype(object)
        mask[~known] = None
        mask = mask.tolist()
        floor_horizon = estimate_floor_horizon(mask)
        candidates, evidence = semantic_route_candidates(
            mask, stride=2
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
            "scene": scene,
            "evidence": [asdict(value) for value in evidence],
            "candidates": [asdict(value) for value in candidates],
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
        from robot.mac.cloud_view_calibration import CloudViewCalibration
        self.cloud_views = CloudViewCalibration()
        from robot.mac.navigation_advisor import GeminiNavigationAdvisor
        self.navigation_advisor = GeminiNavigationAdvisor()
        self.route_lock = threading.Lock()
        self.advisor_lock = threading.Lock()

    def route(self, include_view_reference=False):
        if include_view_reference:
            return self._local_route(include_view_reference=True)
        # Do not queue a stopped robot behind another long cloud request.
        if not self.route_lock.acquire(blocking=False):
            raise RuntimeError('A route observation is already in progress')
        try:
            first, payload = self._local_route(return_payload=True)
            started = time.monotonic()
            advice = self.navigation_advisor.route(payload, first)
            if (advice.get('provider') != 'gemini' or advice.get('advisory_only') is not True
                    or advice.get('frame_sha256') != [first['frame_sha256']]):
                raise RuntimeError('Gemini interpretation does not match the observed route')
            # Cloud latency never refreshes the timestamp of an old photograph.
            # Capture and segment again after it finishes, while still stopped.
            fresh, fresh_payload = self._local_route(return_payload=True)
            if (time.monotonic() - started > 15
                    or not same_head_pose(first['camera_head'], fresh['camera_head'], 3)):
                raise RuntimeError('Route viewpoint changed or Gemini interpretation expired')
            from robot.mac.navigation_advisor import route_frame_change
            change = route_frame_change(payload, fresh_payload)
            if not change['stable']:
                raise RuntimeError('Route image changed, froze, or became dark during Gemini analysis; observe again')
            merged = fuse_route_evidence(first, fresh, advice.get('interpretation'), self.engine.planner)
            fresh['decision'] = merged.pop('decision')
            fresh['route_reasoning'] = dict(merged, provider='gemini', advisory_only=True,
                source_frame_sha256=first['frame_sha256'], fresh_frame_sha256=fresh['frame_sha256'],
                model=advice.get('model'), interpretation=advice.get('interpretation'),
                elapsed_seconds=advice.get('elapsed_seconds'), scene_recheck=change)
            # Include all local post-capture work in the existing freshness gate.
            fresh['result_age_seconds'] = max(fresh['result_age_seconds'],
                time.time() - fresh['camera_received_at_unix'])
            return fresh
        finally:
            self.route_lock.release()

    def _local_route(self, include_view_reference=False, return_payload=False):
        before = fetch_camera_status(self.camera_status_url)
        captured_at = time.time()
        payload = fetch_snapshot(self.snapshot_url)
        camera_status = fetch_camera_status(self.camera_status_url)
        head = camera_status.get("camera_head", {})
        previous = before.get("camera_head", {})
        if (
            not head.get("available")
            or head.get("moving") or head.get("homing")
            or previous.get("moving") or previous.get("homing")
            or not same_head_pose(head,previous,3)
        ):
            raise RuntimeError("camera head changed while capturing scene evidence")
        result = self.engine.infer(payload)
        if include_view_reference:
            self.cloud_views.remember(payload, head)
            result["cloud_view_reference"] = self.cloud_views.match(payload, head)
            reference_path = Path(__file__).resolve().parents[2] / "docs/calibration/view_references/overhead_confirmed.jpg"
            if reference_path.exists():
                from robot.mac.view_reference import match_view_reference
                result["view_reference"] = dict(match_view_reference(payload, reference_path.read_bytes()),
                                                role="overhead", label_source="operator", scope="prepared_room")
        final_head = fetch_camera_status(self.camera_status_url).get("camera_head", {})
        if (final_head.get("moving") or final_head.get("homing")
                or not same_head_pose(final_head,head,3)):
            raise RuntimeError("camera head changed during inference")
        result["camera_received_at_unix"] = captured_at
        result["result_age_seconds"] = max(0.0, time.time() - captured_at)
        result["camera_head"] = camera_status.get("camera_head", {})
        return (result, payload) if return_payload else result

    def occlusion_advice(self, request):
        if (set(request) != {'request_id', 'frames_base64', 'view_changed'}
                or type(request['view_changed']) is not bool
                or not isinstance(request['frames_base64'], list) or len(request['frames_base64']) != 2):
            raise ValueError('Occlusion request needs two ordered JPEGs and measured view-change evidence')
        request_id = request['request_id']
        if (not isinstance(request_id, str) or len(request_id) != 32
                or any(c not in '0123456789abcdef' for c in request_id)):
            raise ValueError('Invalid occlusion request ID')
        frames = []
        for encoded in request['frames_base64']:
            try:
                frame = base64.b64decode(encoded, validate=True)
            except (ValueError, TypeError):
                raise ValueError('Invalid occlusion JPEG encoding') from None
            if not 100 <= len(frame) <= 1_000_000 or not frame.startswith(b'\xff\xd8'):
                raise ValueError('Occlusion request requires supported JPEGs')
            frames.append(frame)
        result = self.navigation_advisor.occlusion(frames, request['view_changed'])
        return dict(result, request_id=request_id)

    def scene_advice(self, request):
        before = fetch_camera_status(self.camera_status_url).get('camera_head', {})
        if (not before.get('available') or not before.get('homed') or before.get('manual_override')
                or before.get('moving') or before.get('homing')
                or before.get('reference_id') != before.get('approved_reference_id')
                or before.get('reference_id') != request.get('reference_id')):
            raise ValueError('Stop the camera at the current sweep reference before cloud analysis')
        result = self.cloud_views.analyze(request, current_position=before['position'])
        after = fetch_camera_status(self.camera_status_url).get('camera_head', {})
        if (not after.get('available') or not after.get('homed') or after.get('manual_override')
                or after.get('moving') or after.get('homing')
                or after.get('approved_reference_id') != before.get('approved_reference_id')
                or not same_head_pose(after,before,3)):
            raise ValueError('Camera changed during cloud analysis; advice was not accepted')
        self.cloud_views.accept(result)
        result.pop('reference_payloads',None)
        return result

    def wardrobe_advice(self, request):
        from robot.mac.wardrobe_advisor import WardrobeAdvisor
        with self.advisor_lock:
            if not hasattr(self, 'wardrobe_advisor'):
                self.wardrobe_advisor = WardrobeAdvisor()
            advisor = self.wardrobe_advisor
        return advisor.interpret(request)

    def person_range(self, request):
        from robot.mac.person_range import PersonRangeService
        with self.advisor_lock:
            if not hasattr(self, 'person_range_service'):
                self.person_range_service = PersonRangeService(self.engine)
            service = self.person_range_service
        if not self.route_lock.acquire(blocking=False):
            raise ValueError("Route perception busy; retry range on the next observation")
        try:
            return service.estimate(request)
        finally:
            self.route_lock.release()

    def setup_advice(self, request, search=False):
        """Assess a fresh frame supplied by the opt-in Jetson setup runner.

        Encoder binding and movement remain on the Jetson. This endpoint never
        fetches extra home images, persists images, or changes motor limits.
        """
        from robot.mac.camera_setup_advisor import setup_view_decision
        from robot.mac.camera_setup_pool import CameraSetupPool
        if set(request) != {'request_id', 'jpeg_base64'}:
            raise ValueError('Send a camera-setup request ID and one JPEG')
        request_id = request['request_id']
        if (not isinstance(request_id, str) or len(request_id) != 32
                or any(c not in '0123456789abcdef' for c in request_id)):
            raise ValueError('Invalid camera-setup request ID')
        try:
            payload = base64.b64decode(request['jpeg_base64'], validate=True)
        except (ValueError, TypeError):
            raise ValueError('Invalid camera-setup JPEG encoding') from None
        if not 100 <= len(payload) <= 1_000_000 or not payload.startswith(b'\xff\xd8'):
            raise ValueError('Camera-setup requires one supported JPEG')
        pool_name = 'search_pool' if search else 'setup_pool'
        if getattr(self, pool_name, None) is None:
            setattr(self, pool_name, CameraSetupPool(search=True) if search else CameraSetupPool())
        result = getattr(self, pool_name).interpret([payload])
        result['request_id'] = request_id
        if not search:
            result['decisions'] = {goal: setup_view_decision(result['observations'][0], goal)
                                   for goal in ('lower', 'upper')}
        return result

    def health(self):
        return {
            "ok": True,
            "scene_advisor": "gemini/" + GEMINI_MODEL,
            "gemini_model": GEMINI_MODEL,
            "gemini_thinking_level": GEMINI_THINKING_LEVEL,
            "camera_setup_model": GEMINI_MODEL,
            "person_search_model": GEMINI_MODEL,
            "navigation_model": GEMINI_MODEL,
            "navigation_advisor": "gemini_adc",
            "scene_advisor_quota": self.cloud_views.last_quota,
            "model_loaded": self.engine.model is not None,
            "wardrobe_protocol": 1,
            "wardrobe_model": GEMINI_MODEL,
            "person_range_protocol": 1,
            "person_range_model_loaded": bool(getattr(getattr(getattr(self,'person_range_service',None),'backend',None),'model',None) is not None),
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

        def do_POST(self):
            from robot.mac.camera_setup_pool import CameraCloudError
            from robot.mac.vision_scene_advisor import GroqAccessError
            if self.path not in ('/scene-advice', '/camera-setup', '/search-view', '/occlusion-advice', '/wardrobe', '/person-range'):
                self.send_json(404, {'ok':False,'error':'not found'});return
            try:
                size=int(self.headers.get('Content-Length','0'))
                limit = (4_000_000 if self.path in ('/wardrobe','/person-range') else 2_800_000 if self.path == '/occlusion-advice' else
                         1_400_000 if self.path in ('/camera-setup', '/search-view') else 4096)
                if not 0 < size <= limit:
                    raise ValueError('Scene request is missing or oversized')
                request=json.loads(self.rfile.read(size))
                if not isinstance(request,dict):
                    raise ValueError('Scene request must be an object')
                result = (service.wardrobe_advice(request) if self.path == '/wardrobe' else service.person_range(request) if self.path == '/person-range' else service.occlusion_advice(request) if self.path == '/occlusion-advice' else
                          service.setup_advice(request, search=self.path == '/search-view')
                          if self.path in ('/camera-setup', '/search-view') else self.service_advice(request))
                self.send_json(200,result)
            except (GroqAccessError, CameraCloudError) as exc:
                self.send_json(exc.status,{'ok':False,'error':str(exc),'quota':exc.quota})
            except ValueError as exc:
                self.send_json(400,{'ok':False,'error':str(exc)})
            except Exception as exc:
                self.send_json(503,{'ok':False,'error':str(exc)})

        def service_advice(self, request):
            return service.scene_advice(request)

        def do_GET(self):
            try:
                if self.path.split("?", 1)[0] == "/health":
                    self.send_json(200, service.health())
                elif self.path.split("?", 1)[0] in ("/route", "/view"):
                    self.send_json(200, service.route(include_view_reference=self.path.split("?", 1)[0] == "/view"))
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
