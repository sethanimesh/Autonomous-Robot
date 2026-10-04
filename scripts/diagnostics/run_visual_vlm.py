#!/usr/bin/env python3
"""Run bounded Gemini evaluations on retained images, without robot endpoints.

Cloud use requires --allow-cloud. Responses are genuine model observations;
saved-image pair ordering and wardrobe request bindings are evaluation fixtures.
Existing results are reused only when their model, prompts, schemas, image hashes
and supplied context match. Use --overwrite to rerun selected tasks explicitly.
"""

import argparse
import base64
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import sys
import time


ROOT = Path(__file__).resolve().parents[2]
DIRECTORY = ROOT / "evaluation" / "visual-scenarios"
MODES = ("framing", "route", "occlusion", "wardrobe")
ROUTE_IMAGES = ("floor-hazards", "clear-floor", "chair-caster-edge", "slipper-obstruction")
OCCLUSION_PAIRS = (
    ("seated-recipient", "partial-body"),
    ("camera-forward-overhead", "camera-lowered-floor"),
)
WARDROBE_IMAGES = ("partial-body",)
EVIDENCE_KIND = "actual_vlm_inference_on_retained_images"


def read_json(path):
    def reject_nonfinite(value):
        raise ValueError("Non-finite JSON value")

    return json.loads(path.read_text(encoding="utf-8"), parse_constant=reject_nonfinite)


def digest(value):
    return hashlib.sha256(value).hexdigest()


def json_digest(value):
    return digest(json.dumps(value, sort_keys=True, separators=(",", ":"),
                             allow_nan=False).encode("utf-8"))


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def checked_images(manifest):
    if (not isinstance(manifest, dict) or manifest.get("schema_version") != 1
            or not isinstance(manifest.get("images"), list)):
        raise ValueError("Manifest requires schema_version 1 and an images list")
    images = {}
    for image in manifest["images"]:
        if not isinstance(image, dict):
            raise ValueError("Manifest image entries must be objects")
        image_id, relative = image.get("id"), image.get("path")
        if (not isinstance(image_id, str) or not image_id or image_id in images
                or not isinstance(relative, str) or Path(relative).is_absolute()):
            raise ValueError("Manifest needs unique image IDs and repository-relative paths")
        path = (ROOT / relative).resolve()
        if ROOT not in path.parents or not path.is_file():
            raise ValueError("Image is missing or outside the repository: " + image_id)
        payload = path.read_bytes()
        if not 100 <= len(payload) <= 4_000_000 or not payload.startswith(b"\xff\xd8"):
            raise ValueError("Image must be a retained JPEG within the adapter size limit: " + image_id)
        sha256 = digest(payload)
        if sha256 != image.get("sha256"):
            raise ValueError("Image SHA-256 mismatch: " + image_id)
        images[image_id] = dict(id=image_id, path=relative, sha256=sha256, payload=payload)
    if not images:
        raise ValueError("Manifest has no images")
    return images


def checked_routes(path, images):
    local = read_json(path)
    if (not isinstance(local, dict) or local.get("schema_version") != 1
            or not isinstance(local.get("images"), list)):
        raise ValueError("Local results require schema_version 1 and an images list")
    routes = {}
    for row in local["images"]:
        if not isinstance(row, dict):
            raise ValueError("Local image entries must be objects")
        image_id = row.get("id")
        if image_id not in ROUTE_IMAGES:
            continue
        if image_id in routes or row.get("sha256") != images[image_id]["sha256"]:
            raise ValueError("Local route source image binding changed: " + str(image_id))
        route = row.get("route")
        if (not isinstance(route, dict) or not isinstance(route.get("evidence"), list)
                or len(route["evidence"]) != 3
                or any(not isinstance(item, dict) for item in route["evidence"])):
            raise ValueError("Local route lacks segmentation evidence: " + image_id)
        # Only the values consumed by the production advisor enter its context.
        # Serialized JSON rejects NaN/Infinity before a request can be submitted.
        horizon = route.get("floor_horizon_y")
        if horizon is not None and (type(horizon) not in (int, float)
                                    or not math.isfinite(horizon) or not 0 <= horizon <= 1):
            raise ValueError("Local route has an invalid floor horizon: " + image_id)
        evidence = []
        for item in route["evidence"]:
            if (type(item.get("heading_degrees")) not in (int, float)
                    or item["heading_degrees"] not in (-30, 0, 30)):
                raise ValueError("Local route has an invalid corridor heading: " + image_id)
            for key in ("floor_fraction", "known_fraction"):
                number = item.get(key)
                if (type(number) not in (int, float) or not math.isfinite(number)
                        or not 0 <= number <= 1):
                    raise ValueError("Local route has an invalid corridor fraction: " + image_id)
            if type(item.get("sample_count")) is not int or item["sample_count"] < 0:
                raise ValueError("Local route has an invalid sample count: " + image_id)
            evidence.append({key: item[key] for key in
                             ("heading_degrees", "floor_fraction", "known_fraction", "sample_count")})
        if {item["heading_degrees"] for item in evidence} != {-30, 0, 30}:
            raise ValueError("Local route has repeated corridor evidence: " + image_id)
        selected = dict(floor_horizon_y=horizon, evidence=evidence)
        json.dumps(selected, allow_nan=False)
        routes[image_id] = selected
    if any(image_id not in routes for image_id in ROUTE_IMAGES):
        raise ValueError("Local results are missing one or more route evaluation images")
    return routes


def task_plan(images, routes, modes, model, framing_batches=None):
    from robot.cloud_models import GEMINI_THINKING_LEVEL
    from robot.mac.person_search_advisor import SEARCH_PROMPT, SEARCH_SCHEMA
    from robot.mac.navigation_advisor import ROUTE_PROMPT, OCCLUSION_PROMPT
    from robot.jetson.navigation.navigation_reasoning import ROUTE_SCHEMA, OCCLUSION_SCHEMA
    from robot.mac.wardrobe_advisor import PROMPT as WARDROBE_PROMPT, SCHEMA as WARDROBE_SCHEMA

    tasks = []

    def append(task_id, mode, ids, prompt, schema, context):
        if any(image_id not in images for image_id in ids):
            raise ValueError("Manifest lacks an image required by task: " + task_id)
        signature = dict(
            model=model, thinking_level=GEMINI_THINKING_LEVEL,
            image_ids=list(ids), frame_sha256=[images[key]["sha256"] for key in ids],
            prompt_sha256=digest(prompt.encode("utf-8")), schema_sha256=json_digest(schema),
            context_sha256=json_digest(context),
        )
        tasks.append(dict(task_id=task_id, mode=mode, image_ids=list(ids), context=context,
                          request_signature=signature, request_sha256=json_digest(signature)))

    if "framing" in modes:
        if framing_batches is None:
            ordered = list(images)
            framing_batches = [dict(task_id="framing-{:02d}".format(start // 3 + 1),
                                    image_ids=ordered[start:start + 3])
                               for start in range(0, len(ordered), 3)]
        if not isinstance(framing_batches, list) or not framing_batches:
            raise ValueError("Framing batches must be a nonempty list")
        seen_tasks, seen_images = set(), set()
        for batch in framing_batches:
            if not isinstance(batch, dict):
                raise ValueError("Framing batches must be objects")
            task_id, ids = batch.get("task_id"), batch.get("image_ids")
            if (not isinstance(task_id, str) or not re.fullmatch(r"framing-[0-9]{2}", task_id)
                    or task_id in seen_tasks or not isinstance(ids, list) or not 1 <= len(ids) <= 3
                    or any(not isinstance(key, str) or key not in images for key in ids)
                    or len(set(ids)) != len(ids) or seen_images.intersection(ids)):
                raise ValueError("Framing batches need unique task and retained-image bindings")
            seen_tasks.add(task_id)
            seen_images.update(ids)
            append(task_id, "framing", ids,
                   SEARCH_PROMPT, SEARCH_SCHEMA,
                   dict(image_order="manifest order; classify each image independently"))
    if "route" in modes:
        for image_id in ROUTE_IMAGES:
            append("route-" + image_id, "route", [image_id], ROUTE_PROMPT, ROUTE_SCHEMA,
                   dict(local_segmentation=routes[image_id],
                        scope="saved-image near-floor image-space candidates"))
    if "occlusion" in modes:
        for previous, current in OCCLUSION_PAIRS:
            append("occlusion-" + previous + "-to-" + current, "occlusion",
                   [previous, current], OCCLUSION_PROMPT, OCCLUSION_SCHEMA,
                   dict(view_changed=True, context_kind="authored_saved_image_pair",
                        timing="Ordering is an evaluation fixture, not a synchronized temporal capture",
                        binding="View-change context does not establish fresh motion-state evidence"))
    if "wardrobe" in modes:
        for image_id in WARDROBE_IMAGES:
            append("wardrobe-" + image_id, "wardrobe", [image_id], WARDROBE_PROMPT, WARDROBE_SCHEMA,
                   dict(operation="describe", references=[], binding_kind="synthetic_evaluation_binding",
                        input_scope="full retained frame; no identity or stored-reference comparison"))
    return tasks


def safe_usage(value):
    """Retain token accounting without arbitrary provider metadata."""
    if not isinstance(value, dict):
        return None
    result = {}
    for key, item in value.items():
        if key.endswith("TokenCount") and type(item) is int and item >= 0:
            result[key] = item
        elif key.endswith("TokensDetails") and isinstance(item, list):
            details = []
            for part in item:
                if (isinstance(part, dict) and part.get("modality") in
                        ("TEXT", "IMAGE", "VIDEO", "AUDIO", "DOCUMENT")
                        and type(part.get("tokenCount")) is int and part["tokenCount"] >= 0):
                    details.append(dict(modality=part["modality"], tokenCount=part["tokenCount"]))
            result[key] = details
    return result


def sanitized_success(task, response, elapsed):
    signature = task["request_signature"]
    if (not isinstance(response, dict) or response.get("provider") != "gemini"
            or response.get("model") != signature["model"]
            or response.get("frame_sha256") != signature["frame_sha256"]):
        raise ValueError("Provider response model or source-image binding changed")
    parsed = response.get("observations") if task["mode"] == "framing" else response.get("interpretation")
    if parsed is None:
        raise ValueError("Provider response has no parsed observation")
    # The production adapters validate the parsed schema. No credentials, ADC
    # project/account fields, headers, raw provider payloads or errors are copied.
    return dict(status="completed", provider="gemini", model=signature["model"],
                frame_sha256=signature["frame_sha256"],
                prompt_sha256=signature["prompt_sha256"],
                elapsed_seconds=round(elapsed, 3), usage=safe_usage(response.get("usage")),
                parsed_response=parsed, advisory_only=True)


def sanitized_failure(exc, elapsed):
    """Classify failure without saving provider/credential exception text."""
    from robot.mac.camera_setup_pool import CameraCloudError

    status, http_status = "unavailable", None
    if isinstance(exc, CameraCloudError):
        http_status = exc.status if type(exc.status) is int else None
        if http_status == 429:
            status = "quota_limited"
        elif http_status in (401, 403):
            status = "authorization_unavailable"
        elif http_status in (400, 404):
            status = "request_or_model_unavailable"
    elif isinstance(exc, (ValueError, KeyError, TypeError, json.JSONDecodeError)):
        status = "invalid_response_or_configuration"
    return dict(status=status, http_status=http_status, elapsed_seconds=round(elapsed, 3),
                usage=None, parsed_response=None,
                error="Request stopped; provider and credential details omitted")


def make_clients(model):
    from robot.mac.camera_setup_pool import GeminiCameraSetupAdvisor, VertexADC
    from robot.mac.navigation_advisor import GeminiNavigationAdvisor
    from robot.mac.person_search_advisor import SEARCH_PROMPT
    from robot.mac.wardrobe_advisor import WardrobeAdvisor

    adc = VertexADC()
    framing = GeminiCameraSetupAdvisor(model=model, adc=adc, timeout=20,
                                      prompt=SEARCH_PROMPT, search=True)
    structured = GeminiCameraSetupAdvisor(model=model, adc=adc, timeout=20)
    return dict(framing=framing, navigation=GeminiNavigationAdvisor(client=structured),
                wardrobe=WardrobeAdvisor(client=structured))


def execute_task(task, images, routes, clients):
    ids = task["image_ids"]
    frames = [images[key]["payload"] for key in ids]
    if task["mode"] == "framing":
        return clients["framing"].interpret(frames)
    if task["mode"] == "route":
        return clients["navigation"].route(frames[0], routes[ids[0]])
    if task["mode"] == "occlusion":
        return clients["navigation"].occlusion(frames, task["context"]["view_changed"])
    if task["mode"] == "wardrobe":
        binding = dict(request_id="saved-image-" + task["task_id"],
                       frame_key="fixture-" + images[ids[0]]["sha256"],
                       track_id="evaluation-track", revision="evaluation-revision-v1",
                       image_sha256=images[ids[0]]["sha256"])
        return clients["wardrobe"].interpret(dict(
            operation="describe", binding=binding, references=[],
            image=base64.b64encode(frames[0]).decode("ascii")))
    raise ValueError("Unknown evaluation task")


def load_cache(path):
    if not path.exists():
        return {}
    report = read_json(path)
    if (not isinstance(report, dict) or report.get("schema_version") != 1
            or report.get("evidence_kind") != EVIDENCE_KIND
            or not isinstance(report.get("tasks"), list)):
        raise ValueError("Existing VLM report has an unsupported schema")
    cache = {}
    for row in report["tasks"]:
        if not isinstance(row, dict):
            raise ValueError("Existing VLM report task entries must be objects")
        task_id = row.get("task_id")
        if not isinstance(task_id, str) or task_id in cache or not row.get("request_sha256"):
            raise ValueError("Existing VLM report has invalid task identifiers")
        cache[task_id] = row
    return cache


def save_report(path, cache, manifest, selected_tasks, stopped_reason=None):
    rows = list(cache.values())
    report = dict(
        schema_version=1, evidence_kind=EVIDENCE_KIND, updated_at_utc=utc_now(),
        interpretation="Actual Gemini observations on saved images; pair ordering and wardrobe bindings are evaluation fixtures",
        manifest_sha256=json_digest(manifest), hardware_used=False, movement_executed=False,
        live_robot_contacted=False, alternate_models_used=False,
        selected_task_ids=[task["task_id"] for task in selected_tasks],
        completed_count=sum(row.get("status") == "completed" for row in rows),
        failed_count=sum(row.get("status") != "completed" for row in rows),
        stopped_reason=stopped_reason, tasks=rows,
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DIRECTORY / "manifest.json")
    parser.add_argument("--local-results", type=Path, default=DIRECTORY / "local-results.json")
    parser.add_argument("--output", "--report", dest="output", type=Path, default=DIRECTORY / "vlm-results.json")
    parser.add_argument("--tasks", default=",".join(MODES), help="comma-separated framing,route,occlusion,wardrobe")
    parser.add_argument("--limit", type=int, help="maximum new cloud requests in this invocation")
    parser.add_argument("--allow-cloud", action="store_true", help="explicitly permit saved-image Gemini requests")
    parser.add_argument("--overwrite", action="store_true", help="explicitly rerun selected cached tasks")
    parser.add_argument("--dry-run", action="store_true", help="verify inputs and list requests without credentials or network")
    parser.add_argument("--model", help="explicit Gemini model; defaults to the repository's configured selection")
    args = parser.parse_args(argv)
    if args.limit is not None and args.limit < 1:
        parser.error("--limit must be positive")
    modes = args.tasks.split(",")
    if not modes or len(set(modes)) != len(modes) or any(mode not in MODES for mode in modes):
        parser.error("--tasks must contain unique supported modes")
    if not args.allow_cloud and not args.dry_run:
        parser.error("Cloud requests require --allow-cloud; use --dry-run for offline input validation")
    sys.dont_write_bytecode = True
    sys.path.insert(0, str(ROOT))
    from robot.cloud_models import GEMINI_MODEL
    model = args.model or GEMINI_MODEL
    if not re.fullmatch(r"gemini-[A-Za-z0-9._-]{1,100}", model):
        parser.error("--model must be a Gemini model identifier")
    try:
        manifest = read_json(args.manifest)
        images = checked_images(manifest)
        routes = checked_routes(args.local_results, images) if "route" in modes else {}
        tasks = task_plan(images, routes, modes, model, manifest.get("framing_batches"))
        cache = load_cache(args.output)
        for task in tasks:
            previous = cache.get(task["task_id"])
            if previous and previous["request_sha256"] != task["request_sha256"] and not args.overwrite:
                raise ValueError("Cached model/source/prompt/context changed for " + task["task_id"] + "; use --overwrite explicitly")
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print("Saved-image evaluation input error: " + str(exc), file=sys.stderr)
        return 1
    pending = [task for task in tasks if args.overwrite or task["task_id"] not in cache]
    limited = pending[:args.limit] if args.limit else pending
    print("{} images verified; {} selected tasks; {} cached; {} requests planned; model {}.".format(
        len(images), len(tasks), len(tasks) - len(pending), len(limited), model))
    if args.dry_run:
        for task in limited:
            print("{}: {}".format(task["task_id"], ", ".join(task["image_ids"])))
        return 0
    if not args.overwrite and any(cache.get(task["task_id"], {}).get("status") not in
                                 (None, "completed") for task in tasks):
        print("A selected task previously stopped. Use --overwrite explicitly to retry it; no requests sent.")
        return 1
    if not limited:
        print("No missing requests. Use --overwrite only when a fresh evaluation is intended.")
        return 0
    clients = None
    for task in limited:
        started = time.monotonic()
        record = dict(task_id=task["task_id"], mode=task["mode"], image_ids=task["image_ids"],
                      input_context=task["context"], request_signature=task["request_signature"],
                      request_sha256=task["request_sha256"], attempted_at_utc=utc_now())
        try:
            if clients is None:
                clients = make_clients(model)
            response = execute_task(task, images, routes, clients)
            record.update(sanitized_success(task, response, time.monotonic() - started))
        except Exception as exc:
            record.update(sanitized_failure(exc, time.monotonic() - started))
            record.update(provider="gemini", model=model,
                          frame_sha256=task["request_signature"]["frame_sha256"],
                          prompt_sha256=task["request_signature"]["prompt_sha256"])
            cache[task["task_id"]] = record
            save_report(args.output, cache, manifest, tasks, stopped_reason=record["status"])
            print(task["task_id"] + ": " + record["status"] + "; remaining requests stopped.")
            return 1
        cache[task["task_id"]] = record
        save_report(args.output, cache, manifest, tasks)
        print(task["task_id"] + ": completed ({:.3f}s).".format(record["elapsed_seconds"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
