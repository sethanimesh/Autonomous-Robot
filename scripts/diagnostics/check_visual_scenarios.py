#!/usr/bin/env python3
"""Check production decisions with synthetic inputs bound to retained images.

Image bytes establish provenance only. This harness does not infer masks, depth,
identity, hazards, or VLM answers from the pixels, and never executes movement.
"""

import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import sys


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_DIRECTORY = ROOT / "evaluation" / "visual-scenarios"
EVIDENCE_KIND = "synthetic_policy_inputs_on_retained_images"


def prohibit_live_operations(event, args):
    """Prevent accidental network, subprocess, and device access during checks."""
    if event in {
        "socket.connect", "socket.bind", "socket.getaddrinfo", "socket.sendto",
        "subprocess.Popen", "os.system", "os.exec", "os.posix_spawn",
    }:
        raise RuntimeError("Offline visual scenarios prohibit live operation: " + event)
    if event == "open" and isinstance(args[0], (str, bytes, os.PathLike)):
        path = Path(os.path.realpath(os.fsdecode(args[0])))
        if any(path == prefix or prefix in path.parents for prefix in (Path("/dev"), Path("/sys"))):
            raise RuntimeError("Offline visual scenarios prohibit device access: " + str(path))


def read_json(path):
    def reject_constant(value):
        raise ValueError("Non-finite JSON value: " + value)

    return json.loads(path.read_text(encoding="utf-8"), parse_constant=reject_constant)


def checked_images(manifest):
    if manifest.get("schema_version") != 1 or not isinstance(manifest.get("images"), list):
        raise ValueError("Image manifest requires schema_version 1 and an images list")
    images = {}
    for image in manifest["images"]:
        image_id = image.get("id")
        if not isinstance(image_id, str) or not image_id or image_id in images:
            raise ValueError("Image IDs must be unique, nonempty strings")
        relative = image.get("path")
        if not isinstance(relative, str) or Path(relative).is_absolute():
            raise ValueError("Image paths must be relative to the repository")
        path = (ROOT / relative).resolve()
        if ROOT not in path.parents or not path.is_file():
            raise ValueError("Retained image is missing or outside the repository: " + relative)
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        if digest != image.get("sha256"):
            raise ValueError("Retained image SHA-256 mismatch: " + image_id)
        images[image_id] = {"id": image_id, "path": relative, "sha256": digest}
    if not images:
        raise ValueError("Image manifest contains no retained images")
    return images


def policy_functions():
    """Import only policy modules; ROS, services, and model execution stay idle."""
    from robot.jetson.navigation.navigation_reasoning import fuse_route_evidence, occlusion_recovery
    from robot.jetson.navigation.closed_loop_detour import DetourError, validate_route_reasoning
    from robot.jetson.mission.search_view_policy import combine_framing_plans, upward_budget_reached
    from robot.jetson.mission.person_approach import approach_decision
    from robot.jetson.perception.speech_delivery import delivery_gate

    return dict(
        route_fusion=fuse_route_evidence,
        route_binding=validate_route_reasoning,
        route_error=DetourError,
        camera_framing=combine_framing_plans,
        camera_budget=upward_budget_reached,
        occlusion_recovery=occlusion_recovery,
        person_approach=approach_decision,
        delivery_gate=delivery_gate,
    )


def run_policy(policy, value, functions, images):
    if policy == "route_fusion":
        return functions[policy](value["initial_local"], value["fresh_local"], value["vlm_advice"])
    if policy == "route_binding":
        source = images[value["source_image_id"]]
        fresh = images[value["fresh_image_id"]]
        binding = copy.deepcopy(value["route_result"])
        binding["frame_sha256"] = fresh["sha256"]
        reasoning = binding["route_reasoning"]
        reasoning["source_frame_sha256"] = source["sha256"]
        reasoning["fresh_frame_sha256"] = fresh["sha256"]
        if value.get("corrupt_fresh_binding"):
            # Deliberately invalid fixture hash; never presented as an image hash.
            reasoning["fresh_frame_sha256"] = "0" * 64
        try:
            functions[policy](binding)
        except functions["route_error"] as exc:
            return dict(accepted=False, reason=str(exc), checked_result=binding)
        return dict(accepted=True, checked_result=binding)
    if policy == "camera_framing":
        return functions[policy](value["cloud_plan"], value["local_plan"],
                                 value["position"], face_visible=value["face_visible"])
    if policy == "camera_budget":
        return dict(budget_reached=functions[policy](value["origin"], value["current"],
                                                    face_visible=value["face_visible"]))
    if policy == "occlusion_recovery":
        return functions[policy](value["vlm_advice"], value["previous_position"],
                                 value["current_position"], memory_age=value["memory_age"],
                                 same_reference=value["same_reference"],
                                 face_visible=value.get("face_visible", False),
                                 attempts=value.get("attempts", 0))
    if policy == "person_approach":
        return functions[policy](value["observation"])
    if policy == "delivery_gate":
        reason = functions[policy](**value)
        return dict(permitted=reason is None, reason=reason)
    raise ValueError("Unknown production policy: " + str(policy))


def matches_expected(actual, expected):
    """Compare declared outcome fields; retain the complete actual policy result."""
    if isinstance(expected, dict):
        return isinstance(actual, dict) and all(
            key in actual and matches_expected(actual[key], item) for key, item in expected.items())
    if isinstance(expected, list):
        return isinstance(actual, list) and len(actual) == len(expected) and all(
            matches_expected(item, reference) for item, reference in zip(actual, expected))
    if type(expected) in (int, float) and type(actual) in (int, float):
        return math.isclose(actual, expected, rel_tol=1e-9, abs_tol=1e-9)
    return type(actual) is type(expected) and actual == expected


def check_scenarios(manifest, fixtures):
    images = checked_images(manifest)
    if (fixtures.get("schema_version") != 1 or fixtures.get("evidence_kind") != EVIDENCE_KIND
            or not isinstance(fixtures.get("scenarios"), list) or not fixtures["scenarios"]):
        raise ValueError("Scenario fixtures require the declared schema and evidence kind")
    functions = policy_functions()
    results, seen = [], set()
    for scenario in fixtures["scenarios"]:
        scenario_id = scenario.get("id")
        if not isinstance(scenario_id, str) or not scenario_id or scenario_id in seen:
            raise ValueError("Scenario IDs must be unique, nonempty strings")
        seen.add(scenario_id)
        image = images[scenario["image_id"]]
        inputs, expected = scenario["synthetic_inputs"], scenario["expected"]
        if not isinstance(inputs, dict) or not isinstance(expected, dict) or not expected:
            raise ValueError("Each scenario needs synthetic inputs and explicit expected outcome fields")
        try:
            actual = run_policy(scenario["policy"], copy.deepcopy(inputs), functions, images)
            passed = matches_expected(actual, expected)
        except Exception as exc:
            actual, passed = dict(error_type=type(exc).__name__, error=str(exc)), False
        results.append(dict(
            id=scenario_id, image=image, feature=scenario["feature"], policy=scenario["policy"],
            policy_source=scenario["policy_source"], fixture_purpose=scenario["fixture_purpose"], synthetic_inputs=inputs,
            expected=expected, actual=actual, passed=passed,
        ))
    return dict(
        schema_version=1, evidence_kind=EVIDENCE_KIND,
        interpretation="Conditional production-policy outcomes for authored fixtures; image bytes are verified but not interpreted.",
        model_inference_run=False, cloud_called=False, hardware_used=False, movement_executed=False,
        python_version=platform.python_version(),
        scenario_count=len(results), passed_count=sum(item["passed"] for item in results),
        ok=all(item["passed"] for item in results), scenarios=results,
    )


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_DIRECTORY / "manifest.json")
    parser.add_argument("--scenarios", type=Path, default=DEFAULT_DIRECTORY / "scenario_checks.json")
    parser.add_argument("--report", type=Path, help="save complete fixture inputs and observed policy decisions")
    args = parser.parse_args(argv)
    if sys.version_info < (3, 11):
        parser.error("Python 3.11 or newer is required; third-party dependencies are unnecessary")
    sys.dont_write_bytecode = True
    sys.path.insert(0, str(ROOT))
    sys.addaudithook(prohibit_live_operations)
    try:
        result = check_scenarios(read_json(args.manifest), read_json(args.scenarios))
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print("Visual scenario check failed: " + str(exc), file=sys.stderr)
        return 1
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    for item in result["scenarios"]:
        if not item["passed"]:
            print("FAILED {}: expected {}; actual {}".format(item["id"], item["expected"], item["actual"]), file=sys.stderr)
    print("{}/{} conditional visual policy scenarios passed; images verified; no model inference or movement.".format(
        result["passed_count"], result["scenario_count"]))
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
