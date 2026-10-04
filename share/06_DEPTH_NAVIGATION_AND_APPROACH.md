# Depth, floor assessment, hazards, and approach

These guides retain the original source investigation and historical results, with a 2026-10-04 revision for the hospital assistance scenario and the current message-delivery code. They distinguish implementation, recorded software checks, physical demonstrations, and unverified assumptions; no live robot was tested for this revision. Refer to the [full dossier](FULL_TECHNICAL_DOSSIER.md) and [snapshot](SOURCE_SNAPSHOT.md) for provenance. Repository links resolve from this folder; cited source line numbers belong to the original investigation unless a current-source review is identified.

**C8. Depth, floor corridors, and hazard assessment**

The production depth model is **`depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf`**, running through Transformers/PyTorch on the Mac, normally using MPS. It is a metric-trained model, but the current physical-distance output is not validated merely by that model designation. See depth backend, lines 246–260 — `robot/mac/person_range.py:246`.

Two range modes exist.

**Measured mode** uses saved head-dependent geometry, scale, and front offset, with a held-out acceptance tool. The tool requires distinct training/validation observations, seated and standing near/far cases, and worst-error gates. I found implemented tooling but no accepted production measured-range dataset in the inspected evidence.

**Approximate mode**, currently enabled by default, estimates floor orientation from model depth and segmentation. It anchors scale at a lower view to a nominal **0.15 m lens height**; other views inherit scale. Front offset is zero, and the reported reference is the camera’s ground projection. Floor-pose caching is bounded by time, reference, intrinsics, and motion state. Results explicitly include `validated=False` and heuristic uncertainty. See automatic pose and approximate result, lines 274–375 — `robot/mac/person_range.py:274`.

Person range uses segmented person pixels associated with a detector box, permits a limited connected extension below a clipped box, and estimates the near visible surface using a low percentile of forward depth. Foot support checks are image/geometry rules; they do not establish measured foot position.

The negative evidence is substantial:

- One operator front-to-foot measurement was **34 inches / 0.8636 m**.
- The system’s camera-origin estimate was **0.3421 m**, despite two consistent samples and `feet_checked=True`.
- Camera-to-front offset was unmeasured, so this is not a clean formal error measurement.
- Later same-pose observations with measured lens height produced **0.321 m** and **0.244 m**, while inferred pitch changed from **20.75° to 25.35°**.
- Floor-plane inlier rates remained about **99.9%**.

Therefore good plane fit and repeated model consistency demonstrably did not establish correct physical geometry. See operator-distance check — `artifacts/mom-combined-right-20260912/operator-distance-check.json:2` and same-position discrepancy and diagnostic alternatives, lines 2581–2593 — `docs/DEVELOPMENT_LOG.md` (local development evidence) (source line 2581).

Floor corridors use **SegFormer-B0 fine-tuned on ADE20K**, running on Mac MPS. Logits are resized back to the input image; pixels with maximum class probability below **0.65** are unknown. Three fixed trapezoids correspond to left, center, and right, nominally −30°, 0°, +30°. Their upper boundary is trimmed to a detected floor horizon.

The route gate requires:

- At least **95% floor among known pixels**.
- At least **75% known pixels**.
- Candidate confidence at least **0.70**.
- Appropriate heading and margin bounds.

A qualifying corridor receives a constant **0.15 m clear-distance value**. This is an image heuristic, not depth-measured free space. The route service caps its resulting primitive at **5 cm**. See segmentation and engine settings, lines 61–140 — `robot/mac/route_perception.py:61`, corridor construction, lines 27–180 — `robot/jetson/navigation/image_corridors.py:27`, and planner gates and scoring, lines 59–152 — `robot/jetson/navigation/local_planner.py:59`.

Two implementation findings materially limit the clearance claim:

1. **Water is included in the floor IDs.** Runtime defaults are `(3,21,28)`. The cached model configuration maps these to floor, **water**, and rug. Confident water pixels can therefore enter the local floor mask and range-plane fitting. This is a verified code/configuration mismatch, not a recorded water encounter. Gemini may separately veto a scene, but that does not correct the local label definition. Evidence: runtime floor IDs — `robot/mac/route_perception.py:61` and cached model labels, lines 29–58 — `external model metadata/models--nvidia--segformer-b0-finetuned-ade-512-512/snapshots/489d5cd81a0b59fab9b7ea758d3548ebe99677da/config.json:29` (external metadata; not included).
2. **The image polygons are not a calibrated footprint projection.** The planner stores 20×25 cm dimensions and a 5 cm margin, but the corridor polygons are not derived from those dimensions, camera pose, or a turn swept volume. A claimed 30 cm-wide corridor is not geometrically proved by this active path.

The floor/known thresholds also allow, mathematically, only `0.95 × 0.75 = 71.25%` of all sampled pixels to be confidently floor. The remaining pixels are not all proved traversable.

Gemini receives the stopped image, the exact candidate polygons, and local segmentation evidence. Its strict schema describes quality, visibility, hazards, turn-space visibility, and route preference. Unknown/occluded/hazardous answers veto candidates; preference only ranks locally admitted routes. It cannot directly choose motor speed or invent a route that local checks rejected.

After Gemini responds, the Mac captures and segments another image, compares scene change, and intersects old/new local evidence with the cloud decision. The Jetson then validates image binding, result age, stopped pose, and references. Malformed or unavailable advice blocks movement. See prompt and image comparison, lines 12–101 — `robot/mac/navigation_advisor.py:12`, schema and deterministic fusion, lines 21–143 — `robot/jetson/navigation/navigation_reasoning.py:21`, and paired route computation, lines 189–255 — `robot/mac/route_perception.py:189`.

These checks assess visible near-floor evidence. They do not establish complete three-dimensional clearance, reliable drop detection, or collision-free turning.

**C9. Approach policy and motor execution**

Current family-mode approach requires current identity, an associated range result, acceptable age, a valid interval, and two consistent estimates.

Approximate thresholds are currently:

| Approximate estimate | Action |
|---|---|
| Below 0.12 m | Stop as already close; no drive. |
| At or below 0.20 m | `arrived_estimate`. |
| 0.20–0.30 m | Request 2 cm. |
| Above 0.30 m | Normally request 5 cm. |
| Uncertain feet near the person | Inspect lower; after a current lower-floor assessment, limit to 2 cm. |

Measured mode has a separate rule: arrival requires the interval to fit within **0.5–0.7 m**, with enough consistent samples and required foot checks. Legacy non-family stopping uses confirmed face-height fraction, currently default **0.28**, rather than a metric gap.

See approach policy, lines 24–81 — `robot/jetson/mission/person_approach.py:24` and legacy/current arrival dispatch, lines 44–85 — `robot/jetson/mission/autonomous_find.py:44`.

The approximate thresholds were lowered at operator request while range error remained unresolved. This is a documented operating choice, not a measured improvement in accuracy. Older `.5–.7 m` approximate descriptions and the unused YAML’s `.90 m` standoff are stale relative to the current policy.

The detour worker:

- Stops before inspection.
- Verifies a calibrated floor head pose.
- Requests a route.
- Turns and stops if necessary.
- Requires a fresh forward route after an actual turn.
- Drives while monitoring odometric progress.
- Stops and obtains feedback.

Defaults are **0.06 m/s** translation, **0.60 rad/s** turn, 5-second drive timeout, 8-second turn timeout, 1 cm distance tolerance, and 3° yaw tolerance. It detects backward motion, excessive yaw, wrong-direction turns, and lack of progress. See motion loops, lines 423–484 — `robot/jetson/navigation/closed_loop_detour.py:423` and primitive execution, lines 505–576 — `robot/jetson/navigation/closed_loop_detour.py:505`.

Distance is encoder-derived. There is no independent translational observation confirming ground travel during each primitive. Tolerance, feedback delay, braking, and track slip mean requested distance is not a hard physical maximum—the 5 cm request producing 6.77 cm encoder travel is a concrete example.

If a drive is interrupted after starting, the parent requires target reacquisition rather than blindly issuing another full-distance request. Persistent identity loss or route issue pauses or terminates the mission. See interrupted-drive handling, lines 338–351 and 442–459 — `robot/jetson/mission/autonomous_find.py:338`.

[Back to reading guide](README.md)
