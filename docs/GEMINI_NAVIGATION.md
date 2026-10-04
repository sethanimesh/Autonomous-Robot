# Gemini routes and occlusion — prepared offline

2026-09-09 deployment: the 67-file bundle is installed and verified on Jetson;
the Mac service is restarted. Jetson-only preflight passed (119 fresh frames in
eight seconds), and both real Gemini route and paired-image API checks passed.
No motors were commanded. A staged occlusion and chassis test remain pending.

Gemini interprets visible objects and compares left/center/right candidates. The
deterministic planner keeps authority over every movement. No hardware was used
to prepare this change; image heuristics and stopping still need a prepared live test.

## Route cycle

1. Stop with the approved floor view. Capture a JPEG and segment candidate corridors.
2. Ask Gemini about visible floor, hidden floor, solid/thin obstacles, people,
   steps/drops, nearby turn space and route preference.
   Local segmentation and Gemini share the exact horizon-trimmed near-floor
   polygons (top at the larger of the floor horizon and image y=0.50).
   Furniture/person regions outside those polygons are context, not a veto
   based on apparent size or horizontal alignment. Overlapping obstacles and
   hidden floor still block; equally clear candidates prefer straight ahead.
3. Capture and segment again. Reject a changed, frozen, dark, stale or differently
   positioned view. Require the robot to remain stopped throughout the request.
4. Keep only candidates passing **both** local checks and every semantic veto.
   Local thresholds remain 95% floor among known pixels and 75% known coverage.
   Gemini's small ranking bonus only orders surviving candidates.
5. Execute the existing bounded turn, stop, recheck the forward corridor, then
   travel at most 5 cm at the existing 0.06 m/s development speed. Stop and
   reacquire the target before another approach step.

A cable or chair leg can veto a corridor even when segmentation calls most of it
floor. Hidden floor stays blocked. A sideways route approval cannot authorize
forward travel after turning. No automatic reverse or blind movement is added.
The camera remains a monocular heuristic: these checks do not measure depth,
prove clearance behind the chassis, or guarantee detection of every thin object.

## Occlusion cycle

Within a stationary person investigation, retain one recent candidate JPEG and
its camera/motor reference. If local person evidence disappears, Gemini compares
that image with the current one. The pair expires after 30 seconds and is consumed
once. Camera-reference changes and chassis heading changes prevent its reuse.

- Measured tilt change: restore the previous useful view within approved limits.
- Person partly hidden by furniture: hold briefly for fresh recognition, then rescan.
- Person outside the view or uncertain cause: resume the bounded search.
- Dark, blurred or obstructed view: stop the investigation.

An indicated left/right edge is recorded for diagnosis; it does not cause lateral
travel. Occlusion never confirms identity. A fresh local face takes priority.
Memory is transient within the scan process; it is not carried across approach
subprocesses, robot reboots or searches.

## Runtime and verification

The Mac's existing route service now uses Gemini on `/route`; `/view` retains
its camera-calibration role. `/occlusion-advice` accepts two caller-supplied JPEGs.
The existing Vertex ADC identity stays on the Mac. No new API key is required.
All robot cloud decisions use `gemini-3.5-flash-lite` with minimal thinking, selected in
`robot/cloud_models.py`. The old navigation-only model override is no longer read.
One cloud call has an eight-second HTTP timeout, with no internal retries and a
failure cooldown. Cloud failure blocks route execution; person investigation can
fall back to its existing bounded detector rules. Images are not saved by these endpoints.

The image-change guard samples the visible route region at 160×120. More than
2.5% of pixels changing by over 35 intensity levels, mean intensity below 15,
or identical JPEG bytes reject the pair. These are conservative initial settings,
not measured obstacle-detection accuracy. Both machines need synchronized clocks
for the one-second delivered-route freshness check.
Stopped route requests allow up to eight seconds for delayed feedback to return,
then require unchanged motor references and pose. An image that expired during
that wait is refreshed once. Feedback limits during movement are unchanged.

Offline commands from the repository root:

```sh
python3 scripts/phase6/check_build.py
python3 scripts/diagnostics/simulate_navigation_reasoning.py --report /tmp/navigation-replay.json
python3 scripts/phase6/phase6.py bundle --output /tmp/echora-navigation.tar.gz
```

When ready, power the Jetson first, install the verified bundle using the existing
[Phase 6 procedure](../scripts/phase6/README.md), restart the Mac route service,
and run the Jetson-only health check. Leave EV3 off until that passes. The prepared
hardware test is a stopped floor observation, one supervised 5 cm detour, then a
stationary person-occlusion test. Turn EV3 off after that short test.

API references: [Google structured output](https://cloud.google.com/vertex-ai/generative-ai/docs/multimodal/control-generated-output)
and [multiple-image understanding](https://cloud.google.com/vertex-ai/generative-ai/docs/multimodal/image-understanding).
