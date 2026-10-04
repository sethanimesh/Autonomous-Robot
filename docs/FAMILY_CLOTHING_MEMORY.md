# Family clothing memory — implementation and rollout

The combined implementation is installed and active on the Jetson. Me retains 18 templates; MOM has 17. Mom was recognized by face and her first outfit was saved with six views. After fixing brief detection-gap handling, a 15-second repeat retained one stable track and matched her outfit, but face recognition was also present throughout. The operator waived further pose checks and accepted clothing recognition provisionally; independent clothing identification is not marked verified. Clothing approach remains disabled pending metric-range calibration and approach acceptance.

## What runs where

| Component | Responsibility |
| --- | --- |
| Jetson SQLite `data/people.sqlite3` | Named family profiles, stable IDs, face enrollment revisions, permanent outfits and clothing crops |
| Target recognizer | Compare each face with every family profile; separate face-track histories, existing threshold and two-of-five default, 0.05 runner-up margin |
| Long-lived target observer | Join body/image/face evidence, retain tracks across scan subprocesses, learn face-anchored outfits, compare known clothes and request range |
| Mac `/wardrobe` | Gemini 3.5 Flash Lite via existing ADC; structured `describe` and direct image `compare` operations |
| Mac `/person-range` | Depth Anything V2 Metric Indoor Small, same-frame person/floor segmentation and measured head geometry |
| Existing mission workers | Bounded search, clearer-view rules, route checks, short steps and arrival decisions |

No ADK, LangGraph, Mem0, vector database, or new cloud memory service was added. Gemini has no actuator API.

## Permanent records and UI

SQLite schema version 2 migrates the legacy selected-target JSON once, preserving the original file for rollback. All 18 existing samples are copied when present. Adding a profile creates a new stable ID; updating an existing enrollment changes its revision. Rename preserves ownership. Delete cascades to the person's outfits and crops. The migration marker prevents deleted legacy enrollment from reappearing on restart.

Each outfit retains up to six views. Perceptually redundant views refresh an existing view; at capacity, the older member of the closest pair is replaced. Distinct outfit records have no expiry. Crops are cut below the current confirmed face before permanent storage. Enrollment photos still follow the existing storage choice, and cleanup checks every remaining profile plus the retained legacy rollback file.

The UI has a family selector, **Find [name]**, add/update/rename/delete actions, a **Remembered clothes** gallery and individual forget buttons. The selected stable ID and revision are passed into the mission and every scan worker. UI selection and enrollment changes are blocked during an active mission. A worker rejects observations for a different profile or revision.

Relevant APIs:

- `POST /api/family/action`: `select`, `rename`, `delete`, `forget`, with `profile_id` and the relevant `label` or `outfit_id`.
- `POST /api/enrollment/start`: omit `profile_id` to add a member; supply it to update that member.
- `GET /api/family/wardrobe`: selected member's remembered clothes.
- `GET /api/family/range-frame`: a fresh, stationary raw image and its exact target box, held only in memory for measurement.

## Recognition and continuity

1. Detect body regions locally. The face worker can be absent: body tracking proceeds after a brief same-frame join window.
2. Associate a confirmed face only when at least 80% of its box belongs to exactly one current body.
3. Shortlist stored views using region colour and texture descriptors. The current detector does not publish person masks to the tracker, so its local descriptors use inner body crops. These are coarse cues, not reliable anatomical segmentation.
4. Gemini compares the current crop with up to two references, preferring references from different plausible family owners. A unique supported match can establish clothing identity. Uncompared near-identical competing owners remain unresolved.
5. Continue locally using the accepted appearance reference. Clothing-only tracking never updates permanent ownership or gradually replaces the trusted descriptor.

Cloud results carry a request ID, frame key, track ID, motion epoch, profile revision and image hash. A newer face identification overrides a pending clothing answer. Deleted profiles/outfits, moved cameras, stale tracks and mismatched replies cannot receive delayed identity assignments. The observer sends one wardrobe request at a time, rejects replies after ten seconds, respects quota backoff and avoids repeated calls for unchanged candidates. Local tracking and Stop remain independent of that request.

Hidden garment regions remain unknown. Normalized enum fields control matching and camera advice; free text is display-only. Identical reference pixels cannot establish a unique owner, even if Gemini selects the first reference. Shared clothes are permitted in the database and can remain ambiguous.

Motion invalidates image coordinates and range while keeping appearance memory. New detections reacquire position. A continuously supported track has no ten-second face expiry. Brief gaps and crossings trigger reacquisition or another view, rather than erasing the wardrobe. Live tracks are temporary and disappear after 30 seconds without observations; durable clothes remain available for a new track.

`/perception/people_tracks` and `/mission/target_observation` expose profile/track IDs, identity source, current body box, source frame/time, visible clothing regions when known, calibrated bearing when available, and separate range. Legacy face confirmation is never synthesized from clothes. UI messages are “Recognized by face,” “Matched saved clothes,” “Tracking,” and “Looking for a clearer view.”

## Range and approach

The selected model is `depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf`. The Mac retains the warmed model. Ranging uses segmented person pixels inside the current target box, calibrated rays (including distortion), head-position-dependent height/pitch/front offset, and a measured depth scale. Floor geometry checks scale consistency; it does not silently change the validated scale. Optical-axis depth and forward ground distance are distinct.

The reported gap is from the robot front to a near visible person surface, with an uncertainty interval. Near arrival, a lower head view, untruncated body boundary, visible floor and person pixels near the measured ground plane are required before feet are considered checked. This is a geometric visibility check, not an anatomical foot detector; hidden feet can leave approach incomplete.

The policy targets 0.60 m, requires the interval to fit within 0.50–0.70 m for arrival, and requires two consistent fresh measurements. Steps retain the current speed, are route checked, never exceed 0.10 m, and shorten near the target. Camera/chassis movement invalidates old range. Missing or stale range requests another view while retaining identity. Unavailable range ends with “person located; approach incomplete,” not successful arrival. Actual metric accuracy has **not** been established for this robot.

## Prepared checks

### Automatic approximate range (2026-09-12)

The webcam-only mode no longer requires a manual person-range calibration file.
`approximate_approach_enabled` and `clothing_approach_enabled` now default to true
on the target observer. Both settings were confirmed active after the operator's
Jetson reboot on September 12. These settings do not start a mission by themselves.

The Mac uses the installed indoor metric depth model, the camera's existing
intrinsics, a segmented person region, and a robust floor-plane fit. The existing
15 cm lower-camera measurement supplies a nominal scale reference at the lower
view only. This is a prior, not a fresh measurement of the current mount. Other
head views inherit that scale correction and estimate their own height and tilt.
A first raised view requests a lower view to initialize scale; a changed camera
reference or intrinsics invalidates it. A floor pose can survive a brief occlusion
for 30 seconds only at the same camera reference, angle, intrinsics and motion
epoch. Otherwise the robot requests another lower view.

On an archived approved lower image, the raw model implied a camera height of
0.639 m. The nominal lower-height reference corrected its scale by 0.235. This
checks the initialization path with actual model output, not person-distance
accuracy. The model can still have different scale errors on later views.

The September 12 right-side Mom run exposed a live discrepancy: the operator
reported a 34 inch (0.8636 m) robot-front gap where the camera-origin estimate
was 0.3421 m. Recognition and clothing reacquisition worked, but this range
estimate cannot establish correct stopping distance. The camera/front offset and
height at the newly selected lower view remain unmeasured. Each approximate
result now includes numerical geometry diagnostics; the reported gap is stored
as failed working evidence, not used as a universal distance multiplier.

Every result stays `validated: false`. Uncertainty is a heuristic allowance,
not a measured accuracy bound. Because the camera-to-chassis offset is unmeasured,
the distance origin is the camera's ground projection; the front-of-robot gap
remains unverified. A valid measured calibration retains priority and is never
overwritten by these estimates.

Two consistent fresh estimates can authorize route-checked steps up to 5 cm,
shortened to 2 cm at model estimates of 0.30 m or less; existing encoder control
and speeds remain in place. At the operator's request on September 12, the
approximate stopping threshold is now 0.20 m, with immediate close stopping
below 0.12 m. These are model estimates and do not claim those physical gaps.
Within an estimated metre, inspect lower for potentially closer feet before
approaching further. If a fresh floor estimate already comes from the lower
quarter of the saved head range, uncertain foot segmentation permits a 2 cm
route-checked step instead of requesting the same lower view repeatedly.
This leaves `feet_checked` false and does not validate the physical gap.
Raised views and cached floor poses still need the lower inspection.
Estimated arrival in the 0.12–0.20 m band has a distinct
`target_found_at_estimated_standoff` outcome and UI wording. Missing distance
retains identity and requests another view, then reports incomplete approach if
it remains unavailable. The recorded-image and measured workflows below remain
available to evaluate actual accuracy; they are not prerequisites for this mode.
The closer policy is installed on Jetson and will load on the next mission.
Floor-marker preparation is paused; its proposed workflow is not required.

Use the Mac environment `/Users/animesh/Library/Caches/Echora/depth-venv/bin/python` from the repository root for these commands. None of the family-memory utility commands moves a motor.

```sh
python scripts/phase6/check_build.py
python scripts/phase6/family_memory.py warm-depth
python scripts/phase6/family_memory.py status --database /path/to/people.sqlite3
```

### Recorded clothing evaluation

`evaluate-wardrobe` accepts a JSON manifest with `references` (`id`, `profile_id`, `image`) and `cases` (`id`, `image`, `references` as an ID list, `expected_profile` or null, `scenario`). Image paths are relative to the manifest. Supply clothing crops; use genuinely different candidate and reference images. Self-image comparisons are rejected as acceptance evidence.

```sh
python scripts/phase6/family_memory.py evaluate-wardrobe family-cases.json \
  --execute-cloud --output family-evaluation.json
```

The report records every decision, missed identification, uncertainty and wrong owner. Prepared-check coverage requires two family members, a correct match for each, no wrong owners, and cases for `same_outfit`, `different_outfit`, `partial_trousers`, `side_view`, `low_light`, `shared_clothes`, and `crossing`. It never enables movement. Review the individual abstentions as well as the summary; passing this small evaluation is not a general accuracy claim.

### Measured range calibration

For repeatable **offline** geometry diagnosis, the numeric captures can now be
replayed without inference, Mom or the robot:

```bash
python scripts/diagnostics/person_range.py replay \
  artifacts/person-range-same-position-20260912/comparison.json \
  artifacts/person-range-same-position-20260912/foot-on-floor.json \
  --output /tmp/person-range-replay.json
```

The report keeps nearest visible body distance separate from the measured foot
gap; bottom image pixels are not automatically feet. It compares inferred tilt
only for matching camera intrinsics and stopped encoder references. The saved
sparse samples cannot rerun segmentation or test the new connected-foot crop
recovery. The depth model's scene-dependent tilt/scale error remains unresolved.

When the operator has readied a person for a stationary check, capture reusable
numeric evidence with the current measured lens height (metres):

```bash
python scripts/diagnostics/person_range.py capture \
  --height-m 0.1524 --gap-m 0.8636 --measured-region nearest_foot \
  --feet on-floor --output /tmp/person-range-capture.json
```

Those example measurements belong to the September12 lower-view setup; reuse
them only when the operator confirms the same physical position. Multiple
visible candidates require an explicit `--candidate` index. This diagnostic
does not establish identity, save images, move motors, or change calibration.
It retains connected-mask extension evidence for future checks without another
capture. Normal range processing now includes small connected mask extensions
and checks floor directly under the visible lower surface. This crop fix is
active; its live effect on Mom's feet has not yet been observed.

For a quick diagnosis before the full accuracy evaluation, `diagnose-range`
compares raw model depth, the historical 15 cm height prior, and an optional
current measured lens height on the **same** stationary person frame. It uses
the Jetson's actual intrinsics and records the head/motor references associated
with that image. Stale feedback, pending motion and a pose change after capture
request another capture without affecting the mission or manual controls. No
image is saved, and the command does not change runtime range settings.

```sh
python scripts/phase6/family_memory.py diagnose-range \
  --height-m <current-lens-height> --output range-diagnostic.json
```

Optionally add `--front-offset-m <offset>` and either `--measured-m <gap>` or
`--measured-inches <gap>` for a comparison from the same distance origin. The
offset is positive when the robot front is ahead of the camera's ground
projection. These measurements must describe the captured view; do not attach
the earlier 34 inch measurement to a new frame after Mom or the robot moved.
Missing measurements stay unknown. A passing single point never enables
validated ranging. The earlier 34 inch discrepancy is retained separately in
`artifacts/mom-combined-right-20260912/operator-distance-check.json`.

The range-capture endpoint update is active after the operator's Jetson restart
on 2026-09-12; installed hashes and service activation were checked. It requires the EV3's stopped feedback when
capturing; keep the EV3 off during development and start it only for the prepared
short check. Normal encoder settling is mapped consistently to the lower/upper
view when recording a calibration sample.

The operator measured the current lower-view lens height as **6 inches
(0.1524 m)**, recorded against its head reference in
`artifacts/range-height-check-20260912/operator-lens-height.json`. This differs
from the historical height prior by only 1.6%; applying that change alone to
the old estimate would move it from 0.3421 to 0.3476 m. It cannot explain the
earlier 0.8636 m gap. The old person image was not retained, so that measurement
can diagnose the recorded failure but cannot calibrate a new frame. A regression
also demonstrates that inverse-depth bias can produce a perfect floor-plane
fit and a wrong inferred tilt/range even with correct lens height. This is a
synthetic limitation check, not proof of the live error's exact cause. Do not
interpret plane consistency as independent range accuracy or apply a universal
correction from this one observation.

An experimental independent angle check is available in
`scripts/diagnostics/compare_camera_tilt.py --live --output tilt-check.json`.
It compares [GeoCalib](https://github.com/cvg/GeoCalib) image gravity with the
depth-derived floor tilt on one stationary frame; no motor commands or images
are retained. For the prepared Mac installation, run it with the depth virtual
environment and set `PYTHONPATH` to the cached `GeoCalib` and `geocalib-deps`
directories, `TORCH_HOME` to `~/Library/Caches/Echora/torch`, and `HF_HOME` to
`~/Library/Caches/Echora/huggingface`. These optional dependencies are isolated
from the running services. The live lower/raised comparison followed head
movement, but a saved ceiling view gave a wrong-sign result. It is diagnostic
only and does not replace production ranging or establish metric accuracy.

The same-position person check on 2026-09-12 reproduced estimates of0.321 m
and0.244 m at the operator's0.8636 m gap, with depth-derived tilt changing
despite an unchanged head pose. Sparse numeric samples are retained in
`artifacts/person-range-same-position-20260912/`. The experimental
`robot/mac/range_anchor.py` fits inverse-depth scale and offset from separate
geometry instead of letting depth establish its own tilt. It improves the
floor-contact anchor frame, but the separate lifted-foot frame still misses
the reported gap by about0.22 m. It is not used by the running range service
or approach policy; neither a general correction nor metric acceptance has
been established.

For each stationary sample, measure the gap from the robot front to the nearest person surface. Also measure lens height and the distance from the camera's ground projection to the robot front (positive when the front is ahead). Camera pitch is estimated from a robust fit of the segmented depth floor plane; an optional measured pitch (positive downward) can override it when floor evidence is insufficient. The plane fit is calibration evidence, not range acceptance. The normalized head fraction is read from the saved lower/upper limits; the old 15 cm height is not applied globally.

Capture at least three training samples and six held-out samples. Held-out samples need at least three at 0.5–1.0 m and three at 1–3 m, both seated and standing in each band, and at least two useful head positions. Record physical values with:

```sh
python scripts/phase6/family_memory.py capture-range \
  --measured-m <measured-gap> --height-m <lens-height> \
  --front-offset-m <measured-offset> \
  --posture seated --split validation --output samples/example.json
```

The captured `head_pose` contains the measured height/offset and estimated or explicitly measured tilt. Use `docs/calibration/person_range_head_poses.example.json` to record those values at the relevant lower/raised positions. Nulls are intentional placeholders and cannot pass calibration. Then:

```sh
python scripts/phase6/family_memory.py calibrate-range samples/ \
  --head-poses head-poses.json --output person_range_calibration.json
```

Acceptance requires maximum held-out error ≤10 cm at 0.5–1.0 m and ≤20% at 1–3 m. Frames cannot be reused between training and validation. The output records tested head coverage; out-of-coverage angles cannot borrow its validated accuracy. Copy the result to Jetson `data/person_range_calibration.json`. Failure leaves recognition usable and range unvalidated.

### Deployment and live sequence

1. Install the prepared Jetson bundle with the existing installer, which backs up replaced runtime files and preserves data/configuration. Restart the usual perception/observer/UI units and restart the Mac `com.echora.route-perception` LaunchAgent to load the new endpoints. Do not change track-speed settings.
2. With EV3 still off, check migration/profile counts and observer health, then use stationary images to learn and recover clothes. Check family confusion before enabling clothing-based pursuit.
3. With EV3 ready, confirm this boot's saved head range. Test learn outfit → hide face → head/chassis turn → reacquire → restart services → reacquire again → select the other family member.
4. Run the combined family approach using the approximate mode above. To evaluate verified front-gap stopping separately, complete measured range acceptance and install its calibration file. For observation-only trials, set `clothing_approach_enabled` false on `/echora_target_observer`.
5. Verify the 60 cm approach with both seated and standing examples, Stop during cloud inference, and camera-loss recovery.

The previous “robot unexpectedly reports motion” event is handled separately: bounded search can send Stop, refresh feedback, verify unchanged encoder references and continue the same scan. This fixes a missing recovery path. The physical cause of the earlier event still needs the unavailable Jetson log or a live replay.

## Evidence from this offline build

- The actual Mac depth model loaded and a warmed 640×480 inference took about 0.108 s. This excludes segmentation/network time and says nothing about metric accuracy.
- Gemini described an authorized archived clothing image and matched a self-reference in roughly 4–5 s. A duplicate-reference trial initially selected one identical reference; the deterministic ambiguity correction was added and retested successfully. These are integration checks, not family-confusion acceptance.
- Automated tests cover migration with 18 templates, persistence, multiple profiles/outfits, ownership, crossings, low light, restart recognition, late responses, strict schemas, geometry, hidden feet, stale range and bounded approach policy.
- Browser verification used a local mock with two family names, including gallery empty state and the family-panel layout. It did not connect to the robot.

Actual family examples, held-out range accuracy and hardware movement remain pending. The EV3 can remain off until those checks are prepared. The combined installation and activation checks are recorded in `artifacts/gemini-3.8-flash-20260909/deployment-report.json` and `family-live-check.json`.

### Partial-view continuity update (2026-09-12; activation pending)

The tracker keeps a temporary ordered clothing signature from a face-confirmed or Gemini-confirmed view. If lowering the camera leaves only part of that view, it compares aligned colour, brightness and texture strips instead of assuming the crop's top is always upper clothing. Two fresh matching observations reconnect a unique recent track after movement; range and screen coordinates are acquired again. Shared clothing and multiple visible people remain ambiguous. Temporary matches do not teach ownership or replace trusted references. Tests use generated crop changes; a Mom live run is still required after restarting the Jetson tracking service.

### Appearance-only reacquisition scores (2026-09-12)

When movement has invalidated the screen positions of every candidate, the tracker
normalizes all appearance-only scores to0–1 before applying its existing0.08
uniqueness margin. It does not individually boost stale candidates when another
candidate still has useful spatial evidence. On the first saved Mom recording,
this recovers the original track after a split leg detection:289 evaluable frames
retain identity after the recorded anchor, versus220 before. This single-run
continuity result does not validate a physical head transition (battery ended
that run) or independent family identification accuracy. Runtime deployment awaits
the next tracker-service/Jetson restart.
