# On-Call Hospital Assistance with Recipient Directed Care Coordination

This dossier retains the original repository-grounded technical investigation and measurements. The 2026-10-04 documentation revision applies the hospital assistance scenario and records the current supervised message-delivery code. It is a source review, not a new live evaluation. Repository links work from this folder; some raw evidence and the adjacent communication app remain local-only. Historical source line citations may have shifted since the original investigation.

See [reading guide](README.md), [source snapshot](SOURCE_SNAPSHOT.md), [reproduction requirements](14_REPRODUCTION_REQUIREMENTS.md), and [current delivery contract](10_HOSPITAL_SCENARIO_AND_REQUEST_DELIVERY.md).

---

**This project is presented as On-Call Hospital Assistance with Recipient Directed Care Coordination. Its implementation combines selected-person search, bounded approach, and supervised approved-message delivery.** The principal systems work coordinates identity evidence, a shared movable camera, delayed remote inference, and short feedback-monitored movements.

Recorded runs demonstrate finding an enrolled person, selecting a route, moving short distances, reacquiring the person, and stopping. They also document incomplete approaches, inaccurate distance estimates, camera and communication issues, and recovery limits. The current delivery code adds gated robot playback; the retained trials do not establish a complete physical request-to-recipient delivery, and there is no caregiver-acknowledgement mechanism.

**Original investigation snapshot.** I inspected branch `main`, HEAD `1e8fbb63b7bb09c54e2c065aa39fd336e6843a63`, dated September 4, 2026. The working tree contains 54 modified tracked files and substantial untracked material, including newer family recognition, appearance memory, range estimation, recording tools, and communication code. Consequently, that commit alone cannot reproduce the system described below.

The original investigation was read-only: it did not run tests, import project modules, contact devices or services, activate cameras or motors, install dependencies, or modify source files. The current revision includes documentation/source review and 138 passing offline checks through the [reproduction harness](../scripts/diagnostics/reproduce_checks.py). No live hardware, model inference, or network service was exercised. Historical execution results remain identified as such.

---

**A. Technical goal and demonstrated scope**

**Hospital assistance scenario.** A patient needs urgent assistance while the assigned nurse or doctor is occupied elsewhere in the same room and does not have a phone in hand. A general callout may attract another person; a notification still depends on someone checking a device. The robot's proposed role is to locate the pre-enrolled caregiver named in the request and bring that request to their attention.

The robotics contribution is identity-aware, recipient-directed search and approach: choose whom to find, obtain useful person views, retain qualified identity evidence, inspect the route with the same motorized webcam, and supervise short movements. The scenario supplies the motivation; the recorded prototype evidence comes from supervised household/room trials.

In plain language, the supported goal is:

> Find a selected enrolled person in one prepared room, maintain a qualified estimate of which visible person is the target, inspect the floor with the same camera, and attempt a short approach while stopping or reconsidering when observations or robot feedback become unusable.

A technically precise formulation is:

> Coordinate profile-bound face and appearance evidence, camera-view transitions, pose-bound distributed visual assessments, and encoder-monitored motion primitives on a tether-constrained EV3/Jetson robot, with explicit invalidation and bounded recovery when identity, scene, or actuator evidence becomes stale or inconsistent.

This formulation defines the system-level coordination problem and the scope of the prototype.

| Scope level | What the evidence supports |
|---|---|
| Intended application | On-call hospital assistance: locate the named, pre-enrolled nurse or doctor and present an approved assistance request. |
| Implemented robotics | Configurable enrolled-recipient selection; face and clothing-supported identity; bounded camera/chassis search; route assessment; short approach segments; cancellation and recovery. |
| Recorded physical capability | Selected-person search, route-checked movement, reacquisition, and stopped feedback in supervised indoor trials. |
| Arrival | Implemented stopping policies, including an explicitly approximate outcome. Reliable physical standoff is unverified. |
| Assistance-message delivery | Current console code integrates reviewed text or an edited speech transcript, Mac synthesis, supervised search, and gated Jetson speaker playback. Complete home delivery is reported by the project owner; retained artifacts document partial trials. |
| Acknowledgement | Human acknowledgement is reported in the home simulation. No software receipt/acknowledgement mechanism is implemented. |
| Hospital deployment or clinical benefit | Not established in the inspected code, reports, or development history. |

**Current-source delivery update (2026-10-04).** The original investigation described the adjacent communication app as a stationary slice. The current robot console now implements **Find & deliver**: approve the exact message for a selected profile/revision, preview audio on the robot, run the supervised search, and permit playback only when the final report and fresh feedback satisfy the delivery gate. Estimated arrival is insufficient. The gate requires a validated measured range interval, unique selected identity, matching track, and stopped tracks/head. It permits identity sourced from face **or clothing**, rather than demanding a fresh facial confirmation at playback.

`played` denotes completion of the playback process. Arrival accuracy and recipient receipt are evaluated separately; the software does not record acknowledgement. See [robot message-delivery contract](../docs/ROBOT_MESSAGE_DELIVERY.md), [delivery gates](../robot/jetson/perception/speech_delivery.py), and [console integration](../robot/jetson/perception/enrollment_console.py).

**Home scenario demonstration.** On 2026-10-04, the project owner reported a complete home simulation: the robot found the selected recipient, approached, played the request, and the person acknowledged it. This complements the retained partial-trial artifacts. Trial count, timing, independently measured stopping distance, and a recording are not documented. Acknowledgement was observed by a person; the software does not record recipient acknowledgement.

The retained physical demonstrations are complementary:

- An older integrated run completed **three approach/reacquisition cycles**, with encoder-estimated movements of approximately **4.04, 4.48, and 4.73 cm**, then encountered a route check limit. It lasted **91.14 seconds** and did not establish arrival.
- A September 12 run found the target, chose a left route, turned **−39.82°**, checked the new forward view, drove an encoder-estimated **6.39 cm**, reacquired the target by face, and stopped. Its outcome was `step_complete_target_reacquired`, after **68.21 seconds**.
- A later clothing-supported run requested **5 cm**, recorded **6.77 cm** of encoder-estimated movement, reacquired the person, then lost usable identity during lower-view inspection. It ended `target_found_not_at_standoff` after **88.62 seconds**.

These are physical integration results, not a complete arrival benchmark. Sources: three-cycle mission report — `docs/perception/integrated_live_20260905/multistep-route-retry-live-20260905/mission.json`, September 12 one-step summary — `artifacts/mom-approach-near-floor-retry-20260912/summary.json`, and later approach result and limitations, lines 2618–2622 — `docs/DEVELOPMENT_LOG.md` (local development evidence) (source line 2618).

---

**B. Architecture and mission trace**

The executable architecture is a collection of managed Python processes and bounded mission subprocesses. ROS 2 supplies the Jetson message bus; the EV3 has a small TCP service. The inspected active mission does not use Nav2, a SLAM map, or ROS navigation actions.

```text
Operator browser
  │ select enrolled profile + confirm cable-neutral setup
  │ POST /api/mission/start
  ▼
Jetson enrollment console
  └─ FindMissionController
       └─ autonomous_find.py
            ├─ camera_head_calibration.py
            ├─ bounded_target_scan.py
            └─ closed_loop_detour.py
                       │
                       ├──────── ROS /cmd_vel ──────────────┐
                       └──── ROS /camera_head/command ──────┤
                                                            ▼
USB webcam ──► camera_node                           ROS EV3 bridge
                 │ /camera/image_raw                        │
                 │ /camera/camera_info                      │ TCP JSON
                 ▼                                          ▼
          YOLOX-s person detector                       EV3 service
                 ▼                                      ├─ B/C tracks
          YuNet face detector                           ├─ A camera tilt
                 ▼                                      ├─ encoder feedback
          AntelopeV2 embeddings                         └─ local timeouts,
            ├─ selected-target matching                    interlocks, stop
            └─ family-profile matching                       │
                       │                                      │
SQLite profiles / outfits ◄─► FamilyObserver / tracker        │
                       │                                      │
             /mission/target_observation              /robot_status
                       │                               /camera_head/status
                       └────────► mission ◄─────────── /odom

Jetson ── HTTP over reverse SSH tunnel ──► Mac route service
                                             ├─ SegFormer-B0, MPS
                                             ├─ DA V2 Metric Indoor Small, MPS
                                             ├─ range / view / wardrobe adapters
                                             └─ Gemini structured visual advice
                                                        │
                                                        ▼
                                                      Cloud

Current supervised message delivery:
reviewed text / edited transcript → Mac ASR/TTS → approved audio
  → Jetson Find & deliver → final identity/range/stopped-feedback gate
  → Jetson speaker playback [recipient acknowledgement not implemented]

Adjacent communication app: separate stationary communication workflow
```

The service files launch loose Python files from the deployed Jetson runtime directory, rather than demonstrating a conventional packaged ROS workspace. The browser launches the top-level mission process; that process starts calibration, scan, and movement workers. Relevant launch evidence includes console service — `robot/jetson/perception/echora-enrollment-console.service:6`, target observer service — `robot/jetson/mission/echora-target-observer.service:6`, bridge service — `robot/jetson/ev3_bridge/echora-bridge.service:6`, and Mac route-service launch configuration — `robot/mac/com.echora.route-perception.plist:7`.

The main interfaces are:

| Interface | Actual role |
|---|---|
| `/camera/image_raw`, `/camera/camera_info`, `/camera/status` | Raw images, intrinsic calibration metadata, capture health. |
| `/perception/person_detections` | Timestamped person boxes. |
| `/perception/face_detections`, `/perception/face_observations` | Face boxes and transient five-landmark observations. |
| `/perception/target_matches`, `/perception/recognition_status` | Legacy selected-target matching and temporal status. |
| `/perception/family_matches`, `/perception/people_tracks` | Family-profile face results and current identity/appearance tracking. |
| `/mission/target_observation` | Mission-facing identity, position, range, and freshness evidence. |
| `/cmd_vel` | Unstamped `Twist` velocity commands. |
| `/camera_head/command`, `/camera_head/status` | JSON command/status messages for the shared camera mechanism. |
| `/robot_status`, `/odom`, `/joint_states`, TF | EV3 feedback and encoder-derived robot state. |
| HTTP `/route`, `/person-range`, `/search-view`, `/wardrobe`, `/camera-setup` | Mac-backed visual computation. |
| EV3 TCP port 9999 | Newline-delimited JSON motor commands and feedback. |

The bridge publishes `odom → base_link`. I found no camera-to-base extrinsic TF or robot-description path in the inspected `robot`, `scripts`, and `config` material. Camera frame labels therefore should not be mistaken for a complete calibrated spatial transform tree. See bridge interfaces and transform publication, lines 305–321 and 410–419 — `robot/jetson/ev3_bridge/ros_node.py:305`.

A normal browser-started mission follows this path:

1. **Bind the request to a selected enrollment.** The browser sends the displayed profile ID and revision. The console rejects a mismatch with the currently selected profile, active manual driving, enrollment in progress, moving robot, or stale robot/head/camera readiness.
2. **Acquire control ownership.** `FindMissionController` obtains an exclusive camera-control lease, creates a child process group, removes an old report, and starts the mission.
3. **Prepare the camera.** The mission reuses matching saved useful views or performs bounded discovery.
4. **Search.** The scanner combines fixed sweeps with face/body/cloud-guided investigation. It returns target identity evidence, useful head position, and measured cable heading.
5. **Choose whether more approach is warranted.** The policy distinguishes legacy face-size stopping, measured-range stopping, approximate-range stopping, missing range, and identity reacquisition.
6. **Inspect the floor and execute one primitive.** The detour worker lowers the camera, obtains local/cloud route evidence, turns if needed, checks the new forward corridor, and drives a short segment.
7. **Reacquire the target.** A new local scan follows every completed or interrupted approach segment before another target approach.
8. **Report and stop.** The result preserves outcomes such as incomplete approach, estimated arrival, no target, pause, or error; the console’s completion callback requests a stop.

The code anchors are console preflight, lines 1551–1598 — `robot/jetson/perception/enrollment_console.py:1551`, mission process ownership, lines 209–349 — `robot/jetson/perception/mission_control.py:209`, and top-level mission, lines 194–469 — `robot/jetson/mission/autonomous_find.py:194`.

**Current-source delivery entry.** `POST /api/mission/deliver` adds profile/message approval, mandatory speaker preview, the ordinary supervised mission, and post-mission playback checks. Search-only `POST /api/mission/start` does not speak. See [delivery guide](10_HOSPITAL_SCENARIO_AND_REQUEST_DELIVERY.md) and [current contract](../docs/ROBOT_MESSAGE_DELIVERY.md).

An important execution distinction: the CLI wrapper’s `run_live()` explicitly passes `--skip-camera-calibration`, whereas the ordinary browser command does not. “Every entry point automatically calibrates before searching” would therefore be too broad. See CLI launch construction, lines 189–211 — `scripts/phase6/phase6.py:189`.

---

**C. Detailed subsystem findings**

**C1. Mission contract, recipient selection, and termination**

The hospital scenario names a pre-enrolled caregiver as the intended recipient. The prototype selects that recipient through enrolled profiles; source names such as `family` and historical labels describe the household implementation and trials. Profile selection is stored locally, and browser requests carry a profile revision to avoid starting against a selection that changed after the page was rendered. Target identity, rather than nearest-person proximity, determines the requested endpoint.

The current delivery request is bound to the selected profile and approved message revisions. There is no patient assignment table, caregiver rota, or hospital directory; selection of the responsible caregiver remains an operator decision.

Implemented prerequisites include enrollment, a usable live camera, EV3/head feedback, stopped starting conditions, camera-reference preparation, route-service availability when movement is needed, and operator confirmation of cable-neutral setup. “One prepared room” is primarily an operating restriction: there is no mapped room boundary or geofence enforcing it.

The main outcomes have different meanings:

| Outcome | Meaning in the implementation |
|---|---|
| `target_found_at_standoff` | A configured stopping rule passed. Depending on path, this can be a legacy face-size heuristic or measured-range policy. |
| `target_found_at_estimated_standoff` | Approximate camera distance passed its stopping rule; the UI explicitly says the front gap is unverified. |
| `target_found_not_at_standoff` | Identity located, but approach incomplete, budget exhausted, or further motion not justified. |
| `step_complete_target_reacquired` | One bounded approach test completed and identity was reacquired. |
| `person_found_unidentified` | A person candidate remains, without adequate selected-recipient identity. |
| `target_not_found` | The configured bounded search completed without the target. |
| `paused` | A recoverable condition persisted, or heading/feedback needs intervention. |
| `failure` | Unhandled or nonrecoverable mission outcome. |
| UI `stopped` | Cancellation requested; this label alone is not independent measured stop confirmation. |

The browser maps several distinct outcomes to a broad `found` state, but retains explanatory messages and the underlying outcome. Reports should retain the underlying outcome alongside the UI state. See result mapping, lines 22–107 — `robot/jetson/perception/mission_control.py:22`.

Default top-level budgets are two search repositions, eight approach steps, three recovery attempts, one candidate retry, two target-reacquisition retries, and a 0.5-second retry delay. Child deadlines are 900 seconds for a scan, 180 seconds for an approach worker, and 600 seconds for camera preparation. These are bounds on individual operations and nested retries, not a short global mission deadline. See mission defaults, lines 473–536 — `robot/jetson/mission/autonomous_find.py:473`.

A paused report preserves diagnostic progress and heading information. CLI resume starts a **new search** using heading reconstructed from unchanged motor generations and encoder differences; it does not resume an interrupted drive or restore old face evidence. See resume reconstruction, lines 174–213 — `scripts/phase6/phase6.py:174`.

**C2. Current implementation versus prototypes and historical alternatives**

The important distinction is between the conceptual state machine and the actual orchestration.

`SingleRoomMission` defines:

```text
idle → scanning → route_check → approach_step → verify_target
                         └──────────────────────────────→ found
any abort → stopped
```

However, the scoped caller search found this class imported by its tests, not by the active `autonomous_find.py` mission. Its `ANNOUNCE_FOUND` action is an enum value; actual message playback is implemented through the current console/speech path. The active controller is procedural orchestration with report states such as `running`, `recovering`, `paused`, and the terminal outcomes above. See conceptual state machine, lines 12–147 — `robot/jetson/mission/single_room.py:12`.

Other material requiring separation:

- The older C++/Cap’n Proto EV3 service is described in historical notes; the current Python TCP/sysfs implementation is different.
- `PersonContinuity` remains useful in scanning but has stricter, different identity semantics from the newer family tracker.
- Some ground-plane/depth-obstacle helpers are exercised by tests but are not the active route-clearance mechanism.
- `config/navigation.yaml` describes values that are not loaded by the inspected active route/mission paths.
- Current console/speech modules implement the supervised delivery path. The adjacent communication app and archived speech projects are separate workstreams; the evaluation section records the home demonstration and acknowledgement workflow.
- Recording is now implemented despite the root README still saying recording has not been added.

Thus “the repository contains it” is insufficient evidence of active integration.

**C3. Hardware, camera control, and calibration**

The architecture is supported by concrete motor and sensor code:

- B and C drive the tracks.
- A drives a single vertical camera mechanism.
- Horizontal scanning rotates the chassis.
- EV3 feedback contains encoder positions, measured speeds, motor state, generation identifiers, and commanded speeds.
- The current EV3 service uses direct Linux sysfs and the Python standard library, not `ev3dev2`.
- No working IR reading or local IR obstacle-stop path was found. Hardware notes explicitly defer the absent/nonworking sensor.

See motor implementation, lines 44–159 — `robot/ev3/server/ev3_server.py:44` and [hardware inventory and sensor limitation, lines 56–87](<../docs/HARDWARE_NOTES.md>) (source line 56).

The recorded chassis envelope is **20 × 25 cm**. Effective odometry geometry is:

- Drive radius: **0.0144504 m**
- Track width: **0.182557 m**
- Encoder counts/revolution: **360**
- Bridge motor-speed cap: **240 counts/s**
- Feedback/update rate: **10 Hz**

These are effective tracked-chassis parameters derived from floor tests, not necessarily physical sprocket dimensions. Track slip remains unmodelled. See robot configuration, lines 5–15 — `config/robot.yaml:5` and calibration measurements, lines 1200–1251 — `docs/DEVELOPMENT_LOG.md` (local development evidence) (source line 1200).

The repository contains contradictory motor inventory descriptions: the supplied AGENTS guidance says A was physically identified as large; hardware notes call A medium; earlier server notes identify all three as large. The generic sysfs control path tolerates this disagreement, but it cannot establish the physical motor type.

Camera positioning measures the **motor encoder**, not optical pitch. Current EV3 safeguards include:

- Absolute raw position bound: ±180 counts.
- Maximum position-command step: 15 counts.
- Head-move deadline: 8 seconds.
- Completion tolerance: 4 counts, speed ≤5 counts/s, and stable position for 0.4 seconds.
- Stall criterion: less than 2 counts of progress in one second.
- One retry of the same target, then stop and hold.
- Both tracks stopped during a head movement.

See head completion and stall handling, lines 302–426 — `robot/ev3/server/ev3_server.py:302` and head/track interlocks, lines 429–524 — `robot/ev3/server/ev3_server.py:429`.

Named positions are valid only within a reference epoch. Boot changes, motor re-enumeration, or lost reference can invalidate old counts. Saved runtime limits override static values. Consequently, historical poses such as floor `0`, forward `−27`, face `−42` should not be presented as permanent optical angles.

The camera has two separate forms of calibration:

1. **ChArUco intrinsic calibration:** image geometry and lens distortion.
2. **Useful-view calibration:** head encoder positions that show suitable floor/room/upper views.

ChArUco uses a 5×7 board, 25 mm squares, 18 mm markers, and `DICT_5X5_100`. The accepted YAML is 640×480 with approximately `fx=416.371`, `fy=413.608`, `cx=338.723`, `cy=235.303`. Capture publishes raw images and matching `CameraInfo`; it does not rectify all camera pixels. Ranging explicitly undistorts rays; person/face and segmentation paths consume raw imagery.

A consequential exception must remain visible: the retained calibration report says **`accepted: false`**. Its 30-view fit had RMS **0.5958 px**, maximum per-view error **1.3512 px**, and a 609×450 valid ROI, but did not meet vertical-edge coverage and close-view-count requirements. Documentation records **manual promotion after live visual and timestamp checks**. It did not pass every automatic gate. See capture report — `docs/calibration/camera_calibration_capture_report_20260903.json:1` and [acceptance explanation, lines 5–22](<../docs/calibration/README.md>) (source line 5).

Current useful-view discovery is bounded: approximately ±60 encoder counts from the starting reference, 10-count exploration steps, at most 20 observations, bounded provider cooldown, and endpoint return checks. It can acknowledge the current encoder position without mechanical homing. Its result establishes usable operating views, not measured optical tilt or proven mechanical clearance. See discovery policy, lines 63–205 — `robot/jetson/mission/camera_visual_setup.py:63`.

**C4. Person search and viewpoint selection**

Search combines a fixed sweep with feedback-driven candidate investigation.

The active parent requests:

- Main search: **30° steps**, up to **±90°**.
- Local reacquisition: **15° steps**, up to **±45°**.
- Forward and lower room views, with a previously useful target head position preferred only when its reference remains valid.
- Fast-search settling overrides: approximately **0.25 s** settle, **0.1 s** held-frame interval, and **0.8 s** upper dwell.

The raw scanner has different defaults; the parent’s overrides matter. See scan construction, lines 194–244 — `robot/jetson/mission/autonomous_find.py:194` and scanner defaults/overrides, lines 1477–1544 — `robot/jetson/mission/bounded_target_scan.py:1477`.

Candidate investigation has specific rules:

- Fresh usable faces receive priority.
- A face near the upper/lower edge causes a bounded framing adjustment.
- A body cropped at the top can trigger an upward probe.
- A useful face is held steady while waiting for identity.
- A recently useful face/body view can be restored.
- No-face exploration has a 24-count upward budget.
- Candidate investigation has a nominal 12-second budget, a 60-second hard wall limit, at most four cloud checks and six head movements.
- Horizontal centering aims for the middle 35–65% of the image, using bounded ±8° corrections.

See local framing rules, lines 92–182 — `robot/jetson/mission/bounded_target_scan.py:92`, candidate investigation, lines 1064–1194 — `robot/jetson/mission/bounded_target_scan.py:1064`, and view-policy priorities — `robot/jetson/mission/search_view_policy.py:4`.

The policy’s numerical priorities—face 4, Gemini 3, body 1—are rule weights, not learned probabilities.

For multiple people, preliminary framing can follow the **largest visible body or face**, which may belong to someone other than the selected recipient. Later identity gates decide whether the target was found. This is a limitation of candidate selection, not evidence that the final recipient is simply chosen by proximity.

An unidentified candidate can receive another local attempt, then be passed during a broader scan. An absent target eventually exhausts the configured sweep/reposition budgets. Search memory is local and transient: previous useful view, candidate information, recent ceiling limits, retained cable heading, and appearance tracks. There is no active room-by-room coverage map.

**C5. Face detection, enrollment, and identity confirmation**

The implemented visual chain is more specific than a list of models:

| Stage | Input and mechanism | Current configured bounds |
|---|---|---|
| YOLOX-s | Raw BGR image, top-left aspect-preserving letterbox, raw prediction decode, person-class filtering and NMS. | 640×640; confidence 0.45; NMS 0.45; 15 Hz; maximum source age 0.5 s. |
| YuNet 2023mar | Exact-frame person regions, usually upper 72% of body plus padding; maps boxes and five landmarks back to source coordinates. Full-frame fallback if crops fail. | 640×640; confidence 0.60; NMS 0.30; 10 Hz; fallback 3 Hz; at most three body regions. |
| AntelopeV2 `glintr100` | Five-landmark affine alignment to 112×112, BGR→RGB, `(pixel−127.5)/127.5`, embedding inference and L2 normalization. | 8 Hz; at most three faces; 0.40 similarity threshold; two supporting matches within five observations. |

Sources: current perception configuration, lines 20–136 — `config/perception.yaml:20`, face synchronization and regions, lines 43–178 — `robot/jetson/perception/face_sync.py:43`, and alignment, scoring, and embedding preprocessing, lines 27–155 — `robot/jetson/perception/recognition_core.py:27`.

The matching score is:

```text
0.70 × best enrolled-template cosine similarity
+ 0.30 × mean of the best three template similarities
```

It is not a calibrated probability of identity.

Enrollment requires at least ten samples, including three center, two left, and two right views, with a maximum of eighteen. Quality checks include face size, detector confidence, brightness, blur, five landmarks, and plausible eye separation. Same-pose near-duplicates are rejected; new samples must remain compatible with existing enrollment samples. Upload and phone paths require exactly one face and use the TensorRT YuNet decoder.

The live recognition path does **not** apply all enrollment quality gates before embedding every face. Enrollment quality therefore should not be claimed as a live recognition guarantee. See enrollment session, lines 1171–1227 — `robot/jetson/perception/enrollment_console.py:1171`, quality checks, lines 91–124 — `robot/jetson/perception/recognition_core.py:91`, and live recognition, lines 163–200 — `robot/jetson/perception/target_recognizer.py:163`.

There are two confirmation mechanisms:

- **Legacy selected-target path:** one rolling boolean window receives the best selected-target score across all processed faces. It is not tied to a single face track, and the window itself has no time decay.
- **Family path:** separate face tracks use IoU association, survive for up to 1.5 seconds, require overlap ≥0.2 with adequate separation from another track, and require a winning profile margin ≥0.05. The current winning identity must receive two supporting observations.

The family path is stronger, but remains simple image-coordinate tracking. See family face matcher, lines 12–35 — `robot/jetson/perception/family_faces.py:12`.

Repeated-observation protection is incomplete upstream. The recognizer rejects a duplicate equal to the **most recently processed frame**, but does not enforce globally increasing timestamps or remember all previously processed frames. A within-age sequence A→B→A could reuse A in confirmation history. The downstream family observer rejects non-increasing source times, but upstream matcher history may already have changed. Existing duplicate tests cover immediate duplication, not that entire replay pattern. This is a code-review finding.

No inspected evidence establishes robustness to masks, similar-looking people, spoofed faces, broad lighting changes, or a representative population. Historical small acceptance trials are described separately below.

**C6. Clothing memory and reacquisition**

The current family implementation distinguishes:

- Permanent profile enrollment.
- Permanent face-anchored outfit memory.
- Temporary body tracks and appearance continuity.
- Fresh facial confirmation.

SQLite contains `profiles`, `settings`, `outfits`, and `views`. It uses foreign keys and WAL; the database and containing directory receive restrictive filesystem permissions. Each outfit can retain up to six representative clothing crops. Individual outfits and profiles can be forgotten/deleted. No outfit expiration or overall outfit-count bound was found. See schema and storage rules, lines 13–170 — `robot/jetson/perception/family_store.py:13`.

New permanent outfit ownership requires a confirmed face anchor, source frame key, and matching enrollment revision. However, re-enrollment updates the profile revision **without deleting its outfits**; the wardrobe query joins old outfits to the current revision. Pending-response revision binding should therefore not be described as invalidation of all historical appearance.

Local clothing comparison uses normalized hue/saturation histograms and texture in body-relative strips. Those strips approximate upper clothing, lower clothing, and footwear; they are not anatomical segmentation. Partial-view tracking adds eight ordered colour/brightness/texture strips and can match a sufficiently large portion of an earlier view.

Representative tracking gates are:

- Standard appearance similarity ≥0.78.
- Spatial overlap ≥0.15 when position is still valid.
- Unique best score separated by >0.08.
- Partial matching only with one currently detected person, a recent known candidate, score ≥0.90, and a second supporting observation before exposing selected identity.
- Missing bodies immediately invalidate current position.
- Unseen tracks are removed after 30 seconds.
- Motion invalidates coordinates, range, and pending view evidence while retaining trusted appearance.

See descriptors and partial matching, lines 18–76 — `robot/jetson/perception/wardrobe_tracking.py:18` and tracking/selection rules, lines 108–191 and 286–293 — `robot/jetson/perception/wardrobe_tracking.py:108`.

Cloud clothing comparison sends a cropped current view and at most two stored references. It accepts only structured comparison results tied to supplied reference IDs. Requests bind UUID, track, profile, revision, frame, image hash, and motion epoch. Responses older than ten seconds or belonging to changed state are rejected. A briefly missing body may return within the original deadline; its current appearance must still agree.

Identical reference images are explicitly treated as ambiguous even if Gemini selects an owner. New permanent ownership is not learned from a clothes-only match. See cloud request/result handling, lines 193–284 — `robot/jetson/perception/wardrobe_tracking.py:193` and wardrobe schema and ambiguity handling, lines 8–94 — `robot/mac/wardrobe_advisor.py:8`.

The crucial current behavior is:

> Clothing is not relabelled as a fresh face match, but it can authorize family-mode target acquisition, approach, and arrival.

`FamilyObserver` defaults both clothing approach and approximate approach to enabled. It keeps `confirmed` specific to facial evidence while publishing `identity_confirmed` for a qualifying selected track. The scanner and approach policy use the latter in family mode. A saved outfit can reacquire identity after restart without a new face observation on that run. Tests explicitly cover arrival without a face.

See family defaults and enrichment, lines 27–33 and 174–205 — `robot/jetson/perception/family_observer.py:27`, scanner identity gate, lines 75–89 — `robot/jetson/mission/bounded_target_scan.py:75`, and identity readiness, lines 24–35 — `robot/jetson/mission/person_approach.py:24`.

The family tracker also has no absolute “time since last face” requirement while current body support continues. The older `PersonContinuity` module’s ten-second face lifetime does not apply universally.

These distinctions make the mechanism substantive, but leave unresolved risks from similar clothing, crossings, background contamination, and incorrectly accepted appearance ownership. The tests exercise selected cases; they do not establish a household false-identification rate.

**C7. Shared-camera coordination**

The system handles the shared camera through several cooperating mechanisms rather than one unified scheduler:

- Cross-process camera-control lease.
- Single-owner head-command worker with cancellation and stop priority.
- EV3 head/chassis mutual exclusion.
- Mission-level stop/look/turn/look/move sequence.
- Invalidation of position, range, and pending visual results on motion/reference changes.
- Restoration of useful person views and post-movement target reacquisition.

The bridge clears an old chassis command when a head command arrives, so it cannot resume after the head movement. EV3 rejects drive while head motion is active and rejects head movement while tracks are active. See bridge command coordination, lines 324–339 — `robot/jetson/ev3_bridge/ros_node.py:324` and process ownership lease — `robot/jetson/mission/camera_control_lease.py:16`.

The strongest delayed-computation safeguard appears in useful-view calibration: capture is bound to head reference/position and stopped track generations/counts; after cloud inference, the consumer requires newly received feedback showing the same stopped physical state. Request UUID and image SHA also must match. See capture and post-response binding, lines 826–882 — `robot/jetson/mission/camera_head_calibration.py:826`.

There are important limits:

- Raw camera frames do not carry a directly measured optical pose.
- Recognition workers continue processing images; coordination primarily gates downstream use rather than pausing all perception.
- Encoder position cannot by itself establish what the lens sees.
- During floor inspection, the recipient may be outside the image.

Most significantly, the detour worker receives no recipient identity or target-frame binding and subscribes to no identity topic. It checks camera, head, robot, and odometry while moving; the mission reacquires the target **after** the primitive. Thus there is an identity-observation gap through lowering, route inference, turning/rechecking, and driving. See movement handoff, lines 262–299 — `robot/jetson/mission/autonomous_find.py:262` and detour subscriptions, lines 208–226 — `robot/jetson/navigation/closed_loop_detour.py:208`.

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

The investigation identified a label defect and a remaining geometry limitation:

1. **Water was included in the old floor IDs; the source is now corrected.** The investigated `(3,21,28)` default mapped to floor, **water**, and rug, allowing confident water pixels into the local floor mask and range-plane fitting. On 2026-10-04, the [engine](../robot/mac/route_perception.py) was changed to `(3,28)` and support-surface names are validated at model load. [Five regression tests](../tests/test_floor_label_mapping.py) passed. The [retained-image evaluation](../evaluation/visual-scenarios/README.md) uses the corrected labels. This was a code/configuration mismatch, not a recorded water encounter; the correction has not been deployed to a running robot.
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

If a drive is interrupted after starting, the parent requires target reacquisition rather than blindly issuing another full-distance request. Persistent identity loss or route issues pause or terminate the mission. See interrupted-drive handling, lines 338–351 and 442–459 — `robot/jetson/mission/autonomous_find.py:338`.

**C10. Distributed execution, freshness, and fault handling**

**Current-source command coordination (2026-10-04).** `HeadCommandWorker` serializes camera operations away from the ROS feedback callback; its client fence checks cancellation under the same lock as TCP writes. A replaced/cancelled operation cannot issue a subsequent movement after that check. Busy-head feedback skips blocking controller updates while telemetry continues. Mac depth/segmentation/cloud adapters supply interpretations, while Jetson mission code selects actions. The EV3 retains its local drive-refresh watchdog. These mechanisms support locally supervised execution under delayed results; current-version physical stop timing remains unmeasured in this revision. See [command worker](../robot/jetson/ev3_bridge/ros_node.py) and [EV3 watchdog](../robot/ev3/server/ev3_server.py).

Freshness exists at several layers, with different meanings:

| Layer | Implemented check | Practical limitation |
|---|---|---|
| Camera | Shape/encoding validation, reopen logic, 12-second no-publication watchdog. | Repeated frozen content can still receive new timestamps and count as publication. |
| Person inference | Newest-frame slot and 0.5-second configured age gate. | Age is cached at receipt, omitting subsequent queue delay. |
| Face/embedding inference | Exact-frame joins and age recomputed before inference. | No final age recheck after inference; future-stamp handling is inconsistent. |
| Family observer | Strictly increasing source time, ≤1-second source age, motion-boundary filtering. | Stronger than upstream recognizer; does not undo upstream vote-history contamination. |
| Clothing replies | UUID/hash/frame/track/profile/revision/epoch, ten-second limit. | Stored outfits themselves do not expire. |
| Range replies | Exact request/image/track binding; ≤3-second range age; current track. | Consistent estimates are not accuracy validation. |
| Route replies | Paired images, head/reference binding, stopped pose, usually ≤1-second delivered age. | Cross-device wall-clock synchronization is assumed. |
| Browser manual drive | Session token, increasing sequence, 0.25-second command timeout. | Depends on console/bridge/EV3 execution continuing. |
| ROS drive | 0.3-second age since callback receipt. | Unstamped commands can be old when received. |
| EV3 drive | 0.5-second time since accepted drive request. | No source timestamp, sequence, mission ID, or per-command expiry. |

Sources include camera watchdog — `robot/jetson/camera/camera_recovery.py:7`, family source/range validation, lines 65–170 — `robot/jetson/perception/family_observer.py:65`, route delivery checks, lines 314–371 — `robot/jetson/navigation/closed_loop_detour.py:314`, and manual-drive sequence/watchdog, lines 190–311 — `robot/jetson/perception/manual_drive.py:190`.

The route timestamp is Mac-side time around fetching a JPEG, not the camera’s hardware exposure time. Camera ROS stamps themselves are assigned after capture. Clock and capture latency therefore matter even when every field is internally consistent.

The EV3 does independently stop on expired drive refresh, disconnect, invalid requests, and normal shutdown. Status polling does not renew a drive deadline. However, the watchdog executes in the **same server thread** as sysfs operations and request handling. A 50 ms socket timeout bounds ordinary receive waits; sysfs operations have no explicit execution deadline. A hung process or driver cannot be claimed covered by a separately scheduled hardware watchdog. Track actuation uses `run-forever`. See watchdog, lines 623–646 — `robot/ev3/server/ev3_server.py:623` and server loop/cleanup, lines 827–868 — `robot/ev3/server/ev3_server.py:827`.

TCP client transactions are serialized, preventing interleaved responses. A stop may nevertheless wait behind a transaction already holding the lock. The local EV3 watchdog is the fallback at that boundary.

Cancellation of a browser-started mission sends SIGTERM to its process group and publishes motor-stop commands. A terminal-started mission is not owned by that browser process manager; the project documentation explicitly distinguishes those cancellation paths. No general latched emergency-stop authority or authenticated ROS command arbiter was found in the inspected path.

The console binds to `0.0.0.0` and checks an `X-Echora-Action` header for mutation requests. That header is not user authentication. The EV3 uses an allowed-client IP, not cryptographic command authentication. These are relevant limits for the hospital framing, not proof of a present network compromise. See console request handler and bind default, lines 1942–2004 — `robot/jetson/perception/enrollment_console.py:1942`.

**C11. Inference performance and resource placement**

TensorRT FP16 applies to the configured Jetson person, face, and embedding engines—not the entire pipeline.

| Component | Execution path | Evidence limits |
|---|---|---|
| YOLOX-s | Jetson TensorRT; explicit configured backend. | Auto-mode CPU fallback exists, but explicit TensorRT configuration fails instead of silently downgrading. |
| YuNet | Jetson TensorRT. | Recorded engine contained both FP16 and FP32 tensors. |
| AntelopeV2 | Jetson TensorRT, batch-one 112×112 input. | Recorded engine contained 829 FP16 tensors and one FP32 tensor. |
| SegFormer-B0 | Mac PyTorch/Transformers, normally MPS. | No TensorRT or explicit half-precision conversion in this path. |
| Depth Anything V2 | Mac PyTorch/Transformers, normally MPS. | Metric model designation does not establish accurate metres on this camera. |
| Gemini | Cloud through Mac adapters. | Current source selects `gemini-3.5-flash-lite`, `MINIMAL`; dated results used other models. |
| GeoCalib / range anchoring alternatives | Diagnostic experiments. | Not promoted into the active approach policy. |

The TensorRT builder records input/output shapes, hashes, TensorRT version, device, build time, and inspector precision counts. Face inference reads measured precision from an engine sidecar at startup; it does not independently re-inspect every engine tensor on every launch. “Mixed-precision TensorRT engines with FP16 enabled and inspected” is more precise than “all inference is FP16.” See engine metadata/build logic, lines 58–105 and 174–228 — `robot/jetson/perception/build_engine.py:58` and current cloud model constant — `robot/cloud_models.py:1`.

The recorded Jetson inventory is Ubuntu 22.04.5, L4T 36.4.3, ROS Humble, Python 3.10.12, OpenCV 4.5.4, and roughly 7.4 GiB RAM. The EV3 inventory is ev3dev Stretch and Python 3.5.3. These are historical inventory records, not refreshed device observations.

Model-stage speed does not equal mission responsiveness. A ~17 ms person detector coexists with seconds of camera movement/settling, cloud calls, body/range waiting, and retries. Recorded integrated runs lasting 30–100 seconds are compatible with fast neural inference.

I found token usage and call timings in selected cloud artifacts, but not a complete, reproducible cost-per-mission calculation. Nor did I find a controlled whole-stack latency distribution or end-to-end memory/compute profile for the final current configuration.

**C12. Data handling, evaluation infrastructure, and attribution**

Enrollment defaults to numerical templates; optional aligned face crops are separate. Clothing memory retains cropped images and descriptors, and selected crops are sent to Gemini. New recording tools can retain visible people in full camera frames. Therefore the older statement “full camera frames are never stored” is not a project-wide current guarantee.

The recorder is subscriber-only and bounded by duration, frame rate, and storage. It records source/receipt timing, camera/head/encoder state, detections, observations, and observed commands; enrollment vectors are excluded. ROS replay uses an isolated domain and does not republish recorded motion commands. It is a recorded-trajectory replay: a new command cannot change the recorded images or encoders.

The first retained real recording contains 408 frames and 6,882 events. Its mission stopped when the camera head did not move; the operator reported a depleted battery. Since the head never moved, that recording cannot validate cross-view continuity, even though it supports a useful stationary tracker regression.

Engineering attribution distinguishes the project-specific system from its external components:

- **Third-party:** pretrained detectors, embedding model, semantic segmentation, depth model, cloud models, ROS/OpenCV/PyTorch/TensorRT.
- **Repository-specific engineering:** decoders and adapters, synchronization, enrollment workflow, profile/appearance lifecycle, camera and motor coordination, bounded mission logic, result binding, recovery, diagnostics, and evaluation tooling.
- **Project owner:** responsibility for project-specific design, implementation, integration, physical testing, and evaluation, as reported on 2026-10-04. Development notes document coding-agent assistance.

---

**D. Capability–evidence matrix**

“Recorded” below means a saved report or documented execution result, not a test run during this investigation.

| Capability | Implementation status | Verification status | Exact evidence | Limitation / missing proof |
|---|---|---|---|---|
| Searches for a specified enrolled recipient | Implemented and reachable from browser | Recorded selected-person searches | console request binding — `robot/jetson/perception/enrollment_console.py:1551`; mission scan binding — `robot/jetson/mission/autonomous_find.py:240` | Caregiver selection is manual; no patient-to-caregiver assignment mechanism. |
| YOLOX-s → YuNet → InsightFace | Implemented and service-configured | Historical stationary component trials | configuration — `config/perception.yaml:20` | Full-frame YuNet fallback means person boxes are not an absolute prerequisite. |
| TensorRT FP16 on Jetson | Implemented for three perception stages | Recorded engine inspection and timings | build/precision evidence — `docs/DEVELOPMENT_LOG.md` (local development evidence) (source line 1064) | Mixed precision; Mac/cloud stages use other backends. |
| Repeated face observations confirm identity | Implemented | Unit tests and small physical trials | family matcher — `robot/jetson/perception/family_faces.py:12` | Nonconsecutive replay not fully prevented upstream; no broad confusion benchmark. |
| SQLite profiles and appearance memory | Implemented and integrated | Migration/integration/outfit-learning reports | schema — `robot/jetson/perception/family_store.py:13` | No outfit TTL; old clothes survive re-enrollment. |
| Clothing is not fresh face confirmation | Correct field distinction | Tests explicitly exercise it | enrichment — `robot/jetson/perception/family_observer.py:174` | Clothing can nevertheless authorize approach and arrival. |
| Clothing-based identity continuity | Mechanism implemented | One later live clothing-supported movement; narrow replay evidence | live result — `docs/DEVELOPMENT_LOG.md` (local development evidence) (source line 2618) | Lower-view identity loss ended that run; household accuracy unmeasured. |
| One camera coordinates people/floor views | Implemented across several layers | Head tests and integrated short movements | detour sequence — `robot/jetson/navigation/closed_loop_detour.py:373` | Person not rechecked inside the floor-view movement worker. |
| ChArUco camera calibration | Calibration implemented and applied selectively | Numerical fit plus manual visual acceptance | capture report — `docs/calibration/camera_calibration_capture_report_20260903.json:1` | Automatic acceptance was false; manual exception recorded. |
| DA V2 physical range estimation | Implemented estimates | Negative physical comparisons retained | distance discrepancy — `artifacts/mom-combined-right-20260912/operator-distance-check.json:2` | Metric accuracy unresolved; camera/front origins differ. |
| SegFormer candidate floor corridors | Heuristic corridor gate implemented | Synthetic tests and [fresh retained-image outputs](../evaluation/visual-scenarios/README.md) | corridor construction — `robot/jetson/navigation/image_corridors.py:27` | No calibrated footprint projection; water-label defect corrected in source on 2026-10-04, deployment pending. |
| Gemini assesses hazards but does not drive motors | Implemented | API/schema checks and route trials | deterministic fusion — `robot/jetson/navigation/navigation_reasoning.py:106` | Hazard accuracy and dynamic-scene coverage unmeasured. |
| Delayed distributed results are rejected | Many explicit bindings implemented | Tests and selected integration checks | route response validation — `robot/jetson/navigation/closed_loop_detour.py:314` | Not universal end-to-end expiry; clock assumptions remain. |
| EV3 independently stops on command loss | Implemented | Earlier physical watchdog/disconnect tests | watchdog — `robot/ev3/server/ev3_server.py:623` | Same-thread execution; historical tests do not validate every current path. |
| IR adds a local obstacle-stop layer | Not found in current path | Explicitly deferred | [hardware notes](<../docs/HARDWARE_NOTES.md>) (source line 71) | Desired architecture only. |
| Full autonomous arrival | Policies implemented | Retained trials are partial; project owner reports complete home demonstration | latest approach limitation — `docs/DEVELOPMENT_LOG.md` (local development evidence) (source line 2620) | Partial successes and heuristic stop labels are insufficient. |
| Approved request has a supervised robot delivery path | Implemented in current console and speech modules | Current-source review; no retained full physical delivery trial | [delivery contract](../docs/ROBOT_MESSAGE_DELIVERY.md); [gate](../robot/jetson/perception/speech_delivery.py), `delivery_gate` | Validated measured arrival required; gate permits face or clothing identity. `played` reports playback completion only. |
| Recipient receipt and acknowledgement | Software acknowledgement not implemented | Human acknowledgement reported in the home demonstration; recording not documented | [delivery contract](../docs/ROBOT_MESSAGE_DELIVERY.md); [scenario account](10_HOSPITAL_SCENARIO_AND_REQUEST_DELIVERY.md) | `played` alone cannot establish hearing, understanding, or response. |
| Complete home simulation of caregiver scenario | Reported by project owner | Reported on 2026-10-04: finding, approach, playback, human acknowledgement | [scenario evidence boundary](10_HOSPITAL_SCENARIO_AND_REQUEST_DELIVERY.md) | Trial count, timing, stopping-distance measurement, and recording remain undocumented; hospital evaluation is pending. |
| Hospital deployment and clinical benefit | Not established | None found in inspected evidence | [project delivery plan](<../docs/ASSISTIVE_COMMUNICATION_PLAN.md>) (source line 239) | Household trials cannot establish clinical performance. |

---

**E. Engineering decisions and design trade-offs**

These cases support an engineering narrative because they connect an observed challenge, a specific design decision, and bounded verification.

| Observed challenge | Design decision / change | Verification | Remaining limitation |
|---|---|---|---|
| Low calibration RMS but severe image warping | Unstable distortion fit and insufficient image support | Fixed `k3`; added valid-ROI, uncertainty, and coverage gates; live visual/timestamp checks | Final calibration still required a documented manual exception. |
| Camera looked at the ceiling despite a plausible encoder midpoint | Motor position did not imply optical direction | Use a verified useful lower view as forward; raise incrementally | Semantic useful views do not establish optical pitch. |
| Face pipeline lost matching image/detection pairs | Either callback could arrive first | Bounded two-sided exact-frame matcher; live source-stamp checks | DDS/source-age behavior still needs broader fault testing. |
| Dynamic-batch embedding ONNX failed to build | TensorRT profile missing | Fixed batch-one optimization profile; successful engine build | Hardware/version-specific engine reproducibility remains external. |
| Stop acknowledgement falsely appeared to erase head calibration | Bridge treated an acknowledgement as full telemetry | Fetch full status after stop | This repairs one concrete path, not all feedback-loss cases. |
| Face-away clothing test fractured tracks | Missed body detections removed spatial support; delayed result discarded during gap | Preserve limited same-view reassociation and original response deadline | Repeat retained a visible face; it was not clothing-only validation. |
| Gemini blocked regions outside the locally assessed near-floor corridor | Cloud/local polygon mismatch | Share exact horizon-trimmed polygons; later route and one-step run passed | Footprint and 3-D clearance remain heuristic. |
| Identity retries exhausted before lower inspection | Shared retry budget conflated different missing evidence | Separate identity, range-refresh, and lower-view budgets | Later lower-view identity loss still ended approach. |
| A good floor fit produced wrong range | Scene-dependent model geometry/scale | Added diagnostics, sparse replay, alternative anchoring experiment | No production accuracy correction accepted. |
| Known appearance lost to an unidentified competing track | Multiplication by 0.7 shrank an appearance-only margin below 0.08 | Normalize comparison-wide appearance scores when no candidate has spatial evidence | One recording improved; activation was pending in the retained deployment snapshot. |

Sources: calibration challenge, lines 918–950 — `docs/DEVELOPMENT_LOG.md` (local development evidence) (source line 918), face synchronization and preprocessing, lines 985–1021 — `docs/DEVELOPMENT_LOG.md` (local development evidence) (source line 985), stop-acknowledgement behavior, lines 2160 onward — `docs/DEVELOPMENT_LOG.md` (local development evidence) (source line 2160), camera midpoint challenge, lines 2321–2338 — `docs/DEVELOPMENT_LOG.md` (local development evidence) (source line 2321), and recent route/range/tracking iterations, lines 2538–2657 — `docs/DEVELOPMENT_LOG.md` (local development evidence) (source line 2538).

Documented alternatives should also remain bounded:

- YOLOX-tiny and YOLOX-s were both built and timed. The log records choosing the larger model because its throughput still exceeded the configured processing cap; published COCO figures were not measured on this project's data.
- Older depth experiments, including YOLO26n-depth, were explored before the current DA V2 path.
- SCRFD and image enhancement were investigated for difficult faces; the small diagnostic did not justify a deployment or recognition-accuracy claim.
- GeoCalib and inverse-depth anchoring remain diagnostics rather than an accepted replacement.
- The latest tracker work rejected an initially overbroad normalization because replay retention dropped to **163 frames**. The final narrower change reached **289**. This is a genuine local comparison, not an invented ablation.

The code supports plausible engineering explanations for these choices, but only the reasons explicitly recorded in the development log should be attributed to you.

---

**F. Evaluation inventory**

The retained results concern the household/room prototype, not hospital participants or emergency response. Current delivery source review adds implementation evidence only. The evaluation material is useful but heterogeneous. It contains component benchmarks, operator-supervised demonstrations, synthetic control tests, deployment checks, and one retained real recording. They should not be combined into one success percentage.

| Measurement / trial | Conditions and denominator | Result | Evidentiary strength |
|---|---|---|---|
| Camera throughput | Historical 640×480 MJPG setup | 27.3 fps in one brighter setup; roughly 16–18 fps in dimmer conditions | Capture performance, not a fixed operating rate. |
| YOLOX-s stage timing | Orin MAXN_SUPER; 640×480 source; sample count absent from table | 17.07 ms GPU; 23.65 ms total; 23.89 ms total p95 | Component benchmark. Managed service averaged 28.4–28.7 ms. |
| Person detection | Operator-held poses, normal indoor lighting | Near 277/277; medium 297/297; far 223/223; partial 290/290; two-person 295/295 | Adjacent-frame component trials, not independent participant trials. |
| Empty-room detection | 90 seconds | 0 false positives / 1,304 frames | One controlled negative scene. |
| Face detection | Ten-second stationary window | 50/50 processed views with a face | Small pipeline acceptance check. |
| Embedding latency | 30 runs | Mean 14.37 ms; p95 23 ms; max 26 ms | Embedding stage only. |
| Enrolled-person recognition | One 15-second stationary window | 102/102 target-match messages contained target | One person/view; correlated frames. |
| Different-person rejection | Stable face visible; 15 seconds | 119 correlated recognition frames, zero target matches | One negative identity under the then-current 0.45 threshold. |
| Odometry calibration | Short tape/angle measurements and encoder captures | Final effective radius/width derived from measured travel and turns | Useful initial calibration; no broad surface/slip characterization. |
| Initial bounded search | Seated enrolled target, tethered | Two measured ~12° right turns; target at +24.46° | Search demonstrated; no approach because old image-size threshold already passed. |
| Older multistep approach | One integrated trial | Three short movements/reacquisitions, then route check limit; 91.14 s | Partial integrated capability. |
| September 12 one-step approach | Selected target present | −39.82° turn, 6.39 cm encoder travel, face reacquisition; 68.21 s | Physical primitive plus reacquisition, not arrival. |
| Later clothing-led approach | Saved appearance, approximate range | 6.77 cm encoder travel; lower-view identity loss; 88.62 s | Physical clothing-supported movement with recorded outcome. |
| Warm DA V2 timing | One 480×640 warm call | 107.6 ms | Excludes segmentation/network; no range accuracy. |
| Cold range service | Synthetic empty frame | Approximately 18.08 s | Exceeded observer timeout; startup bottleneck evidence. |
| Real route analysis | Selected stopped scene | One example: local 122.9 ms, Gemini 2.595 s | One request, not latency distribution or hazard accuracy. |
| Recorded tracker comparison | Same 297 evaluable frame/detection pairs; one identity anchor | Baseline 220 retained; final change 289; eight pre-anchor frames | Real recording, offline continuity comparison; no independent recognition truth. |

Sources for component figures: person timing/detection trials, lines 706–782 — `docs/DEVELOPMENT_LOG.md` (local development evidence) (source line 706), recognition acceptance, lines 1103–1153 — `docs/DEVELOPMENT_LOG.md` (local development evidence) (source line 1103), cold/warm service limitations — `artifacts/unattended-checks-20260909/README.md:3`, and tracker comparison validation — `artifacts/recorded-mom-live-20260912-01/appearance-margin-validation.json:1`.

The tracker comparison is the clearest retained baseline experiment:

- Recording: **408 frames**, **6,882 events**, approximately **25.8 MB**, over **89.8 seconds** including setup.
- Evaluation: **297 matched/evaluable frame–detection pairs**; unmatched frames were excluded rather than counted as tracking failures.
- Baseline and initial partial-view code both retained identity on **220** evaluable frames.
- Final score-normalization change retained identity on **289/297**.
- All eight remaining frames preceded the single identity anchor.
- No camera transition occurred; all recorded head positions remained zero.
- No new cloud calls occurred during comparison.
- Retained deployment metadata in the September 12 artifact set records **`live_restarted: false`**; activation was pending in that snapshot.

See recording summary — `artifacts/recorded-mom-live-20260912-01/summary.json:1` and deployment state — `artifacts/recorded-mom-live-20260912-01/appearance-margin-deployment.json:1`.

This comparison supports improved continuity on one recorded case. It does not establish fewer identity mistakes across people, general clothing accuracy, or a repaired physical approach.

Tests **exist** for exact synchronization, duplicate frames, profile revision changes, delayed cloud results, motion-epoch invalidation, partial clothing ambiguity, head cancellation, client serialization, route binding, stale feedback, approximate range, mission handoff, and recording/replay.

Tests also have **recorded execution results**:

- 961 tests: navigation build log, lines 68–70 — `artifacts/navigation-20260909/offline-build.log:68`.
- 989 tests: family build log, lines 68–70 — `artifacts/family-memory-20260909/offline-tests.log:68`.
- 1,054 tests: range-diagnostic build log, lines 77–79 — `artifacts/range-diagnostic-capture-20260912/build.log:77`.
- 144 focused tests: latest appearance-margin validation metadata.
- ROS recording smoke: 18 generated frames recorded/replayed and 18 new synthetic commands logged, with recorded commands not republished. Smoke result — `artifacts/mission-recording-tools-20260912/smoke-result.json:1`.

These are historical software results against their respective versions. They are not a fresh pass of the entire current working tree, and the generated ROS smoke did not use a webcam or physical EV3.

**Proposed measurements, not completed work:**

1. **Correct-recipient trials:** separate people, views, lighting, clothing similarity, crossings, absent-target cases, and held-out sessions; report false confirmation, missed identification, and identity-switch rates.
2. **Range and arrival:** same-frame ground truth with explicitly measured lens/front offset and stopping gap; separate seated/standing/partial-body cases; retain outcome distributions.
3. **Shared-camera transitions:** recorded person→floor→person cycles, target movement during the blind interval, and post-segment reacquisition.
4. **Current-version stopping:** independently measured stop delay and travel after command loss, disconnect, delayed commands, process termination, head faults, and low battery.
5. **End-to-end missions:** fixed scenario definitions and denominator; success only when the intended endpoint is actually reached; record interventions, duration, partial completion, and reason for stopping.
6. **Delivery and acknowledgement:** test the implemented approved-message → selected-profile → supervised mission → gated playback path with measured arrival and an explicit trial denominator. Record human-observed receipt and acknowledgement separately from playback completion; an automated acknowledgement workflow is future work.
7. **Hospital scenario:** begin with staged, supervised caregiver-role trials and deliberately similar clothing, competing people, occupied recipients, and absent targets. These are proposed tests; the household results do not measure patient outcomes or response-time benefit.

---

**G. Engineering contributions**

The hospital scenario makes the coordination problem concrete: reach the selected caregiver, obtain usable identity evidence, inspect the route, and decide whether movement or playback is justified. The principal engineering contribution is **coordinating evidence and control across changing camera roles and delayed computation**. It consists of several concrete mechanisms:

1. **Separating identity from current spatial evidence.** Face anchors and durable outfit ownership can survive longer than image coordinates and range. Motion invalidates the latter without automatically erasing all identity history. This directly addresses reacquisition after camera movement.

2. **Binding delayed results to the state that produced them.** Frame keys, image hashes, profile revisions, track IDs, motion epochs, motor generations, head references, and post-response feedback prevent many classes of cross-frame or changed-state result reuse.

3. **A bounded shared-camera control procedure.** Camera leases, motor interlocks, cancellation-aware command ownership, useful-view restoration, floor inspection, short movement, and reacquisition create an executable solution to having one sensor serve incompatible viewing roles.

4. **Motor and feedback integration.** Stop acknowledgements versus telemetry, head backdrive/settling, queued-command timing, source-message arrival order, and physical encoder references required custom handling beyond connecting model APIs.

5. **Recording and replay diagnostics.** The latest recording tools and baseline comparison make a concrete issue reproducible, while keeping encoder travel, model estimates, generated tests, and physical truth distinct.

These contributions concern systems engineering: the integration and coordination of existing perception models, state validation, actuation, and diagnostic tools.

The central design question is **which evidence remains usable after time, motion, viewpoint changes, and asynchronous work**, and how the robot responds when observations are insufficient.

---

**H. Coverage, contradictions, and unresolved evidence**

The investigation used three passes:

| Pass | Material inspected | Coverage boundary |
|---|---|---|
| Repository and history | Branch/commit/status, tracked history, robotics/config/test/script inventory, current and historical documentation | Current working tree distinguished from committed source. |
| Executable paths | Browser mission entry, mission subprocesses, detection/recognition/family tracking, Mac route/range/cloud adapters, camera/bridge/EV3 services | Main robotics chain traced; no runtime processes activated. |
| Evidence reconciliation | Development log, selected deployment/build logs, calibration report, mission reports, range discrepancies, recording/replay summaries | Selected artifacts read in detail; not every historical JSON/log or archived dataset exhaustively audited. |

The inventory included 100 Python files under `robot`, 19 Python scripts, and 90 Python files under `tests`. Counts are file counts, not test-case counts. The much larger `Archive` and communication environment were not exhaustively reviewed; communication entry points, confirmation/audio behavior, scope statements, and robot-linkage searches were inspected.

Two local cached model configurations were read to verify SegFormer labels/preprocessing and the DA V2 model type. Model inference and weight execution were not performed. Remote Jetson/EV3 runtime files, current services, private enrollment databases, and external physical recordings were not independently inspected as live state.

The original investigation recorded the following contradictions and chronology issues. Some surrounding README wording has since been revised; these describe the inspected historical versions:

- README’s three-of-five recognition versus current two-of-five configuration.
- Documentation describing clothing as observation-only versus current enabled clothing approach.
- Current family identity semantics versus older “fresh face required” comments.
- README’s “recording not added” versus implemented recorder and retained recording.
- `navigation.yaml`’s standoff/sweep values versus the active hard-coded/CLI policies.
- Historical camera coordinates versus reference-specific runtime limits.
- “Footprint-wide corridor” language versus fixed image trapezoids.
- Calibration YAML in use despite the retained report’s automatic rejection.
- Historical Gemini model names versus current `gemini-3.5-flash-lite`.
- Installed tracker updates versus activation pending in the retained deployment snapshot.
- Early “enrollment never commands motors” statements versus the later combined console containing manual drive, mission, and stop controls.

Unresolved technical evidence includes current deployed revision/configuration, physical motor identity, camera extrinsics, independent optical tilt, stopping-distance accuracy, low-battery behavior, broad identity/confusion performance, dynamic obstacles during the person-view gap, and complete current-version fault-stop timing.

Evaluation to date concerns supervised household/room trials with a small set of enrolled people. The current implementation includes supervised robot playback; the reported complete home demonstration includes human acknowledgement. Hospital participants, patient outcomes, and care-coordination benefit have not been evaluated. The evaluation section distinguishes the reported home demonstration from retained measurements.

Further evaluation records should capture:

1. A recording, trial count, duration, independently measured stopping distance, and human-observed receipt and acknowledgement for complete delivery trials.
2. The deployed revision, configuration, calibration, participant conditions, and room setup used for each trial.
