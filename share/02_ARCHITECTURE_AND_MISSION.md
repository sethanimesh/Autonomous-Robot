# Architecture, entry points, and mission contract

This guide presents the implementation and evaluation status. Historical measurements retain their original provenance; see the [full dossier](FULL_TECHNICAL_DOSSIER.md) and [source snapshot](SOURCE_SNAPSHOT.md).

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

The browser maps several distinct outcomes to a broad `found` state, but retains explanatory messages and the underlying outcome. Interview and write-up claims should use that underlying outcome. See result mapping, lines 22–107 — `robot/jetson/perception/mission_control.py:22`.

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
- Current console/speech modules implement the supervised delivery path. The adjacent communication app and archived speech projects are separate workstreams; the evaluation guide records the home demonstration and acknowledgement workflow.
- Recording is now implemented despite the root README still saying recording has not been added.

Thus “the repository contains it” is insufficient evidence of active integration.

[Back to reading guide](README.md)
