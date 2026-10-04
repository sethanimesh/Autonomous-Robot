# Evaluation and reproduction

The project's application is **On-Call Hospital Assistance with Recipient Directed Care Coordination**: find the pre-enrolled caregiver named in a request, inspect the route, approach in bounded segments, and bring the request to that person's attention. Evaluation distinguishes identifying the recipient, reaching them, playing a message, and obtaining acknowledgement as separate outcomes.

## Reproduce automated checks

From the repository root:

```sh
python3 scripts/diagnostics/reproduce_checks.py
```

Use Python 3.11 or newer. This selected suite requires only the standard library: no package installation, ROS, EV3, webcam, enrolled faces, model weights, cloud credentials, or dataset download. Keep the EV3 off. The runner refuses network connections, service binding, child-process launches, and `/dev` or `/sys` access. Tests use fake motors/sockets/clocks, mocked inference, generated observations, and the synthetic [cloud-view fixture](../tests/fixtures/cloud_view_calibration.json). Calibration checks generate their source-image hashes from test bytes; they do not read private recordings or local cloud-response artifacts. Temporary files are cleaned by the tests; results print to the terminal. There is no random seed or model inference in this suite.

`--help` explains the options; `--list` lists the selected modules; `--verbose` prints individual checks. Missing imports/fixtures, failures, errors, skips, and expected failures produce a nonzero exit status. CI runs the same command on Ubuntu with Python 3.11 in [robotics-checks.yml](../.github/workflows/robotics-checks.yml).

| Contract | Existing check modules | What is exercised |
| --- | --- | --- |
| Recipient identity and continuity | `test_recognition_core`, `test_person_continuity`, `test_target_gate` | Independent face observations; body appearance cannot create face evidence; ambiguous bodies, expired identity, duplicate/stale frames, and missing current geometry |
| Returned inference belongs to the current scene | `test_cloud_view_calibration`, `test_detour_route_binding` | Image hashes/order, camera reference and position, changed chassis pose, telemetry gaps, response age, mocked delayed/cloud responses |
| Route vetoes and bounded mission actions | `test_navigation_reasoning`, `test_single_room_mission`, `test_mission_control` | Local evidence overrides cloud preference; uncertain/blocked routes stop or rescan; short approach steps require fresh observations; injected mission cancellation |
| Serialized control and local stopping | `test_ev3_client_serialization`, `test_head_command_worker`, `test_ev3_server` | Response ownership, telemetry during camera moves, cancelled command fencing, local watchdog timeout, motor-write failure, camera limits and settlement |

Verification on **2026-10-04**, macOS, Python **3.14.7**: **138 tests passed, 0 skipped, 0 failures, 0 errors in 0.174 s** from a Git-index-only export with Python site packages disabled (`-S`), without the private calibration artifact. Timing varies by machine. The CI definition targets Python 3.11; no hosted CI execution is claimed here. These checks verify software contracts under their fixtures, not detector accuracy, physical clearance, metric stopping accuracy, or completed clinical assistance.

The repository has additional perception, range, camera, and speech tests outside this selection. In particular, [speech delivery tests](../tests/test_speech_delivery.py) use pytest, and [voice API tests](../tests/test_robot_voice_delivery_api.py) use FastAPI's test client. The canonical command does not run these plain pytest functions. Speech preparation, recipient/message revision binding, expiry, and stopped-state delivery gates have corresponding checks there; the selected suite makes no claim to cover audible delivery or human acknowledgement. Production clothing-descriptor extraction and model inference are also outside this suite.

## Retained-image model evaluation

The [visual scenario gallery](visual-scenarios/README.md) publishes fresh CPU outputs from YOLOX-s, YuNet, SegFormer-B0 and Depth Anything V2 on 11 original home-room images, plus 10 actual Gemini requests spanning camera framing, route hazards, paired views and a clothing description. Model/configuration hashes, numerical arrays, parsed responses and visible disagreements are included. The retained requests preserve their original input bindings; two local-model views have no published framing response. The selected images are qualitative diagnostic examples without independent model-accuracy or physical-clearance labels.

Two additional commands reproduce 20 authored policy scenarios and 5 floor-label regression tests without models, cloud credentials or hardware:

```sh
python3 scripts/diagnostics/check_visual_scenarios.py
python3 -m unittest tests.test_floor_label_mapping
```

Both passed on 2026-10-04 with Python 3.14.7. CI includes these alongside the selected regression suite. Scenario inputs are explicitly synthetic and are not pixel-derived measurements; image hashes establish which retained input each fixture accompanies. The gallery documents separate model and cloud reproduction commands.

The [appearance diagnostics gallery](appearance-diagnostics/README.md) adds whole-frame HSV distributions, exact production torso/clothing/strip descriptors on saved person detections, and 162 controlled synthetic image variations. Score matrices and response curves expose colour, lighting and crop sensitivity. Policy diagrams retain separate authored-input provenance; the original images retain their test-capture provenance. The reusable generation tools are maintained locally under a Git-ignored directory.

## Historical component results

These retained records predate the hospital presentation and use household participant labels. They provide component evidence in the recorded conditions. The raw artifacts listed below are retained locally; their paths identify the evidence without implying that those files are included in the public repository.

| Retained evidence | Result and boundary |
| --- | --- |
| Offline BoT-SORT comparison, 2026-09-27 — `artifacts/external-tracker-baseline-20260927/README.md` and `summary.json` | Both BoT-SORT and the custom tracker output the anchored track on 289/289 eligible post-anchor frames from one stationary recording. No independent identity labels or completed camera transition: this measures track availability, not correct identity, identity switches, or superiority. |
| Family-memory build, 2026-09-09 — `artifacts/family-memory-20260909/README.md` | Recorded-image plumbing and ambiguity retests, plus model-load/warm-up records. The report explicitly separates these from physical identification and stopping-distance acceptance. |

## Supervised physical evidence and reported demonstration

| Evidence | What can be concluded |
| --- | --- |
| Short movement log, 2026-09-05 — `docs/perception/short_movement_live_20260905/short-movement-live.json` (local artifact) | One logged route check, stopped turn, renewed route check, and stopped short drive. The reported 0.049 m travel is encoder-derived, not an independent clearance or distance measurement. |
| Live wardrobe checks, 2026-09-09 — `artifacts/mom-wardrobe-live-20260909/README.md` (local artifact) | The initial face-away check had 18 unconfirmed samples. A later stable-track repeat still contained confirmed faces; face-independent clothing recognition remained provisional in that report. |
| Home caregiver demonstration reported by the project owner on 2026-10-04 | Recipient finding, approach, message playback and human acknowledgement were completed. Quantitative records and a recording remain to be documented. Acknowledgement was human-observed; a software acknowledgement workflow and hospital evaluation remain future work. |

Partial historical runs and the reported completed home scenario are different evidence sources. A shareable demonstration should link a dated run manifest and trace, state whether recipient identity, physical approach, playback, and acknowledgement each occurred, and preserve failed/interrupted trials alongside successful ones.

## Proposed recipient-directed hospital benchmark

**This protocol is proposed and has not been executed as a hospital evaluation.** Use a prepared simulated ward with consenting participants acting as patients/caregivers, pseudonymous enrolled identities, an operator stop, measured positions, and inert requests. Record a run ID, source commit, configuration/model hashes, enrollment revision, lighting, participant positions, camera calibration/reference, injected fault, and request/mission timestamps. Ground-truth recipient identity, route obstruction, and acknowledgement must be annotated independently of the robot output.

| Scenario | Observable acceptance condition | Measurements |
| --- | --- | --- |
| Named caregiver with a closer distractor | The requested enrolled caregiver is selected; the nearer person does not receive a substituted alert. Include target-absent trials. | Correct-recipient delivery, wrong-recipient attempts/deliveries, no-recipient outcome, search and approach times |
| Confirmed target turns away; another person has similar clothing | Continuity is traceable to the prior face-confirmed identity; ambiguous clothing produces reacquisition or intervention. Clothing updates do not become new face confirmation. | Identity switches, false confirmations, continuity duration, reacquisition and intervention rates |
| Delayed remote result after a turn, camera move, or new frame | An answer bound to an old image, camera reference, pose, or deadline does not authorize movement. | Stale-result acceptance, motor commands after rejection, feedback gaps, decision latency |
| EV3 command refresh stops or control connection disconnects | The local watchdog stops the tracks without needing a network stop message; recovery requires fresh state. | Independent time from last valid refresh to stopped tracks, stopping distance, unintended commands after recovery |
| Cable, chair leg, person, dark/obscured floor, or blocked corridor | The robot stops, rechecks, or requests intervention; each route-changing turn and movement segment has renewed evidence. | Hazard misses, contacts, minimum measured clearance, segment lengths, recovery count and completion |
| Request expires, recipient/profile changes, or message is edited/cancelled | Expired or superseded requests cannot cause later movement or playback. Prepared-audio expiry already exists; end-to-end request lifecycle needs its own trace and evaluation. | Actions after expiry/cancel, wrong-revision playback, cancellation latency |
| Message plays but acknowledgement is absent or delayed | Playback completion, recipient awareness, explicit acknowledgement, timeout, and escalation are separate recorded events. A play event alone does not count as acknowledged assistance. | Playback completion, correctly attributed acknowledgement, time to acknowledgement, timeout/escalation outcomes |

Fix a trial matrix before collecting results: vary recipient/distractor placement, seated/standing pose, illumination, face visibility, similar clothing, obstruction, delay, and fault phase. Counterbalance order and give every compared system the same scenarios. Proposed comparisons are a general audible callout, a stationary device notification, nearest-person robot delivery, and the recipient-directed robot. For attention comparisons, specify device availability and task load explicitly; the initial problem assumes the caregiver is occupied and has no phone in hand. No advantage over these baselines is currently claimed.

Define success as **the intended recipient explicitly acknowledging a current request within a predeclared deadline**, with approach/clearance and wrong-recipient outcomes reported separately. Report numerators/denominators, missing/aborted trials, latency distributions, and uncertainty intervals. Do not turn a successful stop or abstention into a delivery success. Select repetition counts and thresholds before testing rather than assigning achieved values from the home demonstration.

The protocol evaluates coordination in a simulated scenario. The current camera estimates and route heuristics do not establish complete three-dimensional clearance, suitability for emergency deployment, clinical effectiveness, or improved patient outcomes. Hospital use would require separate validation beyond the evidence retained in this repository.
