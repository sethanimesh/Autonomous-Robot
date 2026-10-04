# Echora Robo — Markdown explanation pack

This folder accompanies the repository-grounded technical dossier. It contains **Markdown only**: a complete report, focused explanations, an evidence index, and original project notes. It does not contain a runnable software copy or private robot data.

## Start here

For one document to share, use [FULL_TECHNICAL_DOSSIER.md](FULL_TECHNICAL_DOSSIER.md). It contains the complete investigation, including architecture, mechanisms, parameters, claims, evidence, results, contributions, and limits.

For a new reader, read the scope and architecture guides first, then the subsystem guides. Read the evaluation and claim matrix before turning any implementation detail into a capability claim.

| Guide | What it explains |
| --- | --- |
| [Project scope and technical goal](01_PROJECT_SCOPE_AND_GOAL.md) | Actual technical goal; implemented endpoint; physical demonstrations; the gap to assistance delivery and acknowledgement. |
| [Architecture, entry points, and mission contract](02_ARCHITECTURE_AND_MISSION.md) | Jetson/EV3/Mac/cloud roles, ASCII diagram, ROS/HTTP/TCP paths, entry points, recipient selection, outcomes, budgets, and prototypes. |
| [Hardware, camera control, and calibration](03_HARDWARE_CAMERA_AND_CALIBRATION.md) | Motors, encoder geometry, camera pose/reference handling, ChArUco calibration, useful-view calibration, and hardware contradictions. |
| [Person search, face recognition, and enrollment](04_SEARCH_FACE_RECOGNITION_AND_ENROLLMENT.md) | Search and viewpoint rules; YOLOX/YuNet/InsightFace mechanisms; enrollment; face association and confirmation limits. |
| [Clothing memory and shared-camera coordination](05_CLOTHING_AND_SHARED_CAMERA.md) | SQLite, permanent outfits versus temporary tracks, appearance matching, cloud binding, and camera-role coordination. |
| [Depth, floor assessment, hazards, and approach](06_DEPTH_NAVIGATION_AND_APPROACH.md) | Metric versus approximate range, SegFormer corridors, Gemini hazard vetoes, stopping policy, and encoder-monitored movement. |
| [Distributed execution, freshness, stops, and recovery](07_DISTRIBUTED_EXECUTION_AND_SAFETY.md) | Freshness, clocks, queues, result binding, local watchdogs, cancellation, manual override, and fault-handling limits. |
| [Models, inference backends, and performance limits](08_MODELS_AND_PERFORMANCE.md) | Model variants/backends/precision, resource placement, measured stage timing versus mission responsiveness. |
| [Evaluation, physical trials, replay, and proposed tests](09_EVALUATION_AND_RECORDED_RESULTS.md) | Existing tests, recorded executions, physical runs, denominators, negative results, baseline replay, and proposed future measurements. |
| [Contribution candidates, ownership, and data handling](11_CONTRIBUTIONS_OWNERSHIP_AND_DATA.md) | Supported engineering contributions, pretrained components, ownership limits, recording and biometric data lifecycles. |
| [Claim–evidence matrix](12_CLAIM_EVIDENCE_MATRIX.md) | Claim-by-claim implementation and verification status with exact source references. |
| [Coverage, contradictions, limitations, and open questions](13_COVERAGE_LIMITATIONS_AND_OPEN_QUESTIONS.md) | Inspection scope, stale documentation, contradictions, missing evidence, and two questions requiring human testimony. |
| [Reproduction requirements](14_REPRODUCTION_REQUIREMENTS.md) | What hardware, code, dependencies, model assets, private setup, and calibration would still be needed to duplicate the system. |
| [Evidence and source index](15_EVIDENCE_AND_SOURCE_INDEX.md) | Original file paths and cited line numbers; what is and is not included. |
| [Source snapshot](SOURCE_SNAPSHOT.md) | Branch/commit, working-tree qualification, copy provenance and hashes. |

## Essential conclusions to retain

- The implemented robotics endpoint is selected-person search and bounded approach. Full, reliable physical arrival is not demonstrated by the inspected evidence.
- The communication app is separate. Robot delivery of a confirmed assistance message and caregiver acknowledgement are not connected in the inspected path.
- Clothing is labelled separately from a fresh face confirmation, but current family mode can authorize approach using clothing/tracking identity.
- Distance estimates are explicitly approximate and have substantial recorded discrepancies. The floor-corridor check is an image heuristic, not proven three-dimensional clearance.
- Software tests, hardware trials, encoder estimates, operator measurements, and model outputs are different kinds of evidence.
- The inspected working tree includes newer uncommitted files; the last commit does not reproduce it. Some latest installed changes were still awaiting service activation.

## Original reference documents

The `reference_docs/` tree contains copies of 24 original Markdown documents, retaining their repository-relative layout. These preserve project history and the author's existing explanations. Particularly useful:

- [Project overview](reference_docs/README.md) and [project guidance](reference_docs/AGENTS.md).
- [Hardware notes](reference_docs/docs/HARDWARE_NOTES.md) and [cable/camera support](reference_docs/docs/CABLE_AND_CAMERA_SUPPORT.md).
- [Camera setup](reference_docs/docs/CAMERA_VISUAL_SETUP.md) and [calibration acceptance note](reference_docs/docs/calibration/README.md).
- [Family clothing memory](reference_docs/docs/FAMILY_CLOTHING_MEMORY.md), [search scenarios](reference_docs/docs/PHASE6_PERSON_SEARCH_SCENARIOS.md), and [Gemini navigation](reference_docs/docs/GEMINI_NAVIGATION.md).
- [Phase 6 build notes](reference_docs/scripts/phase6/README.md).
- [Perception details](reference_docs/robot/jetson/perception/README.md), [EV3 service](reference_docs/robot/ev3/server/README.md), [bridge](reference_docs/robot/jetson/ev3_bridge/README.md), and [Mac services](reference_docs/robot/mac/README.md).
- [Assistive communication plan](reference_docs/docs/ASSISTIVE_COMMUNICATION_PLAN.md) and [communication app status](reference_docs/communication/README.md).

Original notes contain historical and superseded statements. They should not override the dossier's current-source findings without checking dates and execution paths. The explanations and source index are portable; original-reference links to omitted non-Markdown assets may not resolve in this folder.

## What to send

Send this entire `share` folder when the recipient needs the complete explanation and historical context. If they need only one document, send the full dossier. Source code, model weights, private enrollment/outfit records, raw logs and physical recordings are not required to read these explanations and are not included. Independently reproducing the software or experimental results requires the additional material described in the reproduction guide.
