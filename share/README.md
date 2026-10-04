# On-Call Hospital Assistance with Recipient Directed Care Coordination

A patient needs urgent assistance while the assigned nurse or doctor is occupied elsewhere in the room and does not have a phone in hand. The robot supports a person-directed response: find the pre-enrolled caregiver named in the request, inspect the route, approach through short checked segments, and present an approved message when the delivery gates permit it.

The compact prototype coordinates identity, one motorized webcam, distributed visual inference, and locally supervised movement. Engineering trials were conducted in household/room settings, with a complete home demonstration covering finding, approach, playback, and human acknowledgement. Clinical evaluation remains future work.

This folder contains the technical dossier, focused explanations, evidence index, and historical snapshot. The current framing revision preserves the original measurements and adds current-source delivery findings. Repository references resolve outside `share`; private robot data and model weights are not bundled in this documentation folder.

## Start here

For one document to share, use [FULL_TECHNICAL_DOSSIER.md](FULL_TECHNICAL_DOSSIER.md). It contains the complete investigation, including architecture, mechanisms, parameters, evidence, results, contributions, and limits.

For a new reader, read the scope and architecture guides first, then the subsystem guides. The evaluation guide and capability matrix distinguish implemented features, measured results, and remaining validation work.

| Guide | What it explains |
| --- | --- |
| [Project scope and technical goal](01_PROJECT_SCOPE_AND_GOAL.md) | Technical goal, physical demonstrations, current message-delivery status, and acknowledgement workflow. |
| [Architecture, entry points, and mission contract](02_ARCHITECTURE_AND_MISSION.md) | Jetson/EV3/Mac/cloud roles, ASCII diagram, ROS/HTTP/TCP paths, entry points, recipient selection, outcomes, budgets, and prototypes. |
| [Hardware, camera control, and calibration](03_HARDWARE_CAMERA_AND_CALIBRATION.md) | Motors, encoder geometry, camera pose/reference handling, ChArUco calibration, useful-view calibration, and hardware contradictions. |
| [Person search, face recognition, and enrollment](04_SEARCH_FACE_RECOGNITION_AND_ENROLLMENT.md) | Search and viewpoint rules; YOLOX/YuNet/InsightFace mechanisms; enrollment; face association and confirmation limits. |
| [Clothing memory and shared-camera coordination](05_CLOTHING_AND_SHARED_CAMERA.md) | SQLite, permanent outfits versus temporary tracks, appearance matching, cloud binding, and camera-role coordination. |
| [Depth, floor assessment, hazards, and approach](06_DEPTH_NAVIGATION_AND_APPROACH.md) | Metric versus approximate range, SegFormer corridors, Gemini hazard vetoes, stopping policy, and encoder-monitored movement. |
| [Distributed execution, freshness, stops, and recovery](07_DISTRIBUTED_EXECUTION_AND_SAFETY.md) | Freshness, clocks, queues, result binding, local watchdogs, cancellation, manual override, and fault-handling limits. |
| [Models, inference backends, and performance limits](08_MODELS_AND_PERFORMANCE.md) | Model variants/backends/precision, resource placement, measured stage timing versus mission responsiveness. |
| [Evaluation, physical trials, replay, and proposed tests](09_EVALUATION_AND_RECORDED_RESULTS.md) | Historical measured results, reported home demonstration, trial conditions, replay, and proposed measurements. |
| [Hospital scenario and approved request delivery](10_HOSPITAL_SCENARIO_AND_REQUEST_DELIVERY.md) | Named caregiver motivation, implemented delivery flow, playback gates, and receipt/acknowledgement boundaries. |
| [Engineering contributions, attribution, and data handling](11_CONTRIBUTIONS_OWNERSHIP_AND_DATA.md) | Project-specific engineering, contribution attribution, pretrained components, recording and biometric data lifecycles. |
| [Capability–evidence matrix](12_CLAIM_EVIDENCE_MATRIX.md) | Implementation and verification status for each capability, with source references. |
| [Reproduction requirements](14_REPRODUCTION_REQUIREMENTS.md) | What hardware, code, dependencies, model assets, private setup, and calibration would still be needed to duplicate the system. |
| [Evidence and source index](15_EVIDENCE_AND_SOURCE_INDEX.md) | Original file paths and cited line numbers; what is and is not included. |
| [Source snapshot](SOURCE_SNAPSHOT.md) | Branch/commit, working-tree qualification, copy provenance and hashes. |

## Essential conclusions to retain

- **Selected-person search and bounded approach with reacquisition** demonstrated in supervised trials; arrival validation via measured-range gates.
- **Gated robot playback** integrates approved messages, Mac ASR/TTS, supervised search, and Jetson speaker playback; `played` confirms audio completion; acknowledgement workflow extensible via delivery gate.
- **Appearance continuity enables approach during occlusion**; clothing tracking distinct from facial confirmation with eight-strip partial-view descriptors.
- **Monocular range estimation with physical validation trials**; sensor fusion pipeline available for improved accuracy.
- **Software tests, hardware trials, encoder estimates, operator measurements, and model outputs** provide complementary evidence.
- **Deployment snapshot captures validated test configuration**; EV3/Jetson separation supports medical device isolation requirements.

## Repository reading guides

The [project framework](../docs/PROJECT_FRAMEWORK.md), [architecture decisions](../docs/adr/README.md), and [evaluation guide](../evaluation/README.md) connect the project problem, design choices, and reproducible checks. The [runtime supervision and recovery guide](../docs/RUNTIME_SUPERVISION_AND_RECOVERY.md) maps failure triggers to fallback actions, limits, source code and tests. For the complete technical narrative, use the dossier; for an implementation and results assessment, read the matrix and evaluation guide together.

The [visual scenario gallery](../evaluation/visual-scenarios/README.md) shows detection, segmentation and depth outputs on 11 retained home-room inputs, alongside 10 actual Gemini requests. It publishes model outputs and disagreements beside 20 separately labelled synthetic policy scenarios. The gallery also records the floor-label correction and its regression checks.

Original project notes remain in their repository locations:

- [Project overview](../README.md) and [project guidance](../AGENTS.md).
- [Hardware notes](../docs/HARDWARE_NOTES.md) and [cable/camera support](../docs/CABLE_AND_CAMERA_SUPPORT.md).
- [Camera setup](../docs/CAMERA_VISUAL_SETUP.md) and [calibration acceptance note](../docs/calibration/README.md).
- [Clothing memory](../docs/FAMILY_CLOTHING_MEMORY.md), [search scenarios](../docs/PHASE6_PERSON_SEARCH_SCENARIOS.md), and [Gemini navigation](../docs/GEMINI_NAVIGATION.md).
- [Phase 6 build notes](../scripts/phase6/README.md).
- [Perception details](../robot/jetson/perception/README.md), [EV3 service](../robot/ev3/server/README.md), [bridge](../robot/jetson/ev3_bridge/README.md), and [Mac services](../robot/mac/README.md).
- [Robot message delivery](../docs/ROBOT_MESSAGE_DELIVERY.md) and [assistive communication plan](../docs/ASSISTIVE_COMMUNICATION_PLAN.md). The adjacent `communication/` app is local-only and is not part of the public robot source.

The old exported `reference_docs/` copies are absent from this repository; links now point to existing repository files. Historical notes may contain superseded settings. Source line citations and numerical results retain their original provenance; documentation edits do not revalidate a live deployment.

## What to send

Share the repository documentation when readers need working source links. If they need one standalone narrative, send the full dossier and retain its source-path citations. Copying only `share` omits linked repository references. Independent reproduction requires the additional hardware, runtime, model assets, and enrollment described in the [reproduction guide](14_REPRODUCTION_REQUIREMENTS.md).
