# Claim–evidence matrix

These findings explain the inspected working tree, not a freshly verified live robot. They distinguish implementation, recorded software checks, physical demonstrations, and unverified assumptions. Refer to the [full dossier](FULL_TECHNICAL_DOSSIER.md) for the complete argument and the [snapshot](SOURCE_SNAPSHOT.md) for provenance. Source paths refer to the original repository; code and raw data are not bundled here.

**D. Claim–evidence matrix**

“Recorded” below means a saved report or documented execution result, not a test run during this investigation.

| Claim | Implementation status | Verification status | Exact evidence | Limitation / missing proof |
|---|---|---|---|---|
| Searches for a specified enrolled recipient | Implemented and reachable from browser | Recorded selected-person searches | console request binding — `robot/jetson/perception/enrollment_console.py:1551`; mission scan binding — `robot/jetson/mission/autonomous_find.py:240` | No patient-to-caregiver assignment mechanism. |
| YOLOX-s → YuNet → InsightFace | Implemented and service-configured | Historical stationary component trials | configuration — `config/perception.yaml:20` | Full-frame YuNet fallback means person boxes are not an absolute prerequisite. |
| TensorRT FP16 on Jetson | Implemented for three perception stages | Recorded engine inspection and timings | [build/precision evidence](<reference_docs/docs/DEVELOPMENT_LOG.md>) (source line 1064) | Mixed precision; Mac/cloud stages use other backends. |
| Repeated face observations confirm identity | Implemented | Unit tests and small physical trials | family matcher — `robot/jetson/perception/family_faces.py:12` | Nonconsecutive replay not fully prevented upstream; no broad confusion benchmark. |
| SQLite profiles and appearance memory | Implemented and integrated | Migration/integration/outfit-learning reports | schema — `robot/jetson/perception/family_store.py:13` | No outfit TTL; old clothes survive re-enrollment. |
| Clothing is not fresh face confirmation | Correct field distinction | Tests explicitly exercise it | enrichment — `robot/jetson/perception/family_observer.py:174` | Clothing can nevertheless authorize approach and arrival. |
| Clothing-only continuity is reliable | Mechanism implemented | One later live clothing-supported movement; narrow replay evidence | [live result](<reference_docs/docs/DEVELOPMENT_LOG.md>) (source line 2618) | Lower-view identity loss ended that run; household accuracy unmeasured. |
| One camera coordinates people/floor views | Implemented across several layers | Head tests and integrated short movements | detour sequence — `robot/jetson/navigation/closed_loop_detour.py:373` | Person not rechecked inside the floor-view movement worker. |
| ChArUco calibration passed | Calibration implemented and applied selectively | Numerical fit plus manual visual acceptance | capture report — `docs/calibration/camera_calibration_capture_report_20260903.json:1` | Automatic acceptance was false; manual exception recorded. |
| DA V2 supplies usable physical stopping distance | Implemented estimates | Negative physical comparisons retained | distance discrepancy — `artifacts/mom-combined-right-20260912/operator-distance-check.json:2` | Metric accuracy unresolved; camera/front origins differ. |
| SegFormer proves a footprint-wide corridor | Heuristic corridor gate implemented | Synthetic tests and selected scenes | corridor construction — `robot/jetson/navigation/image_corridors.py:27` | No calibrated footprint projection; water class included. |
| Gemini assesses hazards but does not drive motors | Implemented | API/schema checks and route trials | deterministic fusion — `robot/jetson/navigation/navigation_reasoning.py:106` | Hazard accuracy and dynamic-scene coverage unmeasured. |
| Delayed distributed results are rejected | Many explicit bindings implemented | Tests and selected integration checks | route response validation — `robot/jetson/navigation/closed_loop_detour.py:314` | Not universal end-to-end expiry; clock assumptions remain. |
| EV3 independently stops on command loss | Implemented | Earlier physical watchdog/disconnect tests | watchdog — `robot/ev3/server/ev3_server.py:623` | Same-thread execution; historical tests do not validate every current path. |
| IR adds a local obstacle-stop layer | Not found in current path | Explicitly deferred | [hardware notes](<reference_docs/docs/HARDWARE_NOTES.md>) (source line 71) | Desired architecture only. |
| Full autonomous arrival works | Policies implemented | No conclusive complete arrival found | [latest approach limitation](<reference_docs/docs/DEVELOPMENT_LOG.md>) (source line 2620) | Partial successes and heuristic stop labels are insufficient. |
| Assistance request is delivered and acknowledged | Planned | No integrated demonstration found | [communication scope](<reference_docs/communication/README.md>) (source line 1) | Local playback is not caregiver receipt. |
| Hospital use or clinical benefit demonstrated | Not established | None found in inspected evidence | [project delivery plan](<reference_docs/docs/ASSISTIVE_COMMUNICATION_PLAN.md>) (source line 239) | Household trials cannot establish clinical performance. |

[Back to reading guide](README.md)
