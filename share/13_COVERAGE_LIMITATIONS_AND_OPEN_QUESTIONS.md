# Coverage, contradictions, limitations, and open questions

This guide describes the implementation and evaluation status as documented on 2026-10-04. Historical measurements retain their original provenance; see the [full dossier](FULL_TECHNICAL_DOSSIER.md) and [source snapshot](SOURCE_SNAPSHOT.md). Source line citations refer to the original investigation unless identified as a current-source review.

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

Evaluation to date concerns supervised household/room trials with a small set of enrolled people. The current implementation includes supervised robot playback; the reported complete home demonstration includes human acknowledgement. Hospital participants, patient outcomes, and care-coordination benefit have not been evaluated. See the [evaluation guide](09_EVALUATION_AND_RECORDED_RESULTS.md) for the home demonstration and retained measurements.

Further evaluation records should capture:

1. A recording, trial count, duration, independently measured stopping distance, and human-observed receipt and acknowledgement for complete delivery trials.
2. The deployed revision, configuration, calibration, participant conditions, and room setup used for each trial.

[Back to reading guide](README.md)
