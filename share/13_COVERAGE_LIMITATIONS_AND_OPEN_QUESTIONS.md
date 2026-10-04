# Coverage, contradictions, limitations, and open questions

These findings explain the inspected working tree, not a freshly verified live robot. They distinguish implementation, recorded software checks, physical demonstrations, and unverified assumptions. Refer to the [full dossier](FULL_TECHNICAL_DOSSIER.md) for the complete argument and the [snapshot](SOURCE_SNAPSHOT.md) for provenance. Source paths refer to the original repository; code and raw data are not bundled here.

**H. Coverage, contradictions, and unresolved evidence**

The investigation used three passes:

| Pass | Material inspected | Coverage boundary |
|---|---|---|
| Repository and history | Branch/commit/status, tracked history, robotics/config/test/script inventory, current and historical documentation | Current working tree distinguished from committed source. |
| Executable paths | Browser mission entry, mission subprocesses, detection/recognition/family tracking, Mac route/range/cloud adapters, camera/bridge/EV3 services | Main robotics chain traced; no runtime processes activated. |
| Evidence reconciliation | Development log, selected deployment/build logs, calibration report, mission reports, range discrepancies, recording/replay summaries | Selected artifacts read in detail; not every historical JSON/log or archived dataset exhaustively audited. |

The inventory included 100 Python files under `robot`, 19 Python scripts, and 90 Python files under `tests`. Counts are file counts, not test-case counts. The much larger `Archive` and communication environment were not exhaustively reviewed; communication entry points, confirmation/audio behavior, scope statements, and robot-linkage searches were inspected.

Two local cached model configurations were read to verify SegFormer labels/preprocessing and the DA V2 model type. Model inference and weight execution were not performed. Remote Jetson/EV3 runtime files, current services, private enrollment databases, and external physical recordings were not independently inspected as live state.

The important contradictions and chronology issues are:

- README’s three-of-five recognition versus current two-of-five configuration.
- Documentation describing clothing as observation-only versus current enabled clothing approach.
- Current family identity semantics versus older “fresh face required” comments.
- README’s “recording not added” versus implemented recorder and retained recording.
- `navigation.yaml`’s standoff/sweep values versus the active hard-coded/CLI policies.
- Historical camera coordinates versus reference-specific runtime limits.
- “Footprint-wide corridor” language versus fixed image trapezoids.
- Calibration YAML in use despite the retained report’s automatic rejection.
- Historical Gemini model names versus current `gemini-3.5-flash-lite`.
- Latest installed tracker file versus service activation still pending.
- Early “enrollment never commands motors” statements versus the later combined console containing manual drive, mission, and stop controls.

Unresolved technical evidence includes current deployed revision/configuration, physical motor identity, camera extrinsics, independent optical tilt, stopping-distance accuracy, low-battery behavior, broad identity/confusion performance, dynamic obstacles during the person-view gap, and complete current-version fault-stop timing.

The inspected reports concern supervised household/room trials and a small set of enrolled people. I found no hospital trial protocol, clinical population evaluation, measured care-coordination benefit, completed assistance delivery, or caregiver acknowledgement. That is a statement about the inspected evidence, not a claim that no additional work exists elsewhere.

1. **Which parts did you personally design, implement, integrate, and physically test, and which were produced with collaborators or coding-agent assistance—particularly the shared-camera policy, appearance tracking, and engineering diagnosis?**
2. **Outside this repository, do you have any documented trial that completed the full chain from an assistance request to the intended recipient receiving and acknowledging it, or independently verified final approach distance; if so, what setting, participants, measurements, and artifacts support it?**

[Back to reading guide](README.md)
