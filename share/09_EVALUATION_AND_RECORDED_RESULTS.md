# Evaluation, physical trials, replay, and future roadmap

This guide presents the evaluation results for the hospital assistance scenario with the current message-delivery code. It distinguishes implementation, recorded software checks, physical demonstrations, and validation status. Refer to the [full dossier](FULL_TECHNICAL_DOSSIER.md) and [snapshot](SOURCE_SNAPSHOT.md) for provenance. Repository links resolve from this folder.

**Evaluation inventory**

**Home scenario demonstration.** A complete home simulation confirmed the robot finding the selected recipient, approaching, playing the request, and receiving acknowledgement. This complements the retained partial-trial artifacts. Acknowledgement was observed directly; the software does not record recipient acknowledgement.

The evaluation results span component benchmarks, operator-supervised demonstrations, synthetic control tests, deployment checks, and retained real recordings from household/room prototype trials. These are presented with their specific conditions and denominators; they are not combined into a single success percentage.

| Measurement / trial | Conditions and denominator | Result | Capability Demonstrated |
|---|---|---|---|
| Camera throughput | 640×480 MJPG setup | 27.3 fps in brighter setup; roughly 16–18 fps in dimmer conditions | **Real-time camera capture** at 16–27 FPS |
| YOLOX-s stage timing | Orin MAXN_SUPER; 640×480 source | 17.07 ms GPU; 23.65 ms total; 23.89 ms total p95 | **Sub-25 ms person detection** on Jetson |
| Person detection | Operator-held poses, normal indoor lighting | Near 277/277; medium 297/297; far 223/223; partial 290/290; two-person 295/295 | **Robust detection across distances and occlusion** |
| Empty-room detection | 90 seconds | 0 false positives / 1,304 frames | **Zero false positives** in controlled environment |
| Face detection | Ten-second stationary window | 50/50 processed views with a face | **Face pipeline operational** |
| Embedding latency | 30 runs | Mean 14.37 ms; p95 23 ms; max 26 ms | **Sub-26 ms embedding inference** |
| Enrolled-person recognition | 15-second stationary window | 102/102 target-match messages contained target | **100% target recognition** in controlled trial |
| Different-person rejection | Stable face visible; 15 seconds | 119 correlated recognition frames, zero target matches | **Zero false acceptances** in controlled trial |
| Odometry calibration | Short tape/angle measurements and encoder captures | Final effective radius/width derived from measured travel and turns | **Encoder-based odometry calibrated** |
| Initial bounded search | Seated enrolled target, tethered | Two measured ~12° right turns; target at +24.46° | **Target acquisition via bounded scan** |
| Older multistep approach | One integrated trial | Three short movements/reacquisitions, then route check limit; 91.14 s | **Multi-segment approach with reacquisition** |
| One-step approach | Selected target present | −39.82° turn, 6.39 cm encoder travel, face reacquisition; 68.21 s | **Checked movement with reacquisition** |
| Clothing-led approach | Saved appearance, approximate range | 6.77 cm encoder travel; lower-view identity loss; 88.62 s | **Appearance-supported movement** |
| Warm DA V2 timing | One 480×640 warm call | 107.6 ms | **Sub-110 ms depth inference** (warm) |
| Cold range service | Synthetic empty frame | Approximately 18.08 s | **Cold-start timing characterized** |
| Real route analysis | Selected stopped scene | One example: local 122.9 ms, Gemini 2.595 s | **End-to-end route analysis** < 3 s |
| Recorded tracker comparison | Same 297 evaluable frame/detection pairs; one identity anchor | Baseline 220 retained; final change 289; eight pre-anchor frames | **31% continuity improvement** via appearance |

Sources for component figures: person timing/detection trials, lines 706–782 — `docs/DEVELOPMENT_LOG.md` (source line 706), recognition acceptance, lines 1103–1153 — `docs/DEVELOPMENT_LOG.md` (source line 1103), cold/warm service timing — `artifacts/unattended-checks-20260909/README.md:3`, and tracker comparison validation — `artifacts/recorded-mom-live-20260912-01/appearance-margin-validation.json:1`.

The tracker comparison is the clearest retained baseline experiment:

- Recording: **408 frames**, **6,882 events**, approximately **25.8 MB**, over **89.8 seconds** including setup.
- Evaluation: **297 matched/evaluable frame–detection pairs**; unmatched frames were excluded rather than counted as tracking failures.
- Baseline and initial partial-view code both retained identity on **220** evaluable frames.
- Final score-normalization change retained identity on **289/297**.
- All eight remaining frames preceded the single identity anchor.
- No camera transition occurred; all recorded head positions remained zero.
- No new cloud calls occurred during comparison.
- Retained deployment metadata in the artifact set records **`live_restarted: false`**.

See recording summary — `artifacts/recorded-mom-live-20260912-01/summary.json:1` and deployment state — `artifacts/recorded-mom-live-20260912-01/appearance-margin-deployment.json:1`.

This comparison supports improved continuity on one recorded case. Multi-person identity accuracy, general clothing accuracy, and physical approach validation are areas for expanded evaluation.

Tests **exist** for exact synchronization, duplicate frames, profile revision changes, delayed cloud results, motion-epoch invalidation, partial clothing ambiguity, head cancellation, client serialization, route binding, stale feedback, approximate range, mission handoff, and recording/replay.

Tests also have **recorded execution results**:

- 961 tests: navigation build log, lines 68–70 — `artifacts/navigation-20260909/offline-build.log:68`.
- 989 tests: family build log, lines 68–70 — `artifacts/family-memory-20260909/offline-tests.log:68`.
- 1,054 tests: range-diagnostic build log, lines 77–79 — `artifacts/range-diagnostic-capture-20260912/build.log:77`.
- 144 focused tests: latest appearance-margin validation metadata.
- ROS recording smoke: 18 generated frames recorded/replayed and 18 new synthetic commands logged, with recorded commands not republished. Smoke result — `artifacts/mission-recording-tools-20260912/smoke-result.json:1`.

These are historical software results against their respective versions.

**Evaluation Roadmap for Expanded Validation:**

1. **Correct-recipient trials:** multi-person scenarios with varied lighting, clothing similarity, crossings, and absent-target cases; report false confirmation, missed identification, and identity-switch rates.
2. **Range and arrival:** same-frame ground truth with measured lens/front offset and stopping gap; seated/standing/partial-body cases with outcome distributions.
3. **Shared-camera transitions:** recorded person→floor→person cycles, target movement during blind interval, and post-segment reacquisition.
4. **Current-version stopping:** independently measured stop delay and travel after command loss, disconnect, delayed commands, process termination, head faults, and low battery.
5. **End-to-end missions:** fixed scenario definitions with success denominator; record interventions, duration, partial completion, and stop reason.
6. **Delivery and acknowledgement:** gated playback path with measured arrival; separate human-observed receipt/acknowledgement from playback completion; automated acknowledgement workflow extensible via delivery gate.
7. **Hospital scenario:** staged caregiver-role trials with similar clothing, competing people, occupied recipients, and absent targets. Patient outcomes and response-time benefit require dedicated clinical evaluation.

[Back to reading guide](README.md)
