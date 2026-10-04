# Evaluation, physical trials, replay, and proposed tests

These findings explain the inspected working tree, not a freshly verified live robot. They distinguish implementation, recorded software checks, physical demonstrations, and unverified assumptions. Refer to the [full dossier](FULL_TECHNICAL_DOSSIER.md) for the complete argument and the [snapshot](SOURCE_SNAPSHOT.md) for provenance. Source paths refer to the original repository; code and raw data are not bundled here.

**F. Evaluation inventory**

The evaluation material is useful but heterogeneous. It contains component benchmarks, operator-supervised demonstrations, synthetic control tests, deployment checks, and one retained real recording. They should not be combined into one success percentage.

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

Sources for component figures: [person timing/detection trials, lines 706–782](<reference_docs/docs/DEVELOPMENT_LOG.md>) (source line 706), [recognition acceptance, lines 1103–1153](<reference_docs/docs/DEVELOPMENT_LOG.md>) (source line 1103), cold/warm service limitations — `artifacts/unattended-checks-20260909/README.md:3`, and tracker comparison validation — `artifacts/recorded-mom-live-20260912-01/appearance-margin-validation.json:1`.

The tracker comparison is the clearest retained baseline experiment:

- Recording: **408 frames**, **6,882 events**, approximately **25.8 MB**, over **89.8 seconds** including setup.
- Evaluation: **297 matched/evaluable frame–detection pairs**; unmatched frames were excluded rather than counted as tracking failures.
- Baseline and initial partial-view code both retained identity on **220** evaluable frames.
- Final score-normalization change retained identity on **289/297**.
- All eight remaining frames preceded the single identity anchor.
- No camera transition occurred; all recorded head positions remained zero.
- No new cloud calls occurred during comparison.
- Latest deployment metadata says **`live_restarted: false`** and activation pending.

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
6. **Delivery and acknowledgement:** evaluate only after a real message-to-recipient-to-playback-to-response path exists.

[Back to reading guide](README.md)
