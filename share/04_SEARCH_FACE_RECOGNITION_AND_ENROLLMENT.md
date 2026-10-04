# Person search, face recognition, and enrollment

This guide presents the implementation and evaluation status. Historical measurements retain their original provenance; see the [full dossier](FULL_TECHNICAL_DOSSIER.md) and [source snapshot](SOURCE_SNAPSHOT.md).

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

[Back to reading guide](README.md)
