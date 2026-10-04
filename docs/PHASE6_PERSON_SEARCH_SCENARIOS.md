# Phase 6 person-search decisions

The detector supplies fresh body/face boxes. The mission applies the following
rules in order at each candidate view, then observes again. It does not require
the whole body to fit or infer a person's identity from their apparent size.

| Observation | Automatic action |
| --- | --- |
| Face cut off at the top, or high in the image | Raise by one small step and inspect again. Never explore farther downward for this case. |
| Face cut off at the bottom, or low in the image | Lower the camera by one bounded step and observe again. |
| Face vertically framed but at the left/right edge | Let the existing chassis controller center the person before more tilt experiments. |
| Face framed but identity pending | Keep the camera steady while recognition gathers independent matches. A profile remains a valid face candidate. |
| Body reaches the top edge, face absent | Treat the face as possibly above the frame; raise by one small step and inspect again. A seated torso/legs can produce this case. |
| Small body fragment near the bottom | Raise by one small step before chasing its horizontal location. |
| Body geometry suggests a higher face | Try one higher view and reassess the fresh detection. This is a search hint, not proof of head location. |
| Body geometry suggests a lower face | Try one lower camera step. |
| Body at the side with no vertical cropping clue | Center the candidate horizontally, then inspect for a face. |
| Centered body with no face or strong vertical clue | Raise by one small step and reassess. |
| Face disappears while exploring camera angles | Restore the best face view seen during this investigation and obtain fresh observations. If only a body was seen, restore its last useful view. |
| No useful detection or all relevant views already tried | Finish this bounded investigation; the mission retains/revisits the candidate or continues the room search. It does not spin in an endless tilt loop. |

Room sweeps use the forward and lower views. The upper calibration endpoint is
only a travel bound; person evidence drives incremental upward adjustment.

Vertical cropping takes priority over horizontal centering. Camera targets remain
inside the saved operating range. Views within the existing small position
tolerance count as already tried; a near-limit motor position does not have to be
exact. A changed camera reference prevents restoring an old view.

Identification still uses the existing household face threshold and two distinct
supporting matches. Existing face-anchored body/clothing continuity handles brief
occlusion. Body detection alone does not assign a family member's identity.
Confirmed identity proceeds through the existing floor/route checks and short
approach steps. Camera/feedback recovery remains active underneath these decisions.

`candidate_decisions` records the observed scenario, current position, proposed
views, and actual action. `retained_face_views` / `retained_body_views` record
restoration. Camera-view exploration is bounded by the existing candidate timeout.

## Evidence and remaining live verification

The 2026-09-06 repeat found a body and briefly detected a face at the image's top
edge, then moved down to head65/67 and lost the face. It also continued through an
11.08° turn error with the new25° allowance. It did not identify or approach the
operator. The final pause was a separate cable-reserve error at-80.34° against the
UI's±80° planned envelope.

The new scenario rules have focused automated coverage for vertical/side cases,
bounded targets, steady face identification, and restoration after losing a face.
They are installed but have not yet been verified in another physical face-search
run. The operator was told to relax after the preceding run; obtain fresh readiness
before scanning again. Recompute the current cable heading before that run.


On 2026-09-08 the first combined retry exposed a ceiling-facing default before
person investigation. The forward view now uses the verified room/floor image,
and all candidate upward changes are incremental. Two head moves and service
reload passed; the updated combined person search remains unverified.


## Scene-assisted return rules (2026-09-08)

Gemini assesses overhead geometry through ADC. Segmentation remains responsible
for floor/obstacle evidence in the existing approach route checks; face/person
boxes provide the immediate camera-framing evidence. A scene model cannot name
a family member or establish travel clearance.

- Raise by eight counts, then obtain fresh detections. If the person disappears,
  return to the best face view, first useful body view, or entry view.
- At a raised view without a face, consult Gemini up to twice per investigation.
  A clear ceiling classification returns to the useful view. A fresh face takes
  priority over a contrary cloud classification.
- After three upward probes or 24 counts without a face, return and continue the
  existing search. Reaching the saved bound or exhausting visited views also returns.
- Remember failed upward views for 60 seconds at this heading/reference. A fresh
  face can override that search hint; another heading is assessed independently.
- Unknown/slow/unavailable cloud answers use detector rules and the same finite
  probe budget; they do not pause the mission. Replies from a changed pose are ignored.

Focused replay tests cover ceiling, lost person, unavailable cloud, repeat-angle
avoidance, return to an observed face and detector precedence. A fresh combined
hardware search is still required; no face test was started for this change.


The body-part integration asks Gemini 3.8 Flash about anatomy and framing at
candidate views, not just scene type. Fresh legs/torso evidence can continue
raising beyond the detector-only three-probe fallback, up to six total adjustments
inside the saved range. Four cloud checks and a 60-second total bound apply.
Model and motor latency no longer consume the separate observation budget.
See [stage-by-stage Gemini decisions](PHASE6_GEMINI_DECISIONS.md) for implemented
behavior versus proposed route and occlusion work.
