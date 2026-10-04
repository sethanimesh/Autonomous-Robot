# Gemini's role throughout Phase 6

The useful integration is a repeated observation → interpretation → bounded action
→ observation cycle. A larger model alone does not repair a controller that asks
the wrong question or discards the answer before observing its last camera move.

The 2026-09-08 live run demonstrated that failure: Gemini correctly noticed a
person, but only returned scene labels. Two cloud calls used about eight seconds
of a twelve-second candidate timer. The head rose through 25 → 20 → 13 → 6,
then returned to 25 without adequately observing the last view. Search finished
in 180.45 seconds without identity. It recovered from stale feedback and a turn
error, so neither was the final identification blocker.

| Stage | Gemini question and resulting behavior | Current implementation |
| --- | --- | --- |
| Camera setup | Is this a useful room/floor or upper room view? Adjust and verify the actual view, including after reboot. | ADC-backed automatic setup exists. Search starts at the verified room/floor view, never an unobserved encoder midpoint. |
| Initial room search | Do sparse detections miss an obvious partial person? Inspect selected stationary headings with the full image and detector evidence. | Local detectors trigger investigations. Gemini review of an entirely detector-empty sweep is a next change, not implemented. |
| Person investigation | Which human parts are visible? Legs/torso without head means raise; head clipped below means lower; usable head/profile means hold. | New structured body-part/framing output is wired into camera planning. Whole bodies are unnecessary. Each move gets another observation; cloud/motor waiting no longer spends the observation timer. |
| Identity | Is the head sufficiently visible to hold this angle, or should framing improve? | Gemini guides framing. Enrolled face matches establish identity; body/clothing continuity can retain an already identified candidate. Gemini does not invent a family member's name. |
| Route and approach | What objects explain the segmentation evidence? Which candidate corridor avoids the chair, cable, or furniture? | Prepared offline: Gemini hazard vetoes and corridor ranking between two stopped local floor checks. Fresh local evidence still gates the turn and each 5 cm step. Hardware validation pending. |
| Reacquisition/recovery | Did the person become occluded, leave the view, or did our camera adjustment lose them? | Prepared offline: two-image comparison within a stationary candidate investigation; rules restore the previous view, briefly wait, rescan or stop. Side hints are diagnostic and never command travel. Hardware validation pending. |
| End condition | Is a useful head view available and is the known target close enough according to current observations? | Existing identity and short-step approach checks remain. Gemini does not estimate an exact stopping distance from apparent body size. |

## Evidence and actions

For person framing, the Jetson sends a fresh full image from a stationary head.
Gemini 3.5 Flash Lite returns quality, scene, human visibility, visible parts, framing hint,
and one evidence sentence. The Jetson binds the response to the unchanged camera
reference/pose and maps raise/lower to one eight-count step inside saved limits.
A local face detection wins over a model's generic height guess.

The next route integration should send the same fresh image plus segmentation
corridors and obstacles, without inferring a metric path from prose. Gemini can
rank already generated routes or request another view; the planner measures and
executes the selected short segment, then checks again. Keep this separate from
identity and camera-head calibration.

Cloud calls occur when a stage needs interpretation. Local detectors, frame health,
encoder feedback and the manual Stop remain responsive throughout. Unknown or
unavailable cloud output falls back to local evidence and finite search rules.
Cloud text is not executed as motor commands. Rate-limit cooldowns are respected.

## Person-investigation limits

The ordinary detector-only fallback allows three upward probes. Fresh Gemini
body-part evidence can support continued framing within the saved range, with a
maximum of six total camera adjustments and four cloud checks in one investigation.
A 60-second overall bound prevents indefinite waiting. Network and motor time are
excluded from the twelve-second observation budget, so a requested view is actually
observed. Ceiling, lost-person and unhelpful-view returns remain active. A head or
profile reported by Gemini can retain its view while local identity gathers evidence.

The body-part and timer changes have software regression coverage. The operator
authorized all needed task images on 2026-09-08. Gemini Pro correctly returned
raise for the archived cropped torso/legs and hold for the visible profile, both
in direct and running-service checks. No fresh face search should start until
the operator is ready again. Further corridor/occlusion integration above is
explicitly proposed work, not claimed as already deployed.


## Combined framing evidence

Gemini body-part guidance has priority weight3, a coarse local body-box cue weight1,
and a fresh local face-framing cue weight4. These are decision priorities, not
probabilities or measured accuracy. Agreeing directions add support; conflicting
coarse body evidence does not silently replace Gemini's interpretation. A clear
face has the most specific framing evidence and remains primary when available.
Missing/uncertain cloud evidence contributes no vote, so detector rules still work.
Every candidate decision records the source votes, scores, agreement and selected
source. Segmentation remains the route-evidence layer; it does not vote on identity.
