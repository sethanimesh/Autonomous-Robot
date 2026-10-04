# Automatic camera setup

The head-only setup runner owns the complete move → settle → capture → cloud
classification → correction → return-check → save cycle. It does not require a
person to approve individual steps or restore an old lower-view reference.

## Entry points

- `camera_head_calibration.py --execute --discover-views`: find useful views from
  the current pose, including after an EV3 reboot or incorrect saved angles.
- `camera_head_calibration.py --execute --auto-setup`: reuse matching saved views
  when possible; otherwise discover them. The normal find-person mission uses
  this mode. The UI no longer requires manual limit restoration before starting.
- `--visual-setup`: optional refinement within an already valid saved range.
- Without `--execute`, all modes produce a preview and perform no robot access.

## Behavior

The EV3 encoder is acknowledged at its current count when necessary; no zeroing
or mechanical-stop homing is performed. The coordinator binds the resulting
reference to every movement and fresh camera observation. Tracks stay stopped.

The vision model judges functional views rather than matching room colors or old photographs.
A lower view needs nearby floor and forward room context. An upper view needs
useful higher room context; a face or ceiling is optional. Readable but unsuitable
room views can trigger exploration. Dark or covered images are rechecked without
assuming that black means floor or applying extra motor force.

Discovery uses normal-speed ten-count steps within a finite window of 60 counts
on either side of the starting encoder count, with at most 20 image observations.
This is an exploration budget, not a measurement of physical linkage travel.
The current bridge settling allowance (up to four counts) is accepted. The
coordinator reuses upper-view evidence collected while seeking the lower view,
avoiding duplicate image requests at the same position.

The existing small endpoint margins are included in return verification. If a
margin removes the visible floor or useful upper context, the program adjusts
the endpoint and repeats the check automatically. Both endpoints are saved
together only after successful return checks, with source `cloud_useful_views`.
These are useful operating limits, not inferred mechanical stops. Route
clearance and person identification remain separate mission steps.

If every configured model is limited, provider-directed cooldowns allow automatic resumption
at the same pose with a fresh frame. The cumulative cooldown budget is two
minutes. Transient request errors get a retry. A genuine encoder-reference
change, absent motor progress, persistent unreadable view, or exhausted discovery
budget ends the head-only attempt with a report; it does not fabricate limits.

## Cloud and deployment

Gemini now uses Vertex AI with the Mac's Application Default Credentials (ADC),
with automatic in-memory token refresh. Google API keys are not read. The runtime
needs `google-auth[requests]`; project selection uses GOOGLE_CLOUD_PROJECT, ADC's
project, or its quota project, and GOOGLE_CLOUD_LOCATION defaults to global.
See Google's [ADC Vertex quickstart](https://docs.cloud.google.com/vertex-ai/generative-ai/docs/start/quickstart).

Camera setup and search framing use `gemini-3.5-flash-lite`, selected centrally in
`robot/cloud_models.py`, with minimal thinking and existing ADC authentication.
Model cooldowns are respected; there is no automatic older-model or Groq fallback.
Search framing uses a separate `/search-view` pool and human-view instructions. Its eleven-second
Jetson wait leaves ROS callbacks active and falls back to deterministic camera
rules on failure. Cloud output never authorizes chassis movement or identity.

Earlier-model ADC checks on the captured ceiling and room/floor images returned the expected
scene classes in 5.341 and 3.726 seconds respectively. These are individual checks,
not a model accuracy or latency benchmark. Google's billing/quota project is
`project-d55579d3-19a4-4179-926`; no credentials are stored in the repository.

Images are resized without cropping to at most 448 pixels on the longest side,
reducing upload bytes. Gemini uses a validated schema and returns the original
image digest; the Jetson verifies
that response and the stopped camera reference before taking another action.
Gemini uses the Vertex AI image/JSON endpoint
([Google structured-output documentation](https://ai.google.dev/gemini-api/docs/generate-content/structured-output)).

The Jetson connects to the Mac through `http://127.0.0.1:18091/camera-setup`,
forwarded by the managed private SSH tunnel. See
`robot/mac/com.echora.jetson-backend-tunnel.plist`. The listener on the Jetson is
loopback-only. No credentials or images are bundled into deployment archives.

## Evidence

Manual image collection and approved Groq evaluations are in
`docs/calibration/visual_setup_20260906/manual/`. They cover forward/lower, room,
upper, and covered views; earlier reference images are recorded alongside them.
The covered image returned dark/unknown with no tilt direction. These are
calibration development examples, not a generalization benchmark.

Automated tests in `tests/test_camera_visual_discovery.py` cover upper, floor,
ceiling and floor-only starting views; new-boot acknowledgment; obsolete limits;
four-count settling; margin correction; cloud retry/cooldown; covered views;
reference changes; finite search; and atomic persistence. Current live results
and limitations are recorded in `docs/DEVELOPMENT_LOG.md`.

## Live result — 2026-09-08

Automatic discovery and return verification passed on the EV3 in 121.64 s,
using four models across Groq and Gemini. Both operating endpoints were saved.
A subsequent bridge service restart reused them in 4.37 s with no cloud calls
and no repeated scan. Full power-cycle discovery is covered in software tests
but has not yet been repeated on hardware. Full room search was outside this
head-only run. Reports are in
`docs/calibration/visual_setup_20260906/live_20260908/visual-discovery-20260908-1605/`.

The initial person-search view uses the verified room-and-floor lower view. An
encoder midpoint between endpoints is not a calibrated forward image; the live
2026-09-08 check showed that midpoint facing the ceiling. Upper travel remains
available for incremental person-guided framing, never a whole-room sweep.
