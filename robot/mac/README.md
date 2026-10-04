# Mac route-perception offload

Gemini now adds route hazard vetoes, corridor preference and paired-image person
occlusion interpretation. See [runtime and offline checks](../../docs/GEMINI_NAVIGATION.md).
Restart this service when installing the matching Jetson bundle. The Jetson
rejects old route responses without combined evidence.

`route_perception.py` fetches a live Jetson snapshot, runs the ADE20K
SegFormer-B0 model on Apple MPS, and returns conservative left/straight/right
floor evidence plus one bounded route decision. It listens on port 8091 so the
Jetson remains the mission and motion authority.

The launch agent keeps the service available while this Mac user is logged in.
Its Python environment and model cache live under `~/Library/Caches/Echora` and
are intentionally outside Git. Logs live under `~/Library/Logs/Echora`.

The route endpoint refuses inference unless the Jetson reports live camera
frames both before and after snapshot capture. Model weights and the MPS path
are warmed before the HTTP listener starts, so the first accepted decision
fits the Jetson's one-second freshness limit.

## Gemini scene calibration

The `/view` endpoint also caches stationary camera frames for ten minutes.
The Jetson startup calibrator captures the approved lower, middle and upper
poses once, then sends their three cached frame hashes to
`POST /scene-advice`, with their current camera reference. The Mac calls
Gemini `gemini-3.5-flash-lite` once with those ordered images and strict JSON output.
All robot cloud paths share this selection in `robot/cloud_models.py`, use low
thinking, and authenticate with the Mac's existing Vertex ADC identity.

Gemini supplies floor/room/overhead observations. The service checks the camera
reference and stopped pose before and after inference, then retains image
templates for that reference. Startup accepts the three fresh semantic roles
and parks at the middle position, without another sweep or revisit cycle.
The middle view may contain some floor or ceiling; it must still be a useful
room view. Old template labels do not select the startup poses.
When the user has saved both physical endpoints in the UI, the upper capture
may be a clearly recognised raised room view without visible floor; ceiling
visibility is optional. This exception is bound to the saved current-reference
upper target, records the role as `raised`, and never marks ceiling verified.
Calibration uses the existing three-count settling tolerance between stopped
camera telemetry samples. It records the frame's measured position, so image
hashes remain bound to their capture metadata. Transit steps check motor
progress; images are evaluated at the three requested views. Detailed pixel
transition checks remain in the optional movement probe. The route endpoint retains its
own current-pose checks.
Cloud semantics do not extend the approved motor range, clear a route, or
issue motor commands.
Pixel segmentation handles the local floor checks. Gemini also handles useful-view
setup, person search/framing, route and occlusion advice, and wardrobe comparisons.

Images are ordered by role for interpretation. The service only
claims an upward chronological sweep when their capture timestamps agree;
otherwise Gemini classifies the views without that motion assumption.

If both endpoints are established and Gemini says the intermediate view is
overhead, the Jetson can lower its target by five encoder counts and capture
again. This recovery is limited to three adjustments, stays inside the approved
travel bounds, and keeps the middle target at least five counts above the lower
target. Unknown endpoints, no progress, unusable frames or cloud errors end the
attempt. Successful recovery updates the selected forward position without
recapturing the endpoints. Exact image equality is unnecessary.

Successful Gemini calls have no artificial provider pacing delay. The old Groq
65-second input-token pacing applies only when explicitly using that legacy
adapter. Calibration retains its bounded sweep and recovery budget.

Quota headers are exposed in `/health` and successful advice responses. A
rate-limit response stops this attempt and imposes a local cooldown of at
least 60 seconds; it never automatically retries. Other provider or malformed
answer failures also stop the attempt. The daily allowance and minute limits
are separate; use the returned headers rather than assuming a fixed quota.

Saved-image evaluation (no motor movement):

```sh
python3 -m robot.mac.vision_scene_advisor \
  docs/calibration/view_references/floor_negative.jpg \
  docs/calibration/view_references/room_negative.jpg \
  docs/calibration/view_references/overhead_confirmed.jpg \
  --upward-sequence --report /tmp/gemini-scene.json
```

Provider reference: [Gemini 3.5 Flash Lite](https://ai.google.dev/gemini-api/docs/models/gemini-3.5-flash-lite).


Gemini camera setup/search advice uses Vertex AI ADC, not a Google API key. Install
`google-auth[requests]` in the route service Python environment and authenticate
ADC on this Mac. The existing ADC project/quota project is used unless
`GOOGLE_CLOUD_PROJECT` is set. `/search-view` accepts the same bound JPEG/request
format as `/camera-setup`, with person-search scene instructions. Images and tokens
remain transient. Runtime failures respect cooldowns without switching models.
Explicit Groq/Ollama diagnostic adapters remain available for reproducing old
evaluations; they are not selected by default. `/health` reports each cloud model.

## Robot speech delivery

The loopback-only `voice_delivery.py` service reuses the communication backend's
transcription provider and Fish Audio synthesis for operator-approved messages.
Install its LaunchAgent and the matching SSH reverse forward as described in
[the robot delivery setup](../../docs/ROBOT_MESSAGE_DELIVERY.md). The service
does not start or control robot missions.
