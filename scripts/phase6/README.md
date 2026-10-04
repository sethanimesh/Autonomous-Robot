# Phase 6 build and run

The [Gemini route and occlusion update](../../docs/GEMINI_NAVIGATION.md) is installed
on the Jetson with the matching Mac service. Jetson preflight and real Gemini API
checks passed on 2026-09-09; chassis movement and a staged person-occlusion test
remain pending. The normal bundle command includes the update.

The faster build and camera recovery are installed on the Jetson. A complete
search → identification → approach still needs to pass on hardware. The physical
camera unplug/reconnect test was deferred by the operator.

## Behavior

- Household recognition uses a 0.40 similarity threshold and two supporting
  matches among five observations. Duplicate frames do not count twice. Existing
  enrolled face views remain in use; no enrollment is erased.
- Face detection accepts a 0.60 confidence score and checks the full image at up
  to 3 Hz when body crops miss a face or person messages stop arriving. This also
  supports seated and partially visible people.
- Recent face matches, clothing appearance and body overlap retain a candidate
  while waiting for their face again. Body size/position guides camera framing.
  A complete standing body is unnecessary. Body appearance alone does not declare
  a new identity or estimate distance.
- The search sweeps the saved forward view, then the lower room view.
  Upper travel limits are never whole-room search pitches. A candidate gets nearby head views, a revisit, and then the search can
  continue past them. Brief loss during centering triggers reacquisition.
- Candidate views follow the [person-search scenarios](../../docs/PHASE6_PERSON_SEARCH_SCENARIOS.md):
  look upward for a body cropped at the top, frame an edge-clipped face before
  centering, keep a usable face steady, and restore the best recent face view
  after losing it. Each decision is recorded in the scan report.
- Temporary feedback and route failures get up to three retries, with a 0.5 s
  retry delay. Target loss gets local then wider reacquisition. A persistent
  problem leaves a `paused` report and the measured heading for a later run.
- Turn error tolerance is 25° (previously 8°). Accepted overshoot updates the
  measured heading and continues the existing scan. Braking timing is unchanged.
  Turns use feedback after stopping so coasting counts toward cable heading.
  Unknown position, changed encoder references and an unconfirmed stop require
  restored feedback rather than repeated movement commands.
- The camera first retries opening its device. After 12 seconds without frames,
  a separate watchdog exits the camera worker so its existing systemd service
  restarts it. This also handles a blocked capture call without restarting the
  bridge or changing motor references.
- A temporary camera drop during a scan turn stops movement, waits for fresh
  frames and unchanged motor references, then continues the same turn and scan
  step. Recovery has a bounded wait; a persistent outage still pauses the mission.
  A drop after forward movement begins requires target reacquisition before
  another approach step.

These are configurable household tuning defaults, not measured recognition
accuracy. The current gallery contains one active target with several templates;
this change does not introduce a database that names every family member.
The final face-size threshold is an image heuristic, not a distance measurement.
Track speed is increased for the next run: bridge cap 240°/s (previously 120),
scan/detour turn speed 0.60 rad/s (previously 0.30), approach 0.06 m/s. The old bridge
cap limited straight travel to approximately 0.03 m/s with the saved geometry;
the new cap accommodates 0.06 m/s. Actual speed and stopping remain to be measured.

## Build on the Mac

From the repository root:

```sh
python3 scripts/phase6/check_build.py
python3 scripts/phase6/phase6.py bundle --output /tmp/echora-phase6.tar.gz
```

The bundle contains Jetson Python modules, transport profiles, the runner and
checksums. It excludes models, images, credentials, enrollment and robot limits.

## Install when the Jetson is back on

Copy the bundle and `scripts/phase6/phase6.py` to the Jetson, then run there:

```sh
python3 /tmp/phase6.py install /tmp/echora-phase6.tar.gz --household-profile --faster-tracks --restart
source /opt/ros/humble/setup.bash
cd /home/animesh/echora
python3 phase6.py check --seconds 8 --report /tmp/phase6-health.json
```

Installation verifies checksums and syntax first, backs up replaced files under
`/home/animesh/echora/backups`, and merges only the three household tuning values
into the existing `perception.yaml`. The Jetson needs its existing PyYAML/ROS/model
dependencies. `--faster-tracks` also merges only `max_motor_speed: 240` into the
existing `robot.yaml`; saved camera limits, geometry and enrolled templates are preserved.
`--restart` requires working noninteractive sudo, clears service start limits and
stops the temporary `echora-face-recovery` user unit before normal services restart.
Without that sudo access, install without `--restart` and restart the normal units
as an administrator. Do not launch a second face worker.

The health check inspects advancing inference counters, distinct camera frames
and mission target observations. No person needs to face the camera for it.
A stalled body detector is reported as degraded when face/identity processing
still flows. To restart the perception chain and check again:

```sh
python3 phase6.py recover --report /tmp/phase6-recovery.json
```

This recovery does not restart the bridge or reset motor references.

## Prepared live run

The Jetson's Mac-backed routes and camera-setup calls now default to
`http://127.0.0.1:18091`. The Mac user service configured by
`robot/mac/com.echora.jetson-backend-tunnel.plist` forwards this Jetson loopback
port over SSH to the Mac's local port 8091 and reconnects after network loss.
The Mac route service must also be running. CLI URL overrides remain available.
This connection works when the Mac's Wi-Fi address changes. Automatic camera
setup now reuses matching saved views or discovers them through the cloud model
pool; its head-only calibration and service-restart reuse checks passed.

After the operator is ready for face scanning, put the robot at its cable-neutral
orientation. Camera setup runs automatically if this boot needs new views. The Mac route
service must be running. Then, on the Jetson:

```sh
python3 phase6.py check --with-robot
python3 phase6.py run --neutral --recover --steps 4
```

Use `--neutral` only at the marked neutral orientation. The runner saves
`preflight.json` and `mission.json` in a timestamped runtime `logs/phase6-*` folder.
Reports show each recovery and the reason for any pause. Existing UI start/stop
controls use the same mission implementation.

To start another search from the current position after a pause, pass the previous
preflight file (replace the example timestamp):

```sh
python3 phase6.py run --resume logs/phase6-YYYYMMDD-HHMMSS/preflight.json --steps 4
```

This starts a fresh target search and recomputes cable heading from current
encoders; it does not replay an interrupted movement or restore old face evidence.
If a track reference changed, restore neutral and use `--neutral`. An EV3 reboot
also requires restoring the camera's lower-view reference in the UI; stored
relative angle numbers alone cannot establish the camera's physical position.

For a terminal-started run, press Ctrl+C in that terminal to cancel the mission
and its child workers. The UI stop button sends an immediate motor stop, but its
mission manager only terminates missions started by that UI. For UI-started runs,
use the UI stop control. Turn EV3 off after the prepared tests.

### Automatic camera setup

The normal find-person mission now runs `--auto-setup`: matching saved camera
views are reused, otherwise useful views are discovered from the current pose.
The UI no longer blocks Find Me solely because limits need restoration.

For a head-only discovery run on the Jetson (tracks remain stopped):

```sh
python3 camera_head_calibration.py --execute --discover-views --report logs/camera-discovery.json
```

The Mac camera backend uses Gemini 3.8 Flash for all cloud decisions and retains
model cooldowns without substituting an older model. Configure the existing
Google Vertex ADC identity on the Mac. No keys go to the EV3/Jetson
bundle. See `docs/CAMERA_VISUAL_SETUP.md` for behavior and test coverage.
