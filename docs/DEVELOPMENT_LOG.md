# Development Log

This is the chronological record of development work, including failures. Add a
new entry for each meaningful experiment or configuration change; do not erase
failed attempts after they are understood.

## 2026-09-02 — Initial Mac inventory and connection check

### Scope

Read-only discovery only. No software was installed, no remote configuration was
changed, and no motor command was sent.

### Mac workspace

- **Success:** Located the project workspace at
  `/Users/animesh/Animesh/Project 2.0/Echora Robo`.
- **Success:** Confirmed that `AGENTS.md` was the only existing project file.
- **Observation:** The workspace was not a Git repository at the time of the
  check.
- **Success:** Confirmed the Mac is Apple silicon (`arm64`) and its active LAN
  address is `192.168.1.26/24`.

### Remote connectivity

- **Failure:** SSH to the Jetson at `seth@192.168.1.48` failed before
  authentication with `No route to host`.
- **Failure:** SSH to the EV3 at `robot@192.168.1.25` failed before
  authentication with `No route to host`.
- **Interpretation:** The supplied passwords were not tested. Although the Mac
  is on the expected `192.168.1.0/24` LAN, neither target answered the SSH
  connection attempt. Likely causes include a powered-off or disconnected
  device, a changed DHCP address, Wi-Fi client isolation, or SSH not listening.

### Next check

Confirm that both devices are powered on and connected to the same LAN, then
confirm their current IP addresses from the device screens/router. Repeat the
read-only SSH inventory before installing or changing anything.

## 2026-09-02 19:49 IST — Connectivity recheck

### Scope

Repeated only the network and SSH reachability checks after receiving approval
to continue. No remote login succeeded, no software or configuration changed,
and no motor command was sent.

### Results

- **Success:** Confirmed the Mac was still active at `192.168.1.26/24`.
- **Observation:** The Mac neighbor table contained entries for both supplied
  device addresses, but a neighbor entry alone does not prove a device is
  currently online.
- **Failure:** Jetson TCP port 22 at `192.168.1.48` returned
  `No route to host`.
- **Failure:** EV3 TCP port 22 at `192.168.1.25` returned
  `No route to host`.
- **Failure:** Direct SSH attempts to both addresses returned the same error
  before authentication. The supplied passwords remain untested.

### Conclusion

Remote inventory remains blocked pending a physical power/network check and
confirmation of the devices' current IP addresses.

## 2026-09-02 19:53 IST — Codex local-network access diagnosis

### New evidence

- **Owner confirmation:** A normal Mac terminal can SSH successfully to
  `seth@192.168.1.48`.
- **Success:** Confirmed the Mac route to `192.168.1.48` uses active interface
  `en0` from `192.168.1.26`.
- **Success:** Confirmed the Mac neighbor table resolves `192.168.1.48` to a
  hardware address.
- **Failure:** A fresh, exact `ssh seth@192.168.1.48` from the Codex task still
  returned `No route to host` before authentication.
- **Failure:** A direct ping from the Codex task also returned
  `sendto: No route to host`.

### Revised conclusion

The Jetson address and Mac route are valid. Because the same Mac can connect
from a normal terminal while Codex cannot, the remaining likely boundary is
macOS local-network access for the Codex app or its command runner. Check that
Codex is enabled in **System Settings → Privacy & Security → Local Network**,
then retry. If it is already enabled, toggle it off and on and restart Codex.

## 2026-09-02 19:59 IST — Remote inventory completed

### Access resolution

- **Success:** After the owner granted local-network access to Codex, the Jetson
  and EV3 both became reachable from the task.
- **Failure resolved:** The originally supplied Jetson username `seth` was not
  valid for the supplied password. The owner corrected the username to
  `animesh`, and SSH authentication then succeeded.
- **Success:** EV3 authentication succeeded using the owner-provided account.
- **Safety:** Both SSH sessions were closed cleanly after read-only inventory.
  No remote files or settings were changed and no motor command was sent.

### Jetson inventory

- **Success:** Confirmed Jetson Orin Nano developer kit, Ubuntu 22.04.5 LTS,
  arm64, L4T `36.4.3`, Python 3.10.12, and ROS 2 Humble ros-base.
- **Success:** Confirmed OpenCV 4.5.4, Docker 28.0.1, CUDA 12.6 runtime files,
  and approximately 725 GB free on the NVMe root filesystem.
- **Observation:** Nav2 packages queried were not installed.
- **Observation:** PyTorch 2.6.0 is a CPU-only build and reports CUDA unavailable.
- **Failure:** No `/dev/video*` device or current USB camera was present.
- **Root-cause evidence:** Kernel history showed an `Arducam_8mp` USB camera was
  previously detected, followed by descriptor errors `-71`, repeated
  `Cannot enable. Maybe the USB cable is bad?`, disconnect, and failure to
  enumerate. Physical cable/port inspection is required.

### EV3 inventory

- **Success:** Confirmed ev3dev-stretch, kernel
  `4.14.117-ev3dev-2.3.5-ev3`, Python 3.5.3, and a working `ev3dev2` import.
- **Success:** Confirmed large motors on output ports A, B, and C. Each reports
  360 counts/revolution and maximum speed 1050.
- **Observation:** Battery voltage was approximately 7.95 V during inventory.
- **Failure:** No LEGO sensor was detected, so IR feedback is unavailable.
- **Observation:** No medium motor was detected; all three attached motors
  identified themselves as EV3 large motors.
- **Success:** Found an existing C++/Cap'n Proto EV3 server under
  `/home/robot/track3r`. It was not running and was not installed as a service.
- **Unverified mapping:** Its source maps B to left track, C to right track, A to
  tool/head, and expects IR on input 2. This still needs physical confirmation.
- **Safety finding:** The existing server stops motors after a TCP disconnect,
  but lacks a stale-command watchdog, local IR emergency stop, and speed limits.
  It will not be used for a floor test as-is.

### Decision gate

Before implementation or movement testing, obtain owner confirmation of the
physical motor mapping, reconnect the IR sensor, reseat or replace the camera
USB connection, and confirm the robot can be raised with tracks clear of the
floor.

## 2026-09-02 20:14 IST — Connections rechecked and EV3 service implemented

### Physical and remote recheck

- **Owner confirmation:** Output A is the tool/camera-head motor, B is the left
  track, and C is the right track.
- **Owner confirmation:** A safe raised setup with tracks clear is available for
  the first physical motor test.
- **Success:** The Jetson now enumerates the camera as USB device `0c45:6366`
  and exposes `/dev/video0` and `/dev/video1`.
- **Success:** OpenCV opened `/dev/video0` and captured one `640×480` BGR frame.
- **Failure:** The EV3 still exposes no LEGO sensor. Input 1 reports `error`,
  while inputs 2–4 report `no-sensor`.
- **Observation:** Kernel history showed an EV3 touch sensor briefly attached to
  input 2 and later removed; it did not show an IR sensor.
- **Owner decision:** Defer the IR sensor and continue Phase 1 without it.
- **Safety:** Both SSH sessions were closed cleanly after the checks. No motor
  command was sent.

### Local implementation

- **Success:** Added a Python 3.5-compatible, newline-delimited JSON EV3 service
  under `robot/ev3/server`.
- **Safety:** The service stops on startup, explicit stop, zero command, client
  disconnect, server shutdown, stale command after 500 ms, and motor/request
  failures.
- **Safety:** Malformed requests stop motion immediately, and control clients are
  restricted to the Jetson's `192.168.1.48` address by default.
- **Safety:** Track speed is limited to ±250 and tool speed to ±150 versus the
  motors' reported maximum of 1050.
- **Success:** Added encoder/status feedback without resetting motor encoders.
- **Success:** Eleven automated tests passed on the Mac, covering limits, watchdog,
  stop paths, write failure, request validation, JSON framing, and status.
- **Success:** Python compilation and a Python 3.5 grammar parse passed.
- **Scope:** The service remains local-only. It has not been copied to the EV3,
  started against real hardware, or used for motor movement.

### Next gate

With owner approval, copy the service into a new EV3 directory without replacing
`/home/robot/track3r`, run syntax validation on the brick, start it manually with
the robot raised, and test only `ping` and `status`. Pause again before sending a
non-zero motor command.

## 2026-09-02 20:24 IST — First controlled motor test passed

### Deployment

- **Success:** Created `/home/robot/echora` without changing the legacy
  `/home/robot/track3r` directory.
- **Success:** Copied `ev3_server.py` to the new directory. Local and EV3 SHA-256
  checksums matched:
  `049a48a7508a625a008ff9885bac69cdc66e362a0abb739ac0faea7fef6e1ea2`.
- **Success:** Python 3.5 byte-compilation passed on the EV3.
- **Success:** The server started manually, stopped all motors on startup, and
  listened on TCP port 9999.
- **Success:** A Jetson client at `192.168.1.48` passed protocol-version-1 ping
  and status checks. Initial encoders were zero and all motors were stopped.

### Individual motor pulses

The robot was owner-confirmed raised with tracks clear. Each test requested
+100°/s for 0.25 seconds and then sent an explicit stop.

| Role | Port | Encoder before | Encoder after | Result |
| --- | --- | ---: | ---: | --- |
| Tool/camera head | A | 0 | 34 | Passed; tracks unchanged |
| Left track | B | 0 | 25 | Passed; other motors unchanged |
| Right track | C | 0 | 26 | Passed; other motors unchanged |

All three commands were applied to the intended logical role, returned an `ok`
response, and ended with no motor reporting a running state.

### Hardware safety-path tests

- **Watchdog:** Commanded tool/A at +80°/s and intentionally sent no refresh.
  The 500 ms watchdog stopped it automatically with
  `last_stop_reason=watchdog`; the encoder advanced from 34 to 76.
- **Disconnect:** Commanded tool/A at +80°/s and immediately closed the TCP
  connection. The server stopped it with `last_stop_reason=client-disconnect`
  and no additional encoder movement was observed.
- **Shutdown:** Stopped the manually running server with an interrupt. Its final
  cleanup stop executed.
- **Final verification:** A, B, and C each reported speed 0, an empty running
  state, and `brake` stop action. No server process remained.

### Current remote state

The new server file remains at `/home/robot/echora/ev3_server.py`. It is stopped
and has not been installed as an automatic service. No autonomous or floor
movement was attempted. IR safety remains unavailable and explicitly deferred.

## 2026-09-02 20:34 IST — Jetson client and managed EV3 service deployed

### Jetson client

- **Success:** Added a standard-library Jetson client under
  `robot/jetson/ev3_bridge` with ping, status, stop, finite pulse, 100 ms command
  refresh, response framing, and a final-stop path.
- **Safety:** Development pulses are limited to five seconds; transport failures
  discard the connection and fall back to the EV3's independent watchdog.
- **Success:** The complete local suite increased to 18 passing tests.
- **Success:** Deployed the client to
  `/home/animesh/echora/ev3_client.py`. Local and Jetson SHA-256 checksums
  matched:
  `9db77c5d8535e1bf106ad86e29ab4e4bbeae43bfaa75b566ed019a51b7e4e781`.
- **Success:** Python byte-compilation, ping, and status passed on the Jetson.

### Raised paired-track tests

The deployed client refreshed commands inside the EV3's 500 ms watchdog and
sent an explicit stop after each 0.25-second pulse.

| Command | Left encoder change | Right encoder change | Result |
| --- | ---: | ---: | --- |
| Left +80, right +80 | positive | positive | Passed |
| Left -80, right -80 | negative | negative | Passed |
| Left +80, right -80 | +16 | -16 | Passed |
| Left -80, right +80 | -17 | +17 | Passed |

The tool encoder remained unchanged during every paired-track test. After a
short settling delay, both track speeds were zero with no running state.

These tests verify command-to-port behavior and encoder signs. They do not yet
label the chassis' physical forward direction because the raised robot's track
direction was not visually recorded.

### Managed EV3 service

- **Success:** Added and deployed `echora-ev3.service` without modifying the
  legacy `/home/robot/track3r` files.
- **Success:** Installed, enabled, and started the service under the unprivileged
  `robot` account.
- **Success:** The service reported `enabled` and `active`; its journal showed
  Jetson connections and a stop after every disconnect.
- **Success:** A final managed-service pulse at left/right +60 for 0.2 seconds
  moved both encoders positively and ended with both speeds at zero.
- **Current state:** `echora-ev3.service` remains enabled and active. Motors are
  stopped. The Jetson client is deployed but is not an automatic service.
- **Scope:** No floor movement was attempted. IR safety remains unavailable and
  explicitly deferred.

## 2026-09-02 20:46 IST — First ROS 2 `/cmd_vel` path deployed

### Implementation

- **Success:** Added differential-drive conversion, a ROS 2 Humble node, and a
  parameter file under `robot/jetson/ev3_bridge` and `config/robot.yaml`.
- **Success:** The node subscribes to `/cmd_vel`, refreshes EV3 commands at
  10 Hz, stops after 300 ms without a fresh ROS command, and publishes raw EV3
  JSON feedback on `/robot_status`.
- **Safety:** The 300 ms Jetson timeout remains inside the EV3's independent
  500 ms watchdog. Motor speed is limited to 120°/s in this layer.
- **Success:** Added an enabled systemd unit, `echora-bridge.service`, running as
  the unprivileged `animesh` user.
- **Success:** The full local suite increased to 23 passing tests.

### Provisional calibration

The checked-in values are temporary: wheel radius 0.03 m, track width 0.12 m,
and motor signs +1/+1. They are sufficient to test the software path but are not
yet measured chassis geometry.

### Raised ROS hardware tests

- **Success:** `/ev3_bridge` appeared in the ROS node graph.
- **Success:** One `/cmd_vel` message with `linear.x=0.03 m/s` commanded both
  tracks positively, moved the encoders, and stopped automatically after the
  300 ms command timeout.
- **Success:** One `/cmd_vel` message with `angular.z=0.5 rad/s` moved the left
  encoder negatively and the right encoder positively, then stopped.
- **Success:** `/robot_status` reported motor positions, zero final speeds, no
  running state, and `last_stop_reason=remote-stop`.

### Fail-fast findings and fixes

- **Observed:** A direct `ev3_client.py status` call timed out while the ROS node
  was active. This is expected because the EV3 service intentionally permits one
  controlling client and the ROS bridge owns that connection. Operational
  status must be read from `/robot_status` while the bridge runs.
- **Failure:** The first Ctrl-C shutdown sent the motor stop successfully but
  then attempted to publish after ROS had invalidated its context.
- **Fix:** Status publishing now returns immediately when `rclpy.ok()` is false.
- **Failure:** The second Ctrl-C test called ROS shutdown twice because Humble's
  signal handler had already shut down the context.
- **Fix:** The finalizer now calls `rclpy.shutdown()` only while the context is
  still active. A third Ctrl-C test exited cleanly without a traceback.

### Managed-service verification

- **Success:** Installed, enabled, and started `echora-bridge.service`.
- **Success:** The service reported `enabled` and `active`, and its journal was
  clean.
- **Success:** A final `/cmd_vel` linear command through the managed service
  advanced both track encoders and ended with both speeds at zero.
- **Current state:** Both `echora-bridge.service` on the Jetson and
  `echora-ev3.service` on the EV3 remain enabled and active. Motors are stopped.
- **Scope:** Tests remained raised-chassis. Metric odometry and floor navigation
  are not yet calibrated.

## 2026-09-02 — Encoder odometry deployed

### Implementation

- **Success:** Added a tested differential encoder integrator using the EV3
  motors' confirmed 360 counts/revolution.
- **Success:** Added calibration helpers for effective wheel radius and track
  width.
- **Success:** The ROS bridge now publishes `/odom`, `/joint_states`, and the
  `odom → base_link` transform in addition to `/robot_status`.
- **Success:** Odometry updates at 5 Hz while commands are active and 2 Hz while
  idle.
- **Safety:** Planar covariance is populated and unobserved vertical/roll/pitch
  axes are marked with high uncertainty.
- **Success:** The full local suite increased to 30 passing tests.

### Deployment and raised tests

- **Success:** Deployed `odometry.py`, the updated `ros_node.py`, and the updated
  parameter file to `/home/animesh/echora`.
- **Success:** Python compilation/import passed and `echora-bridge.service`
  restarted cleanly.
- **Success:** ROS exposed `/odom`, `/joint_states`, and `/tf`.
- **Straight test:** A one-shot `linear.x=0.03 m/s` command advanced provisional
  odometry to approximately `x=0.00733 m`, `y=0.00003 m`, with essentially zero
  yaw. Final reported velocities were zero.
- **Turn test:** A one-shot `angular.z=0.5 rad/s` command changed provisional yaw
  to approximately `0.096 rad` (5.5°) with minimal translation. Final reported
  velocities were zero.
- **TF test:** `tf2_echo odom base_link` matched the `/odom` pose and continued
  updating.
- **Joint-state test:** Left and right track joint positions published in
  radians, with zero velocity after stopping.
- **Current state:** `echora-bridge.service` remains active; motors are stopped.

### Calibration boundary

The current 0.03 m wheel radius and 0.12 m track width remain provisional.
Raised tests validate encoder integration and ROS plumbing, not metric accuracy
on the floor. Accurate values require a measured straight-distance run and a
measured rotation run using the included calibration helpers.

## 2026-09-02 21:35 IST — Stationary USB camera pipeline deployed

Phase 3 milestone: dependable camera acquisition and ROS integration only. No
detection, recognition, SLAM, navigation, recording, or motor behavior was
added. The robot stayed stationary throughout.

### Pre-change verification

- **Success:** `echora-bridge.service` was `enabled` and `active` before any
  change, and `/robot_status` reported all motors stopped.
- **Success:** `lsusb` identified the camera as `0c45:6366` on bus `1-2.3`.
- **Success:** `udevadm` showed `/dev/video0` with `ID_V4L_CAPABILITIES=:capture:`
  and `/dev/video1` with empty capabilities. `/dev/video1` is a metadata node
  and OpenCV cannot open it. `/dev/video0` is the colour stream.
- **Observation:** `v4l2-ctl` and `ffmpeg` are absent, so formats were
  enumerated with raw V4L2 ioctls from Python.

### Failures found and fixed

- **Failure:** A first 30-frame OpenCV capture reported 6.5 fps, and an MJPG
  attempt produced 30 frames with a single distinct content checksum and a mean
  intensity of exactly 10.00 — apparently a frozen, near-black camera.
- **Cause:** Two separate problems. OpenCV negotiates **YUYV by default**, and
  this camera only offers YUYV at **10 fps** at every resolution. Separately,
  the sensor needs roughly two seconds of auto-exposure settling after each
  open; frames captured before that really are dark and duplicated.
- **Fix:** The node requests `MJPG` explicitly and discards frames for
  `warmup_sec` (2.0 s) after every open. With both fixes a 150-frame
  measurement gave 27.36 fps with 150/150 distinct frames and a mean intensity
  of 105.6.
- **Failure:** Filling `sensor_msgs/Image.data` with `frame.tobytes()` measured
  **151.9 ms per frame**, which would have capped the node near 6 fps. A numpy
  array was rejected outright by the rclpy setter with an `AssertionError`.
- **Fix:** The node uses `array.array("B", frame.tobytes())`, measured at
  **0.101 ms per frame**. This was found by benchmarking all three forms on the
  Jetson before writing the node, not after.
- **Failure:** The first foreground Ctrl-C printed
  `Failed to publish log message to rosout: publisher's context is invalid`.
  The same class of bug was fixed in `ros_node.py` in the previous milestone,
  but the camera node's final summary log was still unguarded.
- **Fix:** `shutdown()` now logs through `/rosout` only while `rclpy.ok()` is
  true and falls back to stdout otherwise. The next Ctrl-C exited cleanly.
- **Failure:** A verification check reported a 0.512 s gap in published frames.
- **Cause:** Test artifact, not a node fault. `/camera/image_raw` uses
  best-effort sensor QoS and the verifier copied 921 KB and ran numpy per
  frame, so it dropped frames. The node's own rate was steady at 27.37 fps.
- **Fix:** `CaptureHealth` now measures `max_frame_gap_sec` where frames are
  produced, so publication stability is asserted at the source instead of being
  inferred from what a slow subscriber received. Measured worst gap is 0.040 s,
  one frame period.

### Implementation

- **Success:** Added `robot/jetson/camera` with a ROS 2 node plus three
  hardware-free modules: `camera_config.py`, `frame_health.py`, and
  `camera_calibration.py`.
- **Success:** Publishes `/camera/image_raw` (`sensor_msgs/msg/Image`, `bgr8`),
  `/camera/camera_info` (`sensor_msgs/msg/CameraInfo`), and `/camera/status`
  (JSON health). Image and CameraInfo are published in one call, so they always
  share a timestamp and `frame_id`.
- **Success:** Image topics use best-effort sensor QoS; status uses reliable
  transient-local so a late subscriber sees health immediately.
- **Success:** Device, width, height, requested rate, fourcc, frame ID,
  reconnect interval, warm-up, failure budget, status period, and calibration
  file are all parameters in `config/camera.yaml`.
- **Success:** The local suite grew from 30 to **101 passing tests**.

### Measured camera capability

| Format | Resolution | Advertised | Measured |
| --- | --- | ---: | ---: |
| MJPG | 640×480 | 30 fps | **27.3 fps** |
| YUYV | 640×480 | 10 fps | 10 fps ceiling |

The honest sustained figure for a requested 30 fps is **27.3 fps**, about 91%
of nominal. Node-side publication rate held between 27.34 and 27.39 fps across
every run, with a worst inter-frame gap of 0.040 s.

### Acceptance tests

- **Success:** 300 published frames validated: strictly increasing ROS
  timestamps, `camera_optical_frame`, 640×480, `bgr8`, step 1920, 921600-byte
  payloads, 300/300 distinct frames, none empty or black.
- **Success:** Every image timestamp had a matching CameraInfo timestamp except
  the final frame, whose CameraInfo had not yet arrived when the verifier
  stopped. CameraInfo frame ID and size matched the image.
- **Success:** A published frame was saved and decoded to a usable 480×640×3
  colour image of the ceiling and its light fittings, with differing channel
  means.
- **Success:** Delivered rate to a lightweight subscriber was 27.09 fps
  (500 images, 505 CameraInfo, 18.4 s).
- **Success:** Five-minute continuous run under the managed service. RSS went
  from 162000 kB to 161628 kB — it fell slightly, so there is no memory growth.
  Rate stayed 27.34–27.39 fps, zero read failures, and the journal was clean.
- **Success:** Stopping the service released `/dev/video0` with no holder;
  starting it reacquired the device and resumed publishing.
- **Success:** Camera-loss recovery was tested by deauthorizing USB `1-2.3`,
  which removes `/dev/video*` much as unplugging does. The node reported the
  failure, released the device after exactly 15 consecutive failures, retried
  every 2.0 s, then reopened and resumed automatically. This was exercised four
  times, three in the foreground and once under the managed service.
- **Success:** No busy loop. CPU while disconnected was **1.5% of one core**
  against 24.6% while streaming.
- **Success:** Under the managed service, an outage left the unit `active` with
  `NRestarts=0`, confirming the node recovered internally rather than being
  restarted by systemd.
- **Success:** `echora-camera.service` installed, enabled, and active.

### Calibration limitation

The camera is **not calibrated** and no calibration file exists. The node
publishes an explicitly uncalibrated `CameraInfo` with `D`, `K`, `R`, and `P`
all zeroed and an empty `distortion_model`, which is the documented
`sensor_msgs/CameraInfo` marker for an uncalibrated camera. `/camera/status`
reports `"calibrated": false` with reason `no_calibration_file_configured`. No
intrinsic values were invented. A calibration file is also refused, with its
reason reported, when it is missing, unparseable, zeroed, or recorded at a
different resolution. Nothing in this milestone supports metric vision.

### Motor safety

- **Success:** Motors remained stopped for the entire milestone. Encoder
  positions were left 46, right 92, tool 76 at the start and identical at the
  end, with every speed zero and `motion_active` false.
- **Success:** `echora-bridge.service` stayed `enabled` and `active` with
  `NRestarts=0`. No `/cmd_vel` message was ever sent.
- **Observation:** The bridge journal shows intermittent
  `EV3 status failed: timed out` errors. These are **not caused by the camera**:
  the worst cluster, at 15:41:57–15:42:01 UTC, predates the first camera node
  run at 15:46:06 UTC; there were five such errors in the 18 minutes before any
  camera existed against four in the 19 minutes after; the Jetson's Wi-Fi is
  PCI (`rtl88x2ce`) so it shares no bus with the USB camera; and a 20-packet
  ping to the EV3 while the camera streamed showed 0% loss at 3.9 ms average.
  They are intermittent timeouts against a 300 MHz EV3 over Wi-Fi, and the
  bridge handles them by reconnecting.

### Deployed state

| Path | Purpose |
| --- | --- |
| `/home/animesh/echora/camera_node.py` | ROS 2 camera node |
| `/home/animesh/echora/camera_config.py` | Parameter validation |
| `/home/animesh/echora/frame_health.py` | Frame validation and recovery bookkeeping |
| `/home/animesh/echora/camera_calibration.py` | Calibration loading |
| `/home/animesh/echora/camera.yaml` | Deployed parameters |
| `/etc/systemd/system/echora-camera.service` | Managed unit |

All deployed files were SHA-256 verified against the repository copies.
`echora-camera.service` and `echora-bridge.service` are both enabled and
active. Motors are stopped.

### Not done in this milestone

Camera calibration, person detection, face recognition, SLAM, navigation,
autonomous movement, and recording were all deliberately excluded. The camera
head motor was not moved.
