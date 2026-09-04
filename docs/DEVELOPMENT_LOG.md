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

## 2026-09-02 17:45 IST — Stationary person detection deployed

Phase 3 milestone: generic person detection from the live camera stream, with
the robot stationary. No face detection, recognition, identity, tracking,
following, navigation, SLAM, or motor movement was added. Motors stayed
stopped throughout.

### Pre-change environment survey

Nothing was installed before the environment was inspected.

| Component | Found |
| --- | --- |
| Platform | Jetson Orin Nano, L4T R36.4.3 (JetPack 6.2), MAXN_SUPER |
| Python | 3.10.12 |
| CUDA | 12.6, cuDNN 9.3 |
| TensorRT | **10.3.0.30** with Python bindings and ONNX parser |
| PyTorch | **2.6.0+cpu — CPU-only wheel** |
| OpenCV | 4.5.4, `cv2.cuda.getCudaEnabledDeviceCount()` = 0 |
| `vision_msgs` | not installed |
| `trtexec` | absent |

- **Finding:** the installed PyTorch offers **no GPU acceleration at all**;
  `torch.cuda.is_available()` is `False`. CUDA itself is healthy — device 0 is
  `Orin`, 8 SMs, 7.99 GB unified memory. TensorRT was therefore chosen as the
  acceleration path, which is also faster than PyTorch-CUDA would have been.
- **Finding:** neither `pycuda` nor `cuda-python` was present, and TensorRT's
  Python API cannot allocate device memory without one.
- **Note:** the Orin Nano has **unified memory**, not separate VRAM; the 8 GB
  is shared with the system.

### Failures found and fixed

- **Failure:** `apt-get install ros-humble-vision-msgs` returned 404, and
  `apt-get update` refused to refresh the index.
- **Cause:** the Open Robotics apt signing key had **expired on 2025-06-01**
  (`EXPKEYSIG F42ED6FBAB17C654`), so the stale index still pointed at a
  withdrawn build (`4.1.1-1jammy.20250325.224255`) that no longer exists in the
  pool.
- **Fix:** installed the renewed key from `ros/rosdistro`. Same key ID
  `C1CF 6E31 E6BA DE88 68B1 72B4 F42E D6FB AB17 C654`, expiry extended to
  2030-06-01 — a genuine renewal, not a different key. The previous keyring was
  backed up. `ros-humble-vision-msgs` 4.1.1 then installed normally.
- **Note:** `/etc/apt/sources.list.d/ros-latest.list` points at the ROS 1
  repository, which has no `jammy` release and errors on every `apt-get
  update`. Left alone as out of scope, but it should be removed.

- **Failure:** the first engine reported `provider=tensorrt_fp32` despite being
  built with `--fp16`.
- **Cause:** the provider was inferred from a string search of the engine
  inspector's `ONELINE` output, which contains only layer names. Worse, the
  JSON output also reported `Precision: UNKNOWN` for every layer because the
  engine had not been built with detailed profiling verbosity.
- **Fix:** `build_engine.py` now sets `ProfilingVerbosity.DETAILED` and reads
  each tensor's `Format/Datatype`, which is where TensorRT actually records
  precision. The deployed engine measures **274 FP16 tensors against 21 FP32**,
  so `fp16` is now asserted from the built artifact rather than from the flag
  passed to the builder. The result is written to a `<engine>.json` sidecar and
  the node reads it; if precision cannot be confirmed the node reports plain
  `tensorrt` rather than guessing.

- **Failure:** the first `RollingLatency.percentile` implementation returned
  0.096 where nearest-rank p95 of 1..100 ms is 0.095.
- **Cause:** `round(fraction * n + 0.5)` hits Python's banker's rounding on an
  exact `.5`, so `round(95.5)` is 96.
- **Fix:** switched to `ceil(fraction * n)`. Verified p50/p95/p99/p100 =
  0.050/0.095/0.099/0.100.

- **Failure:** SIGINT produced an `RCLError: failed to initialize wait set: the
  given context is not valid` traceback and a non-zero exit, even though the
  shutdown summary printed correctly.
- **Cause:** the rclpy signal handler shuts down the context, and the executor
  then fails inside `WaitSet` construction. That surfaces as `RCLError`, not
  `KeyboardInterrupt`, so the existing handler missed it. With
  `Restart=on-failure` this would have looked like a crash on every clean stop.
- **Fix:** `main()` catches `ExternalShutdownException` and treats any
  exception raised once `rclpy.ok()` is false as a normal shutdown, re-raising
  anything else. Verified: exit 0, zero tracebacks.

### Hardware failures found during testing

- **Failure:** the first full person-test recording produced **zero detections
  across 2646 frames** while a person was posing in front of the camera.
- **Cause:** not a software fault. The camera was pointed at the ceiling in an
  unlit room. Every one of the 921,600 pixels sat in the darkest histogram bin
  with a maximum value of 23/255; brightening 12x revealed only a light
  fitting. The detector was correctly reporting that no person was visible.
- **Fix:** owner turned on the room light and re-aimed the camera by hand. The
  camera head is on EV3 motor A and was **not** moved by software, as this
  milestone forbids motor movement. Scene mean intensity went from 10.4 to
  174.8.
- **Lesson:** the camera node's own health reporting did not make this
  obvious. `/camera/status` reported `state: streaming` at 27.37 fps with zero
  read failures while publishing black frames. `mean_intensity` was in the
  payload and read 10.0, but nothing treats a persistently dark or duplicated
  stream as a fault.

- **Failure:** the camera froze during testing — `mean_intensity` exactly 10.0
  with 6508 duplicate frames out of 19086 published, and later 49 *consecutive*
  identical frames, while the node still reported healthy streaming.
- **Cause:** triggered by stopping and starting `echora-camera.service` during
  the camera-interruption test. A service restart alone did not clear it.
- **Fix:** deauthorizing and reauthorizing the USB device cleared the freeze
  (`duplicate_frames` back to 0).

- **Failure:** the camera then stopped enumerating entirely. `lsusb` lost
  `0c45:6366`, no `/dev/video*` nodes, and the camera node retried open 30
  times.
- **Cause:** physical. The kernel reported `device not accepting address, error
  -71` on every control transfer, `Failed to initialize the device (-5)`, and
  `usb 1-2-port3: Cannot enable. Maybe the USB cable is bad?`. This boot
  recorded **24 connect attempts against 19 disconnects**.
- **Fix:** moving the camera to a different downstream port — **1-2.1 instead
  of the faulty 1-2.3** — resolved it completely: clean enumeration, no `-71`
  errors, 9/9 stable samples over 45 s. The fault was the port, not the camera.
- **Note:** the Orin Nano's four USB-A ports all sit behind an internal Realtek
  hub, so there is no true "direct" USB-A path that bypasses it.

- **Observation:** the detector behaved correctly through every one of these
  outages — reported `stale_input`, logged once, never busy-looped, never
  exited, and resumed automatically each time frames returned. The hardware
  faults exercised the recovery requirements more thoroughly than the planned
  test did.

### Model selection

| | YOLOX-tiny | **YOLOX-s (chosen)** |
| --- | ---: | ---: |
| Input | 416x416 | 640x640 |
| COCO mAP | 32.8 | **40.5** |
| GPU inference | 10.68 ms | 17.07 ms |
| Total per frame | 15.75 ms | **23.65 ms** |
| Ceiling | 63.5 fps | **42.3 fps** |
| Engine build | 349 s | 440 s |

Both were downloaded from the official Megvii YOLOX `0.1.1rc0` GitHub release
and compiled to FP16 TensorRT engines on the Jetson. YOLOX-s was chosen because
42 fps is far above both the camera rate and the 15 Hz inference cap, so the
extra accuracy is free. Checksums, sizes, and licence are recorded in
`robot/jetson/perception/README.md`.

**Ultralytics YOLOv8/v11 was deliberately rejected**: it is AGPL-3.0, which
would attach copyleft obligations to anything the project later distributes or
exposes as a network service. YOLOX is Apache-2.0.

The exported ONNX outputs **raw, undecoded** predictions, so the YOLOX anchor
decode is implemented here. It was verified against the canonical YOLOX
`dog.jpg`: YOLOX-s returns bicycle 0.954, dog 0.913, truck 0.613, car 0.565 in
correct source-image coordinates, and correctly zero persons. The anchor grid
sizes match the ONNX outputs exactly (3549 for 416, 8400 for 640).

### Implementation

- **Success:** added `robot/jetson/perception` with the ROS node plus five
  hardware-free modules — `detector_config.py`, `detections.py`,
  `detector_health.py`, `image_intake.py`, `messages.py` — following the
  camera milestone's split so the logic is testable on the Mac.
- **Success:** `inference.py` provides a TensorRT backend and an OpenCV-DNN CPU
  fallback over the same ONNX file, and is importable without TensorRT so its
  failure paths are tested.
- **Success:** the subscription callback only validates and stores into a
  **one-deep slot**; a timer runs inference. Queue depth is one by
  construction, so an unbounded backlog is impossible.
- **Success:** the local suite grew from **101 to 291 passing tests**.

### Measured performance

Jetson Orin Nano, MAXN_SUPER, 640x480 `bgr8`, YOLOX-s FP16:

| Stage | Mean | p95 | Max |
| --- | ---: | ---: | ---: |
| Preprocess | 2.98 ms | 3.01 ms | 4.66 ms |
| **GPU inference** | **17.07 ms** | 17.19 ms | 17.46 ms |
| Decode | 3.60 ms | 3.77 ms | 3.87 ms |
| **Total** | **23.65 ms** | 23.89 ms | 25.13 ms |

Under the managed service with the camera also running, end-to-end latency
measured **28.4-28.7 ms mean** and **28.6-29.0 ms p95** — higher than the
isolated benchmark because the camera node competes for the same SoC.

- Model load **0.42 s**, warm-up **0.11 s**, once at startup.
- CPU **15-24% of one core**; RSS ~333 MB (406 MB measured earlier under
  YOLOX-s in the foreground with a subscriber attached).
- GPU confirmed with `tegrastats`: `GR3D_FREQ` **20-56%** while detecting.
- Provider reported and verified as **`tensorrt_fp16` on `Orin`**.

### Detection accuracy

Live camera, normal indoor lighting, YOLOX-s FP16, confidence 0.45. Each phase
measured separately with the operator watching a temporary preview stream so
positions could be held accurately.

| Phase | Frames with a correct detection | Confidence | Median box |
| --- | ---: | --- | --- |
| Near (~1 m) | **277 / 277 = 100%** | 0.85-0.92 | 396 px high |
| Medium (~2.5 m) | **297 / 297 = 100%** | 0.84-0.95 | 256 px high |
| Far (room depth) | **223 / 223 = 100%** | 0.90-0.92 | 102 px wide |
| Partially visible | **290 / 290 = 100%** | 0.59-0.94 | clipped at 479/480 px |
| Two people | **295 / 295 = 100%** | 0.91 | exactly 2 every frame |
| Empty scene, 90 s | **0 false positives / 1304** | — | — |

The requirement was 80% of processed frames; the measured figure is **100% in
every pose**. The two-person histogram was `{2: 295}` — never 1, never 3 — so
overlapping people were kept distinct and not merged by NMS. Boxes were
verified visually on saved annotated frames at each distance.

- **Correction during testing:** a first empty-scene run reported 5.14% false
  positives (66/1285). Inspecting a captured frame showed a person's torso
  filling the entire image — the room was not actually empty and those
  detections were **correct**. Re-run with a 15 s settling period on a
  genuinely empty room, the figure is **zero**. The first number was not
  reported as a false-positive rate.

### Acceptance tests

- **Success:** `echora-camera.service` enabled and active, `/camera/image_raw`
  publishing, before the detector was started.
- **Success:** ROS exposes `/perception/person_detections`
  (`vision_msgs/msg/Detection2DArray`), `/perception/person_image`
  (`sensor_msgs/msg/Image`), and `/perception/status`.
- **Success:** across 1081 verified messages, every detection carried the
  source image's timestamp and `camera_optical_frame`; zero had a wrong frame
  ID and zero inner `Detection2D` headers disagreed with the array header.
- **Success:** annotated images were 640x480 `bgr8` matching the source
  exactly, with correct step and payload size on all 1064 checked.
- **Success:** `annotated_published` stayed at 0 with no subscriber attached,
  confirming the copy and drawing are skipped when nobody is listening.
- **Success:** camera stopped mid-run — detector stayed alive, reported
  `stale_input` with `last_frame_age_sec` 13.9, logged exactly one warning, and
  did not busy-loop. Camera restarted — returned to `detecting` at 0.045 s
  frame age with zero inference errors. Exercised repeatedly, including during
  the unplanned USB faults.
- **Success:** clean SIGINT stop and restart, exit 0, no traceback.
- **Success:** `echora-person-detector.service` installed, **enabled and
  active**, `NRestarts=0`, clean journal.
- **Success:** **ten-minute continuous soak** under the managed service.
  RSS went 333,120 kB to 334,204 kB (**+0.33%**) and plateaued — the final five
  30-second samples were byte-identical — so memory does not grow continuously.
  Frame age stayed between 0.031 s and 0.087 s and ended at 0.040 s, with no
  upward trend, so published frames do not become progressively stale.
  **Zero inference errors** across 8,159 inferences (1,624 frames dropped by
  the newest-frame slot, 0 rejected). CPU held 15-26% of one core.
- **Observation:** input rate varied between 9.2 and 18.3 Hz during the soak.
  This is the **camera**, not the detector: `/camera/status` accumulated 60
  read failures and the scene darkened (`mean_intensity` fell from 174.8 to
  91.0), which lengthens exposure. The detector tracked the varying input
  without error and recovered from each dip on its own.
- **Success:** all three units — `echora-camera`, `echora-bridge`,
  `echora-person-detector` — enabled, active, `NRestarts=0`. The detector
  journal contains zero error or traceback lines.

### Motor safety

The EV3 was **powered off and unreachable** for this milestone (100% packet
loss to `192.168.1.25`; the bridge logged 5,014 consecutive status failures),
so encoder positions **could not be read** from `/robot_status` and are not
claimed. What was verified instead:

- `/cmd_vel` has **zero publishers**, so nothing commanded motion.
- `ros2 node info /echora_person_detector` shows its only publishers are
  `/perception/person_detections`, `/perception/person_image`,
  `/perception/status`, plus `/rosout` and `/parameter_events`. The node has
  **no motor publisher at all** and is structurally incapable of commanding the
  EV3.
- No `/cmd_vel` message was published at any point during this milestone.

Motors therefore could not have moved. Encoder confirmation should be repeated
when the EV3 is next powered on.

### Deployed state

| Path | Purpose |
| --- | --- |
| `/home/animesh/echora/person_detector.py` | ROS 2 detector node |
| `/home/animesh/echora/inference.py` | TensorRT and CPU backends |
| `/home/animesh/echora/detections.py` | Geometry, filtering, NMS |
| `/home/animesh/echora/detector_config.py` | Parameter validation |
| `/home/animesh/echora/detector_health.py` | Rates, latency, status |
| `/home/animesh/echora/image_intake.py` | Frame acceptance rules |
| `/home/animesh/echora/messages.py` | Message assembly |
| `/home/animesh/echora/build_engine.py` | ONNX to TensorRT compiler |
| `/home/animesh/echora/perception.yaml` | Deployed parameters |
| `/home/animesh/echora/models/` | ONNX and engines (**not** in Git) |
| `/etc/systemd/system/echora-person-detector.service` | Managed unit |

System changes made and documented: `ros-humble-vision-msgs` 4.1.1 installed,
`cuda-python==12.6.2.post1` installed via `pip --user`, and the expired ROS apt
signing key replaced with its renewal.

### Privacy and safety

- No camera frame is written to disk by the node, by default or otherwise.
- No face detection, face recognition, embedding, or biometric processing
  exists in this milestone, and no such data is retained.
- Diagnostic images captured during testing were written to `/tmp` on the
  Jetson and to the session scratch directory only. None are committed;
  `.onnx`, `.engine`, `.engine.json` and `models/` are gitignored.
- A temporary LAN preview server was used so the operator could position
  themselves; it held nothing on disk and was stopped when testing finished.

### Limitations

- The camera remains **uncalibrated**, so `ObjectHypothesisWithPose.pose` is
  published zeroed and detections are pixel coordinates only. Nothing here
  supports metric or 3D reasoning.
- Accuracy was measured in **one room under one lighting condition** with two
  people. It is not a general accuracy claim.
- The camera's frame rate is lighting-dependent: 27.3 fps was recorded against
  a bright ceiling in the previous milestone, against 16-18 fps in this
  room-facing view, since the sensor lengthens exposure in dimmer scenes.
- USB port `1-2.3` is faulty and should not be used for the camera.
- The camera node reports `streaming` while publishing frozen black frames.
  Treating a persistently dark or duplicated stream as a fault is worth adding
  to that node, and is not done here.

## 2026-09-03 — ChArUco camera-calibration tooling deployed

### Technology choice and implementation

- **Choice:** OpenCV ChArUco (`DICT_5X5_100`, 5×7 squares) was selected over a
  plain checkerboard because identified corners and partial-board support make
  real-world collection more robust. Output remains standard ROS 2
  `sensor_msgs/CameraInfo` using the `plumb_bob` model.
- **Success:** Added the exact 2000×2800 printable target, board generator,
  ROS image-topic collector, diverse-view selection, reprojection outlier
  pruning, atomic ROS calibration YAML writer, and a JSON report for every
  successful or failed attempt.
- **Safety:** The collector subscribes only to `/camera/image_raw`; it has no
  EV3 connection, `/cmd_vel` publisher, or motor-control path.
- **Acceptance boundary:** A result is not written unless at least 20 views
  remain, RMS reprojection error is no more than 1.0 px, the worst view is no
  more than 1.5 px, all values are finite, focal lengths are plausible, and the
  principal point lies inside the image.

### Tests and deployment

- **Success:** The complete Mac suite now passes **300 tests**, including new
  board validation, view-diversity, angle-wrap, and calibration-result rejection
  tests.
- **Success:** On the Jetson, OpenCV 4.5.4 exposes `aruco`,
  `CharucoBoard_create`, `interpolateCornersCharuco`, and
  `calibrateCameraCharucoExtended`.
- **Success:** The generated target was detected as all **24 ChArUco corners**.
- **Success:** The synthetic end-to-end Jetson test detected 36/36 rendered
  views, calibrated at **0.354 px RMS**, and recovered focal lengths within
  **0.35% (fx)** and **0.25% (fy)** of the known camera model.
- **Success:** `charuco_config.py`, `generate_charuco_board.py`,
  `calibrate_charuco.py`, `self_test_charuco.py`, and the updated `camera.yaml`
  are deployed under `/home/animesh/echora`.
- **Success:** After the configuration restart, both `echora-camera.service`
  and `echora-person-detector.service` remained active. Camera status honestly
  reports `calibration_file_not_found` until capture succeeds.
- **Expected failure verified:** A four-second live run with no board visible
  processed 100 frames, collected 0 views, exited 2, wrote a failure report,
  and did **not** write an invalid calibration file.

### Remaining physical step at this checkpoint

The USB camera still has no accepted real intrinsics. Present the printed or
displayed ChArUco board at 30 varied positions/tilts while the collector runs,
then restart the camera service and verify loaded CameraInfo plus live
undistortion. Face detection must wait until this physical acceptance test is
complete.

## 2026-09-03 — Physical camera calibration completed

### Preview and first capture

- **Failure:** The earlier temporary `preview_server.py` was no longer present
  on the Jetson, so the operator initially had no live positioning view.
- **Success:** Added a no-recording MJPEG preview showing detected ChArUco
  corners, board coverage, and accepted-view count at
  `http://192.168.1.48:8080/`.
- **Failure:** The first preview process exited with ROS
  `ExternalShutdownException`. It was fixed to shut down cleanly and installed
  as restartable `echora-preview.service`. The unit is intentionally static and
  not enabled at boot because the LAN stream is unauthenticated.
- **Failure caught by visual verification:** The first 30-view solve appeared
  good numerically (0.580 px RMS, 1.228 px worst retained view) but produced
  severe circular rectification distortion. It retained only 244×198 pixels,
  or **15.7%** of the image, and had an unstable k3 of -0.630. The model was
  immediately removed from the active path, camera publication returned to
  explicitly uncalibrated data, and the YAML/report were retained under
  `/home/animesh/echora/rejected_calibrations/`.

### Correction and second capture

- **Fix:** k3 is now fixed at zero during calibration. Acceptance now also
  checks full-FOV rectified ROI, focal uncertainty, whole-sensor corner spread,
  3×3-grid coverage, and the number of close views. This prevents a low RMS
  score from hiding a destructive lens model.
- **Success:** The Mac suite passes **305 tests**, including rejection of the
  exact 244×198 destructive rectification case and poorly distributed datasets.
- **Success:** The Jetson synthetic test still passes all 36 views at 0.354 px
  RMS after the model constraint was added.
- **Failure understood:** Applying the live rectified-ROI gate to the synthetic
  renderer rejected its interpolation artifacts as if they were lens
  distortion. The synthetic test remains scoped to marker detection and known
  focal recovery; rectified area is checked only on real camera frames.
- **Observation:** The second 30-view capture was conservatively rejected by
  the automatic dataset gate because corners did not quite reach both vertical
  extremes and only four rather than six views exceeded 12% coverage. The
  solved lens model itself was stable: k3=0, 0.596 px RMS, 1.351 px worst view,
  mild coefficients, and a 609×450 valid ROI (**89.2%** of the image).
- **Success:** A temporary live deployment then passed 30/30 exact timestamp
  matches between `/camera/image_raw` and `/camera/camera_info`; all matrices
  and coefficients were finite and correctly shaped. The raw-versus-rectified
  image showed normal mild correction, no circular warping, and only narrow
  full-FOV borders. This direct validation justified promoting the model despite
  the two conservative capture-distribution warnings.

### Final deployed state

- **Success:** Accepted intrinsics are committed in
  `config/camera_calibration.yaml` and loaded from
  `/home/animesh/echora/camera_calibration.yaml`.
- **Success:** `/camera/status` reports `calibrated: true` and
  `/camera/camera_info` publishes fx 416.371, fy 413.608, cx 338.723, cy 235.303
  plus five `plumb_bob` coefficients at 640×480.
- **Success:** `echora-camera.service`, `echora-person-detector.service`, and
  `echora-bridge.service` are active. The temporary preview is stopped.
- **Privacy:** Comparison frames existed only in `/tmp` during review and were
  deleted. No camera frame was committed or retained by the preview.
- **Motor safety:** Calibration and preview code have no EV3 connection or
  `/cmd_vel` publisher; no motor command was sent.

## 2026-09-03 — GPU face detection deployed and live-tested

### Technology and boundaries

- **Choice changed after license verification:** SCRFD-2.5G was the initial
  plan, but InsightFace's official pretrained weights are restricted to
  non-commercial research. YuNet 2023mar from OpenCV Zoo was selected instead:
  its model is MIT-licensed, provides five landmarks, and is only 232,589 bytes.
- **Success:** The Orin parsed all 12 YuNet outputs and built a 559,156-byte
  TensorRT engine in 137.9 seconds. Engine inspection measured 98 FP16 and 64
  FP32 tensors, so the service reports `tensorrt_fp16` from evidence rather
  than the requested build flag.
- **Scope:** Face detection only. No identity, recognition, embeddings,
  enrollment, tracking, disk storage, EV3 connection, or motor command exists
  in this stage.

### Implementation failures caught and corrected

- **Failure:** The first implementation resized every upper-person crop
  directly to a square. The live camera's person box was roughly 455×254, so
  this distorted the face and made detection intermittent even though a direct
  full-frame test scored the same face above 0.91.
- **Fix:** Person crops now use aspect-preserving top-left letterboxing. The
  corrected live preview immediately showed a 0.91 face box with five
  correctly placed landmarks; a later partial/profile view scored 0.79.
- **Failure:** The first synchronizer assumed the camera callback always
  arrived before its YOLOX result. DDS sometimes delivered them in the other
  order, causing valid exact-frame pairs to be dropped and throughput to fall
  to roughly 4.5 Hz.
- **Fix:** A two-sided bounded exact-timestamp matcher now accepts either
  callback order. Unmatched images and person messages are independently
  bounded and reported rather than growing without limit.
- **Expected observer failure:** One verification run saw 36 of 37 face stamps
  in its own best-effort camera subscription while all 37 matched the reliable
  person stream. The face service had the source frame; the verifier itself
  dropped one best-effort sample. Its acceptance rule now tolerates up to 10%
  observer loss while requiring at least 90% camera observation and 95% person
  observation.
- **Performance finding:** Subscribing to full annotated ROS images reduces
  throughput because converting a 640×480 raw frame is CPU-heavy. Only the
  diagnostic image is now capped to 3 Hz; structured results keep their own
  10 Hz ceiling.

### Acceptance evidence

- **Success:** A live ten-second run with one visible face received 176 camera
  messages, 123 person messages, and 50 face messages. All 50 contained one
  valid face, all 50 timestamps were observed in both source streams, the
  provider was `tensorrt_fp16`, and inference errors were zero.
- **Success:** With preview disabled, 89 face messages arrived in the first
  ten-second check and all 89 contained the visible face. After the final
  restart, status measured the full configured **10.01 Hz** across 99
  inferences with one current face, 18.09 ms mean inference plus decode, and
  zero errors. With the 3 Hz operator preview active, the structured stream
  remained around 5-7 Hz.
- **Success:** A temporary OpenCV sample photograph was published only for a
  bounded end-to-end test and produced face detections through YOLOX → exact
  timestamp match → YuNet. The image was never added to Git.
- **Success:** `tegrastats` independently observed `GR3D_FREQ` peaks of 44%,
  61%, 33%, and 57% while both detectors were active, confirming GPU work.
- **Recovery:** Stopping `echora-person-detector.service` left the face service
  active and changed `/perception/face_status` to `stale_input`. Restarting the
  upstream service restored `detecting`, one live face, and zero errors without
  restarting the camera or face service.
- **Tests:** The development Mac passes **333 tests**; four NumPy decoder tests
  are skipped in its deliberately lightweight Python environment. The same
  decode path loaded the real Orin engine, decoded live faces, and passed the
  end-to-end verifier.
- **Reboot recovery:** A later Jetson reboot automatically restored the camera,
  YOLOX person detector, YuNet face detector, and EV3 bridge. The face node
  briefly reported stale input while clocks and streams settled, then returned
  to `detecting`. The unauthenticated preview correctly remained inactive.
- **Deployed state:** Camera, YOLOX person detector, YuNet face detector, and
  EV3 bridge are active. The face detector is enabled at boot; the temporary
  preview is static, inactive, and not enabled.

## 2026-09-03 — Target enrollment and GPU recognition deployed

### Model and privacy decision

- **Owner scope confirmed:** This robot will remain personal and
  non-commercial, so InsightFace public pretrained weights may be used under
  their non-commercial research terms. The restriction is now explicit in
  `AGENTS.md`, configuration comments, and perception documentation.
- **Model selected:** AntelopeV2 `glintr100`, the official pack's ResNet-100
  recognizer trained on Glint360K. The 343 MB official archive matched SHA-256
  `8e182f14fc6e80b3bfa375b33eb6cff7ee05d8ef7633e738d1c89021dcf0c5c5`;
  the extracted ONNX matched
  `4ab1d6435d639628a6f3e5008dd4f929edf4c4124b1a7169e1048f9fef534cdf`.
- **Privacy:** The default store contains only normalized embeddings, is
  atomic, model-checksum-bound, and private (0700 directory, 0600 file).
  Opt-in photo retention keeps only aligned 112×112 face crops. Original
  uploads and room frames are never stored. Delete removes both templates and
  retained crops.

### Implementation and failures fixed

- **Failure:** The original TensorRT builder rejected AntelopeV2 because its
  ONNX uses dynamic batch (`-1×3×112×112`) and no optimization profile existed.
  The builder now pins only the batch dimension, rejects dynamic spatial axes,
  and records the fixed batch in its engine metadata.
- **Success:** The Orin built a 131,373,148-byte engine in 93.5 seconds. Its
  SHA-256 is `94bc49a39a76ed9cab5547bcba129757b71323f89b267021c74f04208ab5d2c1`;
  inspection measured 829 FP16 tensors and one FP32 tensor.
- **Failure:** OpenCV 4.5.4 exposed `FaceDetectorYN` but failed while executing
  the 2023 YuNet ONNX for uploaded photos (`Layer with requested id=-1 not
  found`). Uploads now use the same tested YuNet TensorRT decoder as live face
  detection; no risky system-wide OpenCV replacement was needed.
- **Failure:** A managed face-node restart produced a false exit failure by
  publishing a final status after the ROS context was invalid. Shutdown no
  longer publishes on the dead context; the next restart exited cleanly and
  systemd reported `Deactivated successfully`.
- **Test-method failure:** Injecting a second temporary publisher onto the live
  camera topic caused the person and face subscribers to go stale after that
  publisher exited, while the camera itself continued streaming. Restarting
  only those two consumers restored the chain. Future acceptance avoids a
  competing publisher on the production camera topic.

### Enrollment and recognition behavior

- **Success:** Added a browser enrollment console with live preview, guided
  front/left/right collection, uploaded-photo support, explicit consent,
  embeddings-only or aligned-crop retention, quality rejection, duplicate
  rejection, and within-session identity consistency checks.
- **Success:** Added exact-frame transient landmark transport, ArcFace
  alignment, 512-value normalized embeddings, multi-template scoring,
  conservative 0.45 threshold, and three-of-five temporal confirmation.
- **Success:** `echora-target-recognizer.service` is enabled and active. It
  publishes `/perception/target_matches` and `/perception/recognition_status`
  and currently reports `not_enrolled`, which is correct before the real target
  is collected. The temporary `echora-enrollment-console.service` is active for
  the operator but remains static/not enabled at boot.
- **Success:** The browser UI and live stream were visually inspected: the
  current face box and five landmarks were correctly positioned, all controls
  rendered, status updated, and the browser logged no errors.

### Acceptance evidence and remaining physical test

- **Success:** Deployed public-image checks measured 0.9824 similarity for the
  same face after a brightness change and 0.0250 for a different identity.
  Embedding latency over 30 runs was 14.37 ms mean, 23.0 ms p95, 26.0 ms max.
- **Success:** The full ROS path produced target matches on 3/3 observed
  known-person test frames (best 0.9713) and zero matches on 3/3 different-
  person frames. The temporary identity, archive, and public images were then
  deleted; no test biometric remains.
- **Tests:** The Mac suite passes **350 tests**, four skipped NumPy decoder
  tests remain exercised on the Jetson.
- **Safety:** Recognition and enrollment contain no EV3 client and publish no
  `/cmd_vel`; motors remained stopped.
- **Next physical test:** Enroll the actual target through the open console,
  then measure acceptance for front/left/right, different light, distance, and
  glasses as applicable, followed by at least one unknown person to check false
  acceptance.

### Real target enrolled and known-person acceptance passed

- **Success:** The operator completed enrollment with 18 live samples: nine
  front, six left, and three right. No uploaded photograph was needed. The
  stored model ID and ONNX checksum match the deployed AntelopeV2 recognizer.
- **Retention:** The operator explicitly selected aligned-crop retention. The
  target JSON is mode 0600 inside a mode 0700 directory; all 18 retained
  112×112 crops are also mode 0600. No full camera frame was retained.
- **Immediate recognition:** The running service loaded the enrollment without
  a restart and reported five hits in its five-frame window, similarity 0.8643,
  `target_confirmed`, and zero inference errors.
- **15-second acceptance:** 102 target-match messages were observed and all
  102 contained the target. Similarity ranged from 0.9364 to 0.9616 with a
  0.9502 mean. All seven sampled health reports remained
  `target_confirmed`; inference errors remained zero.
- **Invalid first rejection attempt:** The nominal unknown-person window still
  showed the enrolled target. The system matched that face at 0.7245–0.9094;
  a temporary diagnostic frame and one retained enrollment crop confirmed it
  was the same identity. Both temporary diagnostic copies were deleted, and no
  threshold was changed based on the invalid trial.
- **Incomplete second rejection attempt:** A face appeared in only 37/144
  frames while the camera transitioned from the enrolled target to an empty
  view. Because the person did not remain visible for the test window, those
  readings were not accepted as unknown-person evidence.
- **Real unknown-person rejection passed:** The verifier waited for a stable
  face before starting. During the resulting 15-second window, a different
  real person was detected in 148/148 face frames and 119 recognition frames
  were correlated to those detections. There were zero target detections.
  Similarity was 0.1014–0.1666 (0.1334 mean), safely below the configured 0.45
  threshold. All seven health reports stayed `searching`; inference errors
  remained zero.
- **Still required:** Recheck the enrolled target at farther distance and under
  changed indoor lighting before recognition is considered physically
  complete.

## 2026-09-03 — Phase 4 odometry floor calibration started

### Implementation

- **Success:** Added a bounded ROS calibration runner that captures encoder and
  odometry state before and after straight or in-place-turn commands.
- **Safety:** Each capture is limited to five seconds, publishes a zero command
  repeatedly at the end, and accepts a run only after a newer EV3 status message
  confirms stopped motion.
- **Validation:** A run is rejected if either track moves fewer than the minimum
  encoder counts or if straight/turn encoder directions are inconsistent.
- **Traceability:** Every hardware attempt writes an atomic JSON report,
  including failed attempts. Reports stay on the Jetson under
  `/home/animesh/echora/calibration/` and are ignored by Git.
- **Success:** Added calculation mode for effective wheel radius from measured
  straight displacement and effective track width from measured turn angle.
- **Tests:** The local suite reached 358 passing tests with four dependency-based
  skips. Python compilation and repository whitespace checks passed.

### Deployment and real tests

- **Deployment:** Installed `calibration.py` and `calibrate_odometry.py` in
  `/home/animesh/echora/` on the Jetson and passed Python compilation there.
- **Expected failure:** The first 0.5-second capture received no
  `/robot_status`. The EV3 was unreachable from both Mac and Jetson and absent
  from a full `192.168.1.0/24` scan. The runner exited with failure, saved
  `connectivity-check.json`, sent stop messages, and no motor command reached
  the offline robot.
- **Recovery:** After the owner powered on the EV3, ping returned at
  `192.168.1.25` with 0% loss and roughly 4.6 ms average latency. The ROS bridge
  resumed status publication without a restart.
- **Straight smoke success:** A 0.04 m/s, 0.5-second request advanced both
  encoders from 0 to 56 counts. The runner confirmed stopped feedback. With the
  provisional radius, odometry reported 0.02932 m travel.
- **Straight capture success:** A 0.08 m/s, 3-second request advanced the left
  encoder by 424 counts and the right by 419 counts. The runner confirmed
  stopped feedback. Provisional odometry reported 0.22069 m incremental travel
  and about -1.25 degrees of yaw; the encoder mismatch was five counts (1.2%).
- **Pending measurement:** The actual floor displacement and observed steering
  direction are required before computing and applying an effective wheel
  radius. Track-width calibration follows that result.

### Straight measurement received

- **Repeat capture:** The marked 3-second run advanced the left encoder by 368
  counts and the right by 361 counts, with a confirmed stop.
- **Owner observation:** Actual floor displacement was **0.10 m**, and the robot
  travelled straight.
- **Calculated result:** The average 364.5-count rotation gives an initial
  effective wheel radius of **0.015719 m**, replacing the provisional 0.03 m in
  `config/robot.yaml`.
- **Caveat:** A 10 cm hand measurement has substantial relative uncertainty.
  Validate this radius over a longer straight run before navigation acceptance.
- **Next test:** Perform a measured in-place turn using the new radius, compute
  effective track width, then repeat straight and turn validation.

### Initial turn measurement received

- **First turn capture:** A nominal 90-degree left command produced balanced
  -353/+353 encoder counts and a confirmed stop; it was not physically measured.
- **Repeated turn capture:** The same command produced -398/+400 counts and a
  confirmed stop. The encoder difference was 798 counts.
- **Owner observation:** The repeated run physically turned **70 degrees left**.
- **Calculated result:** Using the calibrated 0.015719 m effective wheel radius,
  the measurement gives an initial effective track width of **0.179197 m**,
  replacing the provisional 0.12 m in `config/robot.yaml`.
- **Variation noted:** The two nominally identical turns differed by about 13%
  in encoder travel. The value is therefore provisional until a slower repeat
  turn validates it.

### Calibration refinement and Phase 4 acceptance

- **Turn validation:** With the first calibrated geometry, a slower turn moved
  -563/+566 encoder counts. The owner measured exactly 90 degrees left, while
  odometry predicted 99.1 degrees. Track width was refined to 0.197186 m.
- **Turn repeat:** A second slower run moved -568/+569 counts, physically turned
  90 degrees left, and odometry predicted 90.64 degrees. Both track commands
  were balanced and stopped feedback was confirmed.
- **Tooling failure found:** The first straight validator published `/cmd_vel`
  whenever any ROS callback arrived. Camera-independent odometry and status
  callbacks could therefore create bursts and queue stale forward commands.
  The robot still stopped, but the run was rejected for timing repeatability.
- **Fix:** Calibration commands now publish at a strict 10 Hz and use a
  one-message publisher queue so a zero command cannot wait behind stale motion
  messages. The full local suite increased to 359 passing tests with four
  dependency-based skips.
- **Final straight check:** The corrected runner produced balanced 396/397
  counts; odometry reported 10.88 cm and the owner measured 10 cm straight.
- **Final geometry:** The effective drive radius was refined to **0.0144504 m**.
  Recomputing the confirmed 90-degree turn with that radius produced an
  effective track width of **0.182557 m**. Both are deployed in
  `/home/animesh/echora/robot.yaml`.
- **Acceptance:** Phase 4 is complete for initial mapping experiments. These
  are effective tracked-chassis values, not physical sprocket dimensions;
  surface-dependent track slip remains expected and visual localization should
  correct accumulated encoder drift.

## 2026-09-03 — Roadmap changed to camera-only single-room autonomy

- **Owner decision:** Build the useful single-room find-person experience
  before mapping. The active sequence is camera-head control, camera-only
  single-room guidance/search, find-person mission, voice/intelligence, then
  the PWA.
- **Deferred, not removed:** Visual mapping/localization, whole-home Nav2, and
  multi-room search now follow the PWA.
- **Camera strategy:** Motor A will move the camera between bounded forward and
  downward views. The chassis can perform controlled 360-degree scans. Before
  approaching a person, the controller checks the floor and route, moves only
  a short segment, stops, and checks again.
- **Route behavior:** Visible obstacles such as footwear should mark a route as
  blocked. The robot should compare left/right alternatives and rescan rather
  than continue blindly. Uncertain, stale, dark, or obstructed camera input is
  treated as blocked.
- **Architecture boundary:** This phase is reactive and local; it does not
  require RTAB-Map, a persistent occupancy map, or Nav2. The already installed
  RTAB-Map packages remain inactive and available for the deferred mapping
  phase.
- **Safety:** External or loose cables must not enter the tracks or be wound by
  chassis rotation. Camera-head encoder limits and physical clearance must be
  established before autonomous scan motion.

## 2026-09-03 — Phase 5 camera-head control started

- **Implementation success:** Added EV3 protocol v2 bounded absolute-position
  moves for motor A, a four-second independent timeout, stopped-position zeroing,
  and status fields for camera target and motion state.
- **Coordination success:** Normal ROS chassis commands no longer write motor A.
  The EV3 rejects chassis movement while the head is moving and rejects a head
  move while the chassis is moving. A head move explicitly brakes tracks B/C.
- **Jetson success:** Added a camera-head abstraction plus
  `/camera_head/command` and `/camera_head/status`. Named forward/down moves
  fail closed until calibration is enabled; uncalibrated jogs are limited to
  15 encoder degrees within ±180-degree software limits at 40 degrees/second.
- **Tests:** The Mac suite passes 369 tests with four dependency-based skips;
  Python compilation and whitespace validation pass.
- **Deployment success:** The updated bridge compiled on the Jetson, restarted
  as an active managed service, and exposes both new ROS topics. Camera,
  perception, recognition, enrollment preview, and bridge services are active.
- **Deployment interruption:** The new EV3 file transferred, but the brick
  vanished from Wi-Fi before its service could be restarted and verified. The
  owner confirmed its battery had discharged; this was not a software or
  network configuration failure. No movement command was issued.
- **Next physical test:** Once the EV3 returns, verify protocol v2 and stopped
  B/C feedback, zero motor A at the current known view, then issue one positive
  10-degree jog while watching the live port-8080 camera view. Record physical
  direction before expanding the range.

### Battery recovery, camera-lead test, and tilt-range start

- **EV3 recovery:** After charging, output B initially reported `error`; the
  service correctly failed closed. Reapplying its automatic driver probe
  restored the motor, and the protocol-v2 service started normally.
- **Track-isolation success:** B/C speeds stayed zero and their encoders stayed
  0/1 throughout the motor-A test.
- **Invalid range test:** The initial pose was incorrectly interpreted as the
  lower limit. The owner later clarified that the mechanism was already at its
  top limit and positive commands pushed farther into that stop. The +8 through
  +71 readings are discarded as valid camera travel; likely they measure
  drivetrain compliance/backlash. The robot going offline ended the test.
- **Correction:** Top is defined as encoder 0; downward travel is negative.
  The next test must use negative commands and correlate encoder displacement
  with actual image motion before accepting any range value.
- **Repeatability improvement:** Protocol v3 home-to-top drives A slowly in the
  positive direction with B/C braked, monitors encoder progress
  locally, stops after 0.4 seconds without progress (or a driver stall), and
  then zeros the encoder. An eight-second outer timeout fails closed. Absolute
  head moves are now rejected until the head has been homed or manually zeroed.
- **Camera-lead failure:** The replacement USB lead caused repeated disconnects,
  failed URB resubmissions, USB error `-71`, and a frozen black stream despite
  `state=streaming` (intensity 10.0, 309 consecutive identical frames). A USB
  software power cycle did not recover it.
- **Camera recovery:** Restoring the original lead recovered calibrated live
  video at 640×480 MJPG. Final health measured 27.36 fps, mean intensity 106.19,
  zero consecutive duplicates, and zero consecutive read failures. The port
  8080 enrollment preview is active and reports `camera_ready=true`; all 18
  enrolled target samples remain intact.
- **Tests:** The protocol-v3 homing and client changes bring the Mac suite to
  372 passing tests with four dependency-based skips.

### Manual camera-head controls deployed

- **UI success:** Added two controls to the existing port-8080 live camera
  page: `Tilt up` and `Tilt down`. Each click requests exactly one five-degree
  step and the page displays current encoder position.
- **UI safety:** Buttons disable on stale/unavailable status, before homing,
  while moving/homing, or at the configured -180/0 software limits. Server-side
  validation independently enforces the same rules.
- **Visual confirmation:** The first corrected negative test moved A from top
  zero to encoder -9. Across 67 tracked image features the camera view moved a
  median 93.56 pixels, proving real camera movement in the opposite direction.
- **Measurement-tool failure:** The following step reached encoder -28, but
  changed the view too much for the small-motion optical-flow matcher to retain
  ten features. The measurement tool now falls back to full-frame phase
  correlation and pixel change instead of aborting; no images are stored.
- **Button acceptance:** The deployed HTTP controls moved down from top to -6
  and back up to -2. Throughout the cycle, B/C speeds stayed zero and their
  encoders remained 0/1. The live status endpoint reports the head homed and
  available.
- **Tests:** Five camera-control validation tests bring the full suite to 377
  passing tests with four dependency-based skips.

### Camera tilt direction corrected and UI limits opened

- **Direction settled:** Motor A's encoder counts **increase as the lens tilts
  down**. The owner confirmed it from the physical lens while jogging, and the
  live frames agree. Both previous readings were wrong in the same way, which is
  why the browser buttons moved opposite to their labels.
- **Frame evidence:** At encoder -3 the lens was aimed steeply at the cove
  ceiling and its batten light; by -7 it showed a floor-standing appliance and a
  wooden door. So `camera_head_forward_position: 0` had been labelling the
  *downward* end of travel as the forward view.
- **Image orientation ruled out:** The sensor is **not** mounted inverted. A
  zoom of a framed wall photograph shows the snow peaks pointing up and the
  frame's lower edge visible from below, so preview up/down can be trusted. The
  inversion was entirely in the encoder sign convention.
- **Torque, not calibration:** At speed 40 the head could not lift itself.
  Three +5 jogs wound the encoder from -10 to -2 while the frames stayed
  pixel-identical (profile cross-correlation residual 4-9 at zero shift, versus
  ~1700 for a genuinely different pose). Raising `camera_head_speed` to 150, the
  EV3's `tool_speed_limit`, restored real travel. Note that `speed_sp` is not a
  torque control: it only helped because the head was being driven, not held.
- **"Extreme angles are invalid" root cause:** `jog()` and `validate_camera_jog`
  refused any step crossing a software limit instead of trimming to it. Since
  the head settles a few degrees off round numbers, a +5 step from -4 targeted
  +1 and was rejected outright, making the last degrees permanently unreachable.
  Both now clamp to the limit and refuse only a head already sitting on it.
- **Encoder zero is provisional.** It is only where homing last stalled, not a
  verified stop: with limits opened to -180..180 the head reached +47 and -41.
  The real end stops are still unmeasured.
- **Repeatability:** Returning to encoder -7 reproduced the earlier -7 scene
  (residual 730, against 1732 for a different pose), so the encoder is usable as
  a coarse reference even though it is not precise.
- **Deployed:** `camera_head.py`, `camera_controls.py`, and
  `enrollment_console.py` updated on the Jetson; bridge and enrollment console
  restarted; `robot.yaml` backed up to `robot.yaml.bak-tiltcal`. The console now
  offers 5-degree and 15-degree steps in both directions.
- **Tests:** 384 passing with four dependency-based skips.
- **Still open:** measure the true end stops and the forward/down positions,
  then set `camera_head_calibrated: true`. Named moves stay refused until then.

### Image-guided camera-head sweep and load failure (2026-09-04)

- **Direction confirmed from images:** A settled 15-degree sequence moved from
  a ceiling-only view at the old encoder 0, across the ceiling/wall edge at
  +52/+66, to a level room view at +81. Positive is down and negative is up.
  The earlier low-speed `home` at the ceiling did not move the loaded mechanism
  and therefore supplied false direction evidence.
- **Forward and down candidates:** The verified room-forward view was re-zeroed
  to 0. From it, +16 showed the floor/route ahead; +32 was dominated by the
  nearby fabric surface, +49 showed a close cable, and +67 was occluded by the
  robot/body. The useful down view is therefore near +16, not a hard stop.
- **Load failure documented:** At 150 and 300 counts/s, upward targets either
  timed out or lost position after the driver left regulated hold. At 600,
  results varied with linkage angle. At 1000, one -15 request completed at -14
  in under 0.4 seconds and stayed in `holding`, but a later step toward a more
  heavily loaded angle timed out and the camera fell back. The tracks remained
  stopped throughout.
- **Conclusion:** Visual classification can choose ceiling/forward/floor and
  image change can prove that each command moved the camera, but software cannot
  compensate for a mechanism that cannot lift or hold its payload. Use hand
  support for calibration now and add a LEGO counterweight, stronger gearing,
  or a better-balanced mount before autonomous motion.
- **Preview reliability:** Added `/snapshot.jpg` for one settled frame without
  opening another MJPEG stream. Raw camera preview is independent of
  face-observation freshness. The console service no longer `Requires` the face
  detector, so restarting face inference does not take the controls offline.
  Regression passed: with `echora-face-detector.service` stopped, the console
  remained active and `/api/status` continued reporting `camera_ready=true`;
  all perception services were then restored active.
- **Cloud option:** A cloud vision model may label occasional settled snapshots
  during calibration. Motor motion and image-change checks stay local; missing,
  slow, or uncertain cloud output must stop the sequence.
