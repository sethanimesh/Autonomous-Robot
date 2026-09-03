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
