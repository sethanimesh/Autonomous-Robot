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
