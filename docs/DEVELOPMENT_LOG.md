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
