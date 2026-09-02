# Hardware and Machine Notes

Update this file whenever a wiring, port, calibration, or machine fact is
confirmed. Mark guesses as unknown until they are physically or programmatically
verified.

## Development Mac

| Item | Confirmed value |
| --- | --- |
| Architecture | Apple silicon (`arm64`) |
| LAN address during initial check | `192.168.1.26/24` |
| Role | Development and remote-control workstation |

## Jetson Orin Nano

| Item | Value/status |
| --- | --- |
| Confirmed SSH endpoint | `animesh@192.168.1.48` |
| LAN hardware address observed | `c0:bf:be:eb:44:e1` |
| Role | Main compute, camera, perception, autonomy, and later ROS 2 |
| Hardware | NVIDIA Jetson Orin Nano developer kit, arm64 |
| Operating system | Ubuntu 22.04.5 LTS, kernel `5.15.148-tegra` |
| NVIDIA Linux for Tegra | `36.4.3` |
| Memory | 7.4 GiB; no swap configured during inventory |
| Root storage | 915 GB NVMe, approximately 725 GB available during inventory |
| Python | 3.10.12 |
| ROS 2 | Humble ros-base installed; Nav2 packages queried were absent |
| OpenCV | 4.5.4 |
| PyTorch | 2.6.0 CPU-only; CUDA unavailable to PyTorch |
| CUDA | 12.6 runtime directory present; `nvcc` not available in the shell |
| Docker | 28.0.1 |
| USB camera | USB `0c45:6366` (`Arducam_8mp` in kernel history; currently identifies as Microdia Webcam Vitade AF) |
| Camera verification | `/dev/video0` and `/dev/video1` present; OpenCV captured a `640×480` BGR frame from `/dev/video0` |
| Video tools | `v4l2-ctl` absent during inventory |
| EV3 client | `/home/animesh/echora/ev3_client.py`; deployed and tested |

## LEGO EV3 with ev3dev

| Item | Value/status |
| --- | --- |
| Confirmed SSH endpoint | `robot@192.168.1.25` |
| LAN hardware address observed | `7c:c2:c6:29:b7:f1` |
| Role | Low-level motor, encoder, IR sensor, and local safety controller |
| Operating system | ev3dev-stretch, kernel `4.14.117-ev3dev-2.3.5-ev3` |
| Python | 3.5.3; `ev3dev2` imports successfully |
| Memory | 56 MiB RAM and 95 MiB swap |
| Root storage | 15 GB, approximately 13 GB available during inventory |
| Battery | Approximately 7.95 V during inventory |
| Output A | EV3 large motor; tool/camera head — owner confirmed |
| Output B | EV3 large motor; left track — owner confirmed |
| Output C | EV3 large motor; right track — owner confirmed |
| Medium motor | Not detected during inventory |
| Sensor inputs | Input 1 reports `error`; inputs 2–4 report `no-sensor` |
| IR sensor | Not detected and explicitly deferred by owner |
| Positive motor direction | Unknown for both tracks |
| Positive encoder response | Confirmed on A, B, and C during short +100°/s pulses |
| Wheel/track geometry | Unknown |
| Existing remote code | `/home/robot/track3r`; see `docs/EXISTING_EV3_SERVER.md` |
| New control service | `/home/robot/echora/ev3_server.py`; managed by enabled and active `echora-ev3.service` |

## Safety facts to confirm before motor testing

- Robot can be lifted so the tracks are clear of the ground for the first test.
- An immediate manual stop method is available.
- Left and right track motor ports are known. **Confirmed.**
- IR sensor is deferred; floor tests must not claim IR emergency stopping.
- Short positive commands turn each track in the expected direction.
- EV3 stops locally on stale commands or a lost connection before floor testing.

## Secrets

Passwords are intentionally not recorded here. Use the credentials supplied out
of band by the owner, and prefer SSH keys later if approved.
