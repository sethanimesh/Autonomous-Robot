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
| SSH endpoint provided by owner | `seth@192.168.1.48` |
| LAN address observed in Mac neighbor table | `c0:bf:be:eb:44:e1` (reachability not confirmed) |
| Role | Main compute, camera, perception, autonomy, and later ROS 2 |
| USB camera | Reported attached; device identity and video modes unverified |
| Operating system / JetPack | Unknown |
| Python version | Unknown |
| ROS 2 status | Unknown |

## LEGO EV3 with ev3dev

| Item | Value/status |
| --- | --- |
| SSH endpoint provided by owner | `robot@192.168.1.25` |
| LAN address observed in Mac neighbor table | `7c:c2:c6:29:b7:f1` (reachability not confirmed) |
| Role | Low-level motor, encoder, IR sensor, and local safety controller |
| Operating system / ev3dev version | Unknown |
| Left track motor port | Unknown — confirm before any movement test |
| Right track motor port | Unknown — confirm before any movement test |
| Medium motor port | Unknown |
| IR sensor input port | Unknown |
| Positive motor direction | Unknown for both tracks |
| Wheel/track geometry | Unknown |

## Safety facts to confirm before motor testing

- Robot can be lifted so the tracks are clear of the ground for the first test.
- An immediate manual stop method is available.
- Left and right track motor ports are known.
- IR sensor port is known.
- Short positive commands turn each track in the expected direction.
- EV3 stops locally on stale commands or a lost connection before floor testing.

## Secrets

Passwords are intentionally not recorded here. Use the credentials supplied out
of band by the owner, and prefer SSH keys later if approved.
