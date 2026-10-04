# Distributed execution, freshness, stops, and recovery

These findings explain the inspected working tree, not a freshly verified live robot. They distinguish implementation, recorded software checks, physical demonstrations, and unverified assumptions. Refer to the [full dossier](FULL_TECHNICAL_DOSSIER.md) for the complete argument and the [snapshot](SOURCE_SNAPSHOT.md) for provenance. Source paths refer to the original repository; code and raw data are not bundled here.

**C10. Distributed execution, freshness, and fault handling**

Freshness exists at several layers, with different meanings:

| Layer | Implemented check | Practical limitation |
|---|---|---|
| Camera | Shape/encoding validation, reopen logic, 12-second no-publication watchdog. | Repeated frozen content can still receive new timestamps and count as publication. |
| Person inference | Newest-frame slot and 0.5-second configured age gate. | Age is cached at receipt, omitting subsequent queue delay. |
| Face/embedding inference | Exact-frame joins and age recomputed before inference. | No final age recheck after inference; future-stamp handling is inconsistent. |
| Family observer | Strictly increasing source time, ≤1-second source age, motion-boundary filtering. | Stronger than upstream recognizer; does not undo upstream vote-history contamination. |
| Clothing replies | UUID/hash/frame/track/profile/revision/epoch, ten-second limit. | Stored outfits themselves do not expire. |
| Range replies | Exact request/image/track binding; ≤3-second range age; current track. | Consistent estimates are not accuracy validation. |
| Route replies | Paired images, head/reference binding, stopped pose, usually ≤1-second delivered age. | Cross-device wall-clock synchronization is assumed. |
| Browser manual drive | Session token, increasing sequence, 0.25-second command timeout. | Depends on console/bridge/EV3 execution continuing. |
| ROS drive | 0.3-second age since callback receipt. | Unstamped commands can be old when received. |
| EV3 drive | 0.5-second time since accepted drive request. | No source timestamp, sequence, mission ID, or per-command expiry. |

Sources include camera watchdog — `robot/jetson/camera/camera_recovery.py:7`, family source/range validation, lines 65–170 — `robot/jetson/perception/family_observer.py:65`, route delivery checks, lines 314–371 — `robot/jetson/navigation/closed_loop_detour.py:314`, and manual-drive sequence/watchdog, lines 190–311 — `robot/jetson/perception/manual_drive.py:190`.

The route timestamp is Mac-side time around fetching a JPEG, not the camera’s hardware exposure time. Camera ROS stamps themselves are assigned after capture. Clock and capture latency therefore matter even when every field is internally consistent.

The EV3 does independently stop on expired drive refresh, disconnect, invalid requests, and normal shutdown. Status polling does not renew a drive deadline. However, the watchdog executes in the **same server thread** as sysfs operations and request handling. A 50 ms socket timeout bounds ordinary receive waits; sysfs operations have no explicit execution deadline. A hung process or driver cannot be claimed covered by a separately scheduled hardware watchdog. Track actuation uses `run-forever`. See watchdog, lines 623–646 — `robot/ev3/server/ev3_server.py:623` and server loop/cleanup, lines 827–868 — `robot/ev3/server/ev3_server.py:827`.

TCP client transactions are serialized, preventing interleaved responses. A stop may nevertheless wait behind a transaction already holding the lock. The local EV3 watchdog is the fallback at that boundary.

Cancellation of a browser-started mission sends SIGTERM to its process group and publishes motor-stop commands. A terminal-started mission is not owned by that browser process manager; the project documentation explicitly distinguishes those cancellation paths. No general latched emergency-stop authority or authenticated ROS command arbiter was found in the inspected path.

The console binds to `0.0.0.0` and checks an `X-Echora-Action` header for mutation requests. That header is not user authentication. The EV3 uses an allowed-client IP, not cryptographic command authentication. These are relevant limits for the hospital framing, not proof of a present network compromise. See console request handler and bind default, lines 1942–2004 — `robot/jetson/perception/enrollment_console.py:1942`.

[Back to reading guide](README.md)
