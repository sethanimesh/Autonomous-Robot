# EV3 Fail-safe JSON Server

This is the proposed Phase 1 EV3 control service. It is intentionally small and
uses only the Python 3.5 standard library available on the current ev3dev brick.

Protocol v6 is deployed at `/home/robot/echora/ev3_server.py` on the EV3 and has
passed live loaded-head tests. The accompanying `echora-ev3.service` unit runs
it as the `robot` user and preserves the server's shutdown stop behavior
through `SIGINT`.

## Confirmed motor mapping

| Role | EV3 output |
| --- | --- |
| Tool/camera head | A |
| Left track | B |
| Right track | C |

Speeds are EV3 tacho-motor speed setpoints in degrees per second. Development
limits default to ±250 for each track. Camera position moves normally use 300;
one same-target lifting retry may use 1500 against motor A's reported 1560
maximum.

## Safety behavior

- Stops all motors when the process starts.
- Uses `brake` for both tracks and active `hold` for the loaded camera head.
- Stops on an explicit `stop` command.
- Stops chassis/run-forever motion 500 ms after the last non-zero drive command.
- Rejects raw/run-forever camera speed commands.
- Limits the camera range to ±180 counts, each position change to 15 counts,
  and each position command to eight seconds.
- Watches encoder progress independently of the Jetson. After one second with
  less than two counts of progress, it reapplies the same target once. Lifting
  receives the 1500-count/s recovery; lowering retains its original speed. A
  second one-second stall stops and holds.
- While a camera move is active, accepts only a retry of that exact absolute
  target; a second target cannot be stacked.
- Requires the camera head to be homed or explicitly zeroed after server boot
  before accepting an absolute position move.
- Re-establishes active hold both before and after an encoder zero so an old
  hold target cannot move the loaded mechanism in the new coordinate system.
- Homes toward the configured reference at 300 counts/s, stopping and
  zeroing after 0.4 seconds without encoder progress; an eight-second outer
  timeout stops without declaring success.
- Rejects chassis movement while a camera-head position move is active, and
  rejects camera-head movement while the chassis is active.
- Stops when the TCP client disconnects.
- Stops all motors if a motor write or request fails.
- Stops immediately when a malformed or unknown request is received.
- Accepts only the confirmed A/B/C motor mapping.
- Accepts TCP clients only from the Jetson at `192.168.1.48` by default.
- Limits every requested speed before writing it to hardware.
- Does not reset encoders when a drive command is received.

The IR sensor is explicitly deferred because ev3dev does not currently detect
it. This service does not claim IR-based obstacle stopping.

## Protocol

The server listens on TCP port 9999. Each request and response is one compact
JSON object followed by a newline.

Ping:

```json
{"command":"ping"}
```

Drive, repeated more frequently than the 500 ms watchdog timeout:

```json
{"command":"drive","left":100,"right":100}
```

Any `tool` field on a drive request is rejected. Camera movement is available
only through bounded position commands.

Move the camera head to an absolute encoder position:

```json
{"command":"tool_move","position":-15,"speed":300}
```

Set the stopped camera's current position to zero during calibration:

```json
{"command":"tool_zero"}
```

Repeatably find the physical upper limit after a reboot:

```json
{"command":"tool_home","speed":300}
```

Stop:

```json
{"command":"stop"}
```

Read encoders and current motor state:

```json
{"command":"status"}
```

## Development sequence

Completed:

1. Automated tests passed on the Mac.
2. The service was copied to a new EV3 directory without replacing the old
   server.
3. Python 3.5 compilation passed on the EV3.
4. Ping and status passed before movement.
5. A, B, and C passed individual +100°/s, 0.25-second pulses.
6. The 500 ms hardware watchdog and disconnect stop paths passed.

Deployed protocol v6 has a current operator-selected tilt range of -54 to 0.
Next: select and verify the person/forward pose inside that range.

The server releases a client after five seconds without a complete request.
This lets a restarted Jetson replace a silent connection whose TCP session
survived the network outage. Normal bridge status polling keeps the connection
alive; incomplete bytes do not. Disconnect still stops the motors and preserves
a valid same-boot encoder reference. The independent motion watchdog is unchanged.
