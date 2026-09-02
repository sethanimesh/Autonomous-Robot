# EV3 Fail-safe JSON Server

This is the proposed Phase 1 EV3 control service. It is intentionally small and
uses only the Python 3.5 standard library available on the current ev3dev brick.

It is deployed at `/home/robot/echora/ev3_server.py` on the EV3 and passed a
manual raised-chassis motor test on 2026-09-02. The accompanying
`echora-ev3.service` unit runs it as the `robot` user and preserves the server's
shutdown stop behavior through `SIGINT`.

## Confirmed motor mapping

| Role | EV3 output |
| --- | --- |
| Tool/camera head | A |
| Left track | B |
| Right track | C |

Speeds are EV3 tacho-motor speed setpoints in degrees per second. Development
limits default to ±250 for each track and ±150 for the tool motor, well below the
attached motors' reported maximum of 1050.

## Safety behavior

- Stops all motors when the process starts.
- Uses `brake` as the stop action.
- Stops on an explicit `stop` command.
- Stops 500 ms after the last non-zero drive command.
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
{"command":"drive","left":100,"right":100,"tool":0}
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

Next:

1. Verify physical forward/reverse direction during controlled raised testing.
2. Add a small Jetson bridge client that continually refreshes commands.
3. Test paired tracks at low speed while raised.
4. Only then attempt a short, clear-floor movement test.
