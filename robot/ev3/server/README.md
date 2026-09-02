# EV3 Fail-safe JSON Server

This is the proposed Phase 1 EV3 control service. It is intentionally small and
uses only the Python 3.5 standard library available on the current ev3dev brick.

It has not yet been deployed to the EV3 or used to move a motor.

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

1. Run automated tests on the Mac without hardware.
2. Copy the service to a new EV3 directory without replacing the old server.
3. Validate Python 3.5 syntax on the EV3.
4. Start the service manually with the robot raised and tracks clear.
5. Test ping and status before any non-zero command.
6. Send one short, low-speed pulse to one motor at a time.
7. Verify physical direction and encoder sign with owner feedback.
8. Only then test paired track motion.
