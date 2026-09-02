# Existing EV3 Server Notes

A read-only inventory found an earlier EV3 server implementation at
`/home/robot/track3r` on the brick. These files are not yet tracked in this
repository and were not changed during inventory.

## What exists

- C++ server binary and source: `ev3_server` / `ev3_server.cpp`
- Cap'n Proto schema and generated C++ files
- TCP port: 9999
- Makefile and several small Python repair utilities
- The server was not running and no matching system service was installed.

The schema supports ping, stop, individual-motor commands, combined drive
commands, IR reads, and status reads.

## Mapping encoded in the old source

| Function | EV3 port | Verification |
| --- | --- | --- |
| Left track | B | Source code only; physical confirmation required |
| Right track | C | Source code only; physical confirmation required |
| Tool/head | A | Source code only; physical confirmation required |
| IR distance | Input 2 | Source code only; sensor not currently detected |

All motors currently attached to A, B, and C identify as EV3 large motors. The
expected medium motor is not currently detected.

## Useful behavior

- Stops all three motors when a TCP client disconnects normally.
- Provides a dedicated stop command.
- Uses the EV3 sysfs interfaces directly, avoiding a heavy runtime.
- Returns `-1` when the expected IR sensor cannot be found.

## Safety and correctness gaps

Do not use the old server for a floor test as-is.

- No stale-command watchdog while a client remains connected.
- No local stop based on the IR distance.
- No speed clamping or development-mode maximum speed.
- No validation that requested motor ports are expected/safe.
- A status request returns only IR data, not the defined motor status list.
- Socket setup and send/receive operations are not fully checked.
- The comment says packed Cap'n Proto while the implementation uses a flat
  message array, which may confuse future clients.

## Recommended disposition

Keep this implementation as reference rather than activating it. For the first
vertical slice, a small Python 3.5-compatible EV3 service using newline-delimited
JSON will be easier to inspect and test. It should add a short command timeout,
hard speed limits, an immediate stop command, encoder feedback, and a local IR
stop once the sensor is reconnected.

No replacement should be deployed or motor-tested until the physical mapping
and safe wheels-off-ground setup are confirmed by the owner.
