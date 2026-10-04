# ADR 0003: Keep actuation authority local while offloading interpretation

## Status and context

Accepted in the current distributed implementation; documented retrospectively on 2026-10-04.

The Jetson hosts ROS 2, camera perception and missions. Heavier depth/segmentation and cloud calls run through a companion Mac. Responses can arrive after the scene, recipient selection, camera reference or robot pose has changed. Camera movement also requires blocking TCP transactions without starving robot feedback.

## Alternatives

| Option | Advantage | Cost |
|---|---|---|
| All inference onboard | Fewer network dependencies | Changes local compute/memory workload; not benchmarked as a complete alternative |
| Remote system issues motor commands | Centralized interpretation/action | Movement depends directly on remote state and connectivity |
| Offboard advice with local authority | Separates expensive interpretation from actuator supervision | Needs asynchronous workers, source binding and conservative failure handling |

## Decision and rationale

The Mac/cloud return interpretations and audio; the Jetson retains mission decisions. Route inference runs asynchronously while feedback is processed. Acceptance checks bind replies to requests/images, identity revisions, camera references and stopped motion state. Serialize EV3 TCP operations and fence cancelled camera actuator calls inside that boundary. The EV3 independently stops on absent accepted drive refreshes or client disconnect.

This permits heavier interpretation without allowing remote services to drive the tracks. It does not make remote advice instantaneous or eliminate every stale-data path.

## Evidence

- [Route worker and acceptance](../../robot/jetson/navigation/closed_loop_detour.py), [Mac request binding](../../robot/mac/route_perception.py) and [wardrobe revision/epoch checks](../../robot/jetson/perception/wardrobe_tracking.py).
- [Serialized command worker](../../robot/jetson/ev3_bridge/ros_node.py), [TCP client lock](../../robot/jetson/ev3_bridge/ev3_client.py) and [EV3 watchdog](../../robot/ev3/server/ev3_server.py).
- [Head cancellation tests](../../tests/test_head_command_worker.py), [TCP serialization tests](../../tests/test_ev3_client_serialization.py), [EV3 stop tests](../../tests/test_ev3_server.py) and [route binding tests](../../tests/test_detour_route_binding.py).
- [Freshness table and limitations](../../share/07_DISTRIBUTED_EXECUTION_AND_SAFETY.md).

## Trade-offs and consequences

Feedback can continue during offboard inference and camera-worker waits. Invalid or changed evidence causes stopping/rechecking rather than automatic use of an old answer. Network outages, provider latency, clocks and partial completion become explicit mission concerns.

Source expiry is not uniform at every layer. An unstamped ROS velocity command is aged from receipt, and EV3 drive requests do not contain end-to-end source expiry. The watchdog runs in the EV3 server thread; it does not independently cover a hung process or motor driver. A stop can wait behind a TCP transaction already holding the lock.

## Revisit conditions

Reconsider compute placement if measured mission latency or availability is inadequate. Revisit protocol deadlines, command authority and watchdog scheduling after current-version physical fault-stop measurements, especially delayed commands, stalled I/O and low battery. A stronger deployment setting requires its own evidence and operating constraints.
