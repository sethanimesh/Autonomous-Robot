# ADR 0002: Share one camera through stopped, segmented motion

## Status and context

Accepted in the current single-room implementation; documented retrospectively on 2026-10-04.

The LEGO mechanism tilts one USB webcam vertically using large motor A. The same camera must obtain person/face views and inspect floor corridors. Tracks B/C rotate the chassis for horizontal scanning. The current tether and uncertain monocular geometry constrain movement.

## Alternatives

| Option | Advantage | Cost |
|---|---|---|
| Fixed forward webcam | Simple view/reference handling | Limited nearby floor visibility |
| Separate person/floor cameras | Simultaneous observations | More hardware, calibration and synchronization |
| Depth camera or LiDAR | Additional geometric sensing | Requires different hardware/integration; does not solve identity alone |
| One motorized webcam | Uses the available mechanism and sensor | Person-view gaps, mechanical settling and repeated reacquisition |

## Decision and rationale

Use referenced camera poses, exclusive camera control and track/head interlocks. Stop before route inspection, inspect candidate corridors, recheck after turning, execute one short encoder-monitored segment, then stop and reacquire the recipient before another approach.

This is a practical vertical slice for the available hardware. Monocular depth, segmentation and Gemini visible-hazard advice support route heuristics; none is a guarantee of complete three-dimensional clearance.

## Evidence

- [Top-level mission](../../robot/jetson/mission/autonomous_find.py), [camera lease](../../robot/jetson/mission/camera_control_lease.py) and [detour sequence](../../robot/jetson/navigation/closed_loop_detour.py).
- [Route binding tests](../../tests/test_detour_route_binding.py), [camera lease tests](../../tests/test_camera_control_lease.py) and [single-room policy tests](../../tests/test_single_room_mission.py).
- [Recorded approach/reacquisition results](../../share/09_EVALUATION_AND_RECORDED_RESULTS.md), including an incomplete run that lost identity during lower-view inspection.
- [Calibration provenance](../calibration/README.md): automatic coverage acceptance failed before a manual visual promotion.

## Trade-offs and consequences

The compact sensor arrangement requires camera transitions and adds latency between segments. Recognition workers continue processing images; the mission gates their use during camera transitions and obtains fresh recipient observations after each movement.

The floor-view movement worker does not continuously check recipient identity. Short requested distances are not hard physical bounds because feedback tolerance, braking and track slip matter. There is no active IR obstacle-stop layer. A prepared supervised room and current camera reference remain operating prerequisites.

## Revisit conditions

Add independent sensing or change the shared-camera policy if moving-person trials, small/overhanging hazards, stopping-gap measurements or transition failures exceed acceptable operating limits. Mapping/SLAM and Nav2 remain deferred rather than prerequisites for this prototype.
