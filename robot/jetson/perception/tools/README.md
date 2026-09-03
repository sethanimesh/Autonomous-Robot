# Perception test tools

Operator tools for testing the person detector against real hardware. They are
**not** part of the runtime: nothing here is started by a systemd unit, and the
detector neither imports nor depends on any of it.

They import `rclpy`, `cv2` and `numpy`, so they run on the Jetson only and are
not covered by the repository test suite. The detector's own logic is tested in
`tests/`; these exist for the parts that need a room, a camera, and a person.

Copy them to the Jetson and run them with `/opt/ros/humble/setup.bash` sourced.

## `detection_preview.py`

Serves the annotated person-detection stream as MJPEG so the operator can see
what the detector sees while standing in front of the robot.

Not to be confused with the camera calibration preview, which overlays ChArUco
corners instead of person boxes and lives with the camera node.

```text
python3 detection_preview.py          # then open http://<jetson>:8088/
python3 detection_preview.py --port 9000 --bind 127.0.0.1
```

This is the tool that makes physical testing practical. Asked to hit marks
without a view, an operator cannot tell whether they are in frame — and a
misaimed or dark camera is invisible without a picture. The first person test
of this milestone produced zero detections across 2646 frames because the
camera was pointed at an unlit ceiling, which a preview would have shown
immediately.

**It is unauthenticated and binds to all interfaces by default**, so anyone on
the network can watch the camera. Run it while testing and stop it afterwards.
Do not install it as a service without adding authentication and restricting
the bind address.

## `measure_phase.py`

Measures detection performance for one held pose.

```text
python3 measure_phase.py near --seconds 20 --expect 1
python3 measure_phase.py two_people --expect 2
```

Reports detection percentage, the person-count histogram, confidence range,
and median box size, and saves one annotated frame. Run one pose per
invocation rather than scripting a long timed sequence: the operator stays in
control, and each phase gets a number that does not depend on hitting a cue.

The person-count histogram is the useful part for multi-person tests. `{2: 295}`
means exactly two people in every frame, which is what proves NMS is not
merging overlapping people or splitting one.

**Always look at the saved frame.** A detection rate with no picture behind it
hid two separate faults during this milestone.

## `check_empty_scene.py`

Measures the false-positive rate on an empty scene.

```text
python3 check_empty_scene.py --settle 15 --seconds 90
```

Discards a settling period so that someone walking out of shot does not count
against the result, and saves a frame for every apparent false positive.

Both behaviours exist because of a real mistake: a first run reported 5.14%
false positives, and the captured frame showed a person's torso filling the
image. The room was not empty and those detections were correct. Re-run with a
settle, the same scene measured zero. Inspect the saved frames before
reporting any number from this tool as a false-positive rate.

## `soak.py`

Watches a running detector for memory growth, staleness, and errors.

```text
python3 soak.py --minutes 10
```

Samples RSS and CPU from `/proc` alongside the rates and latency on
`/perception/status`. Three columns decide whether a long run is healthy:

| Column | Healthy |
| --- | --- |
| `RSS(kB)` | plateaus rather than climbing continuously |
| `age_s` | no upward trend, or published frames are going stale |
| `errs` | flat |

`in_Hz` is expected to vary — it follows the camera, which slows in dimmer
light. A dip there is not a detector fault; the detector riding one out without
error is what the tool is watching for.
