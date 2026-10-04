# Clothing memory and shared-camera coordination

This guide presents the implementation and evaluation status. Historical measurements retain their original provenance; see the [full dossier](FULL_TECHNICAL_DOSSIER.md) and [source snapshot](SOURCE_SNAPSHOT.md).

**C6. Clothing memory and reacquisition**

The current family implementation distinguishes:

- Permanent profile enrollment.
- Permanent face-anchored outfit memory.
- Temporary body tracks and appearance continuity.
- Fresh facial confirmation.

SQLite contains `profiles`, `settings`, `outfits`, and `views`. It uses foreign keys and WAL; the database and containing directory receive restrictive filesystem permissions. Each outfit can retain up to six representative clothing crops. Individual outfits and profiles can be forgotten/deleted. No outfit expiration or overall outfit-count bound was found. See schema and storage rules, lines 13–170 — `robot/jetson/perception/family_store.py:13`.

New permanent outfit ownership requires a confirmed face anchor, source frame key, and matching enrollment revision. However, re-enrollment updates the profile revision **without deleting its outfits**; the wardrobe query joins old outfits to the current revision. Pending-response revision binding should therefore not be described as invalidation of all historical appearance.

Local clothing comparison uses normalized hue/saturation histograms and texture in body-relative strips. Those strips approximate upper clothing, lower clothing, and footwear; they are not anatomical segmentation. Partial-view tracking adds eight ordered colour/brightness/texture strips and can match a sufficiently large portion of an earlier view.

Representative tracking gates are:

- Standard appearance similarity ≥0.78.
- Spatial overlap ≥0.15 when position is still valid.
- Unique best score separated by >0.08.
- Partial matching only with one currently detected person, a recent known candidate, score ≥0.90, and a second supporting observation before exposing selected identity.
- Missing bodies immediately invalidate current position.
- Unseen tracks are removed after 30 seconds.
- Motion invalidates coordinates, range, and pending view evidence while retaining trusted appearance.

See descriptors and partial matching, lines 18–76 — `robot/jetson/perception/wardrobe_tracking.py:18` and tracking/selection rules, lines 108–191 and 286–293 — `robot/jetson/perception/wardrobe_tracking.py:108`.

Cloud clothing comparison sends a cropped current view and at most two stored references. It accepts only structured comparison results tied to supplied reference IDs. Requests bind UUID, track, profile, revision, frame, image hash, and motion epoch. Responses older than ten seconds or belonging to changed state are rejected. A briefly missing body may return within the original deadline; its current appearance must still agree.

Identical reference images are explicitly treated as ambiguous even if Gemini selects an owner. New permanent ownership is not learned from a clothes-only match. See cloud request/result handling, lines 193–284 — `robot/jetson/perception/wardrobe_tracking.py:193` and wardrobe schema and ambiguity handling, lines 8–94 — `robot/mac/wardrobe_advisor.py:8`.

The crucial current behavior is:

> Clothing is not relabelled as a fresh face match, but it can authorize family-mode target acquisition, approach, and arrival.

`FamilyObserver` defaults both clothing approach and approximate approach to enabled. It keeps `confirmed` specific to facial evidence while publishing `identity_confirmed` for a qualifying selected track. The scanner and approach policy use the latter in family mode. A saved outfit can reacquire identity after restart without a new face observation on that run. Tests explicitly cover arrival without a face.

See family defaults and enrichment, lines 27–33 and 174–205 — `robot/jetson/perception/family_observer.py:27`, scanner identity gate, lines 75–89 — `robot/jetson/mission/bounded_target_scan.py:75`, and identity readiness, lines 24–35 — `robot/jetson/mission/person_approach.py:24`.

The family tracker also has no absolute “time since last face” requirement while current body support continues. The older `PersonContinuity` module’s ten-second face lifetime does not apply universally.

These distinctions make the mechanism substantive, but leave unresolved risks from similar clothing, crossings, background contamination, and incorrectly accepted appearance ownership. The tests exercise selected cases; they do not establish a household false-identification rate.

**C7. Shared-camera coordination**

The system handles the shared camera through several cooperating mechanisms rather than one unified scheduler:

- Cross-process camera-control lease.
- Single-owner head-command worker with cancellation and stop priority.
- EV3 head/chassis mutual exclusion.
- Mission-level stop/look/turn/look/move sequence.
- Invalidation of position, range, and pending visual results on motion/reference changes.
- Restoration of useful person views and post-movement target reacquisition.

The bridge clears an old chassis command when a head command arrives, so it cannot resume after the head movement. EV3 rejects drive while head motion is active and rejects head movement while tracks are active. See bridge command coordination, lines 324–339 — `robot/jetson/ev3_bridge/ros_node.py:324` and process ownership lease — `robot/jetson/mission/camera_control_lease.py:16`.

The strongest delayed-computation safeguard appears in useful-view calibration: capture is bound to head reference/position and stopped track generations/counts; after cloud inference, the consumer requires newly received feedback showing the same stopped physical state. Request UUID and image SHA also must match. See capture and post-response binding, lines 826–882 — `robot/jetson/mission/camera_head_calibration.py:826`.

There are important limits:

- Raw camera frames do not carry a directly measured optical pose.
- Recognition workers continue processing images; coordination primarily gates downstream use rather than pausing all perception.
- Encoder position cannot by itself establish what the lens sees.
- During floor inspection, the recipient may be outside the image.

Most significantly, the detour worker receives no recipient identity or target-frame binding and subscribes to no identity topic. It checks camera, head, robot, and odometry while moving; the mission reacquires the target **after** the primitive. Thus there is an identity-observation gap through lowering, route inference, turning/rechecking, and driving. See movement handoff, lines 262–299 — `robot/jetson/mission/autonomous_find.py:262` and detour subscriptions, lines 208–226 — `robot/jetson/navigation/closed_loop_detour.py:208`.

[Back to reading guide](README.md)
