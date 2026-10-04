# Contribution candidates, ownership, and data handling

These findings explain the inspected working tree, not a freshly verified live robot. They distinguish implementation, recorded software checks, physical demonstrations, and unverified assumptions. Refer to the [full dossier](FULL_TECHNICAL_DOSSIER.md) for the complete argument and the [snapshot](SOURCE_SNAPSHOT.md) for provenance. Source paths refer to the original repository; code and raw data are not bundled here.

**C12. Data handling, evaluation infrastructure, and ownership boundaries**

Enrollment defaults to numerical templates; optional aligned face crops are separate. Clothing memory retains cropped images and descriptors, and selected crops are sent to Gemini. New recording tools can retain visible people in full camera frames. Therefore the older statement “full camera frames are never stored” is not a project-wide current guarantee.

The recorder is subscriber-only and bounded by duration, frame rate, and storage. It records source/receipt timing, camera/head/encoder state, detections, observations, and observed commands; enrollment vectors are excluded. ROS replay uses an isolated domain and does not republish recorded motion commands. It is a recorded-trajectory replay: a new command cannot change the recorded images or encoders.

The first retained real recording contains 408 frames and 6,882 events. Its mission stopped when the camera head did not move; the operator reported a depleted battery. Since the head never moved, that recording cannot validate cross-view continuity, even though it supports a useful stationary tracker regression.

Git history contains 43 reachable commits under one author name, but much of the newest work is uncommitted. Development notes explicitly mention agent/subagent implementation assistance. Git authorship therefore does not establish sole personal implementation. The defensible division is:

- **Third-party:** pretrained detectors, embedding model, semantic segmentation, depth model, cloud models, ROS/OpenCV/PyTorch/TensorRT.
- **Repository-specific engineering:** decoders and adapters, synchronization, enrollment workflow, profile/appearance lifecycle, camera and motor coordination, bounded mission logic, result binding, recovery, diagnostics, and evaluation tooling.
- **Personal ownership:** needs your account of who designed, implemented, integrated, tested, and interpreted each substantial part.

**G. Contribution candidates**

The strongest supported contribution is **coordinating evidence and control across changing camera roles and delayed computation**. It consists of several concrete mechanisms:

1. **Separating identity from current spatial evidence.** Face anchors and durable outfit ownership can survive longer than image coordinates and range. Motion invalidates the latter without automatically erasing all identity history. This directly addresses reacquisition after camera movement.

2. **Binding delayed results to the state that produced them.** Frame keys, image hashes, profile revisions, track IDs, motion epochs, motor generations, head references, and post-response feedback prevent many classes of cross-frame or changed-state result reuse.

3. **A bounded shared-camera control procedure.** Camera leases, motor interlocks, cancellation-aware command ownership, useful-view restoration, floor inspection, short movement, and reacquisition create an executable solution to having one sensor serve incompatible viewing roles.

4. **Challenge-informed low-level integration.** Stop acknowledgements versus telemetry, head backdrive/settling, queued-command timing, source-message arrival order, and physical encoder references required custom handling beyond connecting model APIs.

5. **Replayable diagnosis with honest measurement boundaries.** The latest recording tools and baseline comparison make a concrete issue reproducible, while keeping encoder travel, model estimates, generated tests, and physical truth distinct.

These are substantive systems-engineering contributions. They do not require claiming a novel detector, a new recognition algorithm, research novelty, or clinical validation.

The strongest interview narrative is therefore about managing **which evidence remains usable after time, motion, viewpoint changes, and asynchronous work**, and about what the robot does when that evidence is insufficient.

## Deletion boundary in the current implementation

SQLite profile deletion cascades through its clothing records. That does not establish deletion of every historical biometric copy: the preserved legacy enrollment JSON can remain, and the console's cleanup retains face-crop directories referenced by that legacy file. The database migration deliberately preserves that older enrollment. Clothing crops and cloud comparison are independent of the enrollment option for aligned-face-photo retention. These are different data lifecycles.

Source: `robot/jetson/perception/family_store.py:52–63,99–118` and `robot/jetson/perception/enrollment_console.py:1815–1829`. This is a code-derived boundary; no private identity database or image was copied into this folder.

[Back to reading guide](README.md)
