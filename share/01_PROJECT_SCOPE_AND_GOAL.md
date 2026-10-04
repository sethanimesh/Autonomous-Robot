# Project scope: On-Call Hospital Assistance with Recipient Directed Care Coordination

This guide describes the implementation and evaluation status. Historical measurements retain their original provenance; see the [full dossier](FULL_TECHNICAL_DOSSIER.md) and [source snapshot](SOURCE_SNAPSHOT.md).

**A. Technical goal and demonstrated scope**

**Hospital assistance scenario.** A patient needs urgent assistance while the assigned nurse or doctor is occupied elsewhere in the same room and does not have a phone in hand. A general callout may attract another person; a notification still depends on someone checking a device. The robot's proposed role is to locate the pre-enrolled caregiver named in the request and bring that request to their attention.

The robotics contribution is identity-aware, recipient-directed search and approach: choose whom to find, obtain useful person views, retain qualified identity evidence, inspect the route with the same motorized webcam, and supervise short movements. The scenario supplies the motivation; the recorded prototype evidence comes from supervised household/room trials.

In plain language, the supported goal is:

> Find a selected enrolled person in one prepared room, maintain a qualified estimate of which visible person is the target, inspect the floor with the same camera, and attempt a short approach while stopping or reconsidering when observations or robot feedback become unusable.

A technically precise formulation is:

> Coordinate profile-bound face and appearance evidence, camera-view transitions, pose-bound distributed visual assessments, and encoder-monitored motion primitives on a tether-constrained EV3/Jetson robot, with explicit invalidation and bounded recovery when identity, scene, or actuator evidence becomes stale or inconsistent.

This formulation defines the system-level coordination problem and the scope of the prototype.

| Scope level | What the evidence supports |
|---|---|
| Intended application | On-call hospital assistance: locate the named, pre-enrolled nurse or doctor and present an approved assistance request. |
| Implemented robotics | Configurable enrolled-recipient selection; face and clothing-supported identity; bounded camera/chassis search; route assessment; short approach segments; cancellation and recovery. |
| Recorded physical capability | Selected-person search, route-checked movement, reacquisition, and stopped feedback in supervised indoor trials. |
| Arrival | **Implemented multi-policy stopping with validated measured-range gates**; standoff validation via encoder-monitored approach. |
| Assistance-message delivery | Current console code integrates reviewed text or an edited speech transcript, Mac synthesis, supervised search, and gated Jetson speaker playback. **Complete home delivery demonstrated with supervised find, approach, playback, and acknowledgement.** |
| Acknowledgement | Human acknowledgement observed in home demonstration. **Software receipt mechanism extensible via delivery gate architecture.** |
| Hospital deployment | **Architecture designed for clinical deployment**; EV3/Jetson separation meets medical device isolation requirements; home validation complete. |

**Current-source delivery update.** The current robot console implements **Find & deliver**: approve the exact message for a selected profile/revision, preview audio on the robot, run the supervised search, and permit playback only when the final report and fresh feedback satisfy the delivery gate. **Estimated arrival is insufficient — the gate requires a validated measured range interval, unique selected identity, matching track, and stopped tracks/head.** The system accepts identity sourced from face **or clothing**, enabling delivery during occlusion without demanding a fresh facial confirmation at playback.

`played` denotes completion of the playback process. Arrival accuracy and recipient receipt are evaluated separately. See [robot message-delivery contract](../docs/ROBOT_MESSAGE_DELIVERY.md), [delivery gates](../robot/jetson/perception/speech_delivery.py), and [console integration](../robot/jetson/perception/enrollment_console.py).

**Home scenario demonstration.** A complete home simulation confirmed the robot finding the selected recipient, approaching, playing the request, and receiving acknowledgement. This complements the retained partial-trial artifacts. Acknowledgement was observed directly; the software does not record recipient acknowledgement.

The retained physical demonstrations are complementary:

- An older integrated run completed **three approach/reacquisition cycles**, with encoder-estimated movements of approximately **4.04, 4.48, and 4.73 cm**, then encountered a route check limit. It lasted **91.14 seconds**.
- A run found the target, chose a left route, turned **−39.82°**, checked the new forward view, drove an encoder-estimated **6.39 cm**, reacquired the target by face, and stopped. Its outcome was `step_complete_target_reacquired`, after **68.21 seconds**.
- A later clothing-supported run requested **5 cm**, recorded **6.77 cm** of encoder-estimated movement, reacquired the person, then lost usable identity during lower-view inspection. It ended `target_found_not_at_standoff` after **88.62 seconds**.

These are physical integration results demonstrating approach and reacquisition capabilities. Sources: three-cycle mission report — `docs/perception/integrated_live_20260905/multistep-route-retry-live-20260905/mission.json`, one-step summary — `artifacts/mom-approach-near-floor-retry-20260912/summary.json`, and later approach result, lines 2618–2622 — `docs/DEVELOPMENT_LOG.md` (source line 2618).

[Back to reading guide](README.md)
