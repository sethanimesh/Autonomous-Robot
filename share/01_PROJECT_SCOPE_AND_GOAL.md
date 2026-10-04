# Project scope: On-Call Hospital Assistance with Recipient Directed Care Coordination

This guide describes the implementation and evaluation status as documented on 2026-10-04. Historical measurements retain their original provenance; see the [full dossier](FULL_TECHNICAL_DOSSIER.md) and [source snapshot](SOURCE_SNAPSHOT.md). Source line citations refer to the original investigation unless identified as a current-source review.

**A. Technical goal and demonstrated scope**

**Hospital assistance scenario.** A patient needs urgent assistance while the assigned nurse or doctor is occupied elsewhere in the same room and does not have a phone in hand. A general callout may attract another person; a notification still depends on someone checking a device. The proposed role of Echora is to locate the pre-enrolled caregiver named in the request and bring that request to their attention.

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
| Arrival | Implemented stopping policies, including an explicitly approximate outcome. Reliable physical standoff is unverified. |
| Assistance-message delivery | Current console code integrates reviewed text or an edited speech transcript, Mac synthesis, supervised search, and gated Jetson speaker playback. Complete home delivery is reported by the project owner; retained artifacts document partial trials. |
| Acknowledgement | Human acknowledgement is reported in the home simulation. No software receipt/acknowledgement mechanism is implemented. |
| Hospital deployment or clinical benefit | Not established in the inspected code, reports, or development history. |

**Current-source delivery update (2026-10-04).** The original investigation described the adjacent communication app as a stationary slice. The current robot console now implements **Find & deliver**: approve the exact message for a selected profile/revision, preview audio on the robot, run the supervised search, and permit playback only when the final report and fresh feedback satisfy the delivery gate. Estimated arrival is insufficient. The gate requires a validated measured range interval, unique selected identity, matching track, and stopped tracks/head. It permits identity sourced from face **or clothing**, rather than demanding a fresh facial confirmation at playback.

`played` denotes completion of the playback process. Arrival accuracy and recipient receipt are evaluated separately; the software does not record acknowledgement. See [robot message-delivery contract](../docs/ROBOT_MESSAGE_DELIVERY.md), [delivery gates](../robot/jetson/perception/speech_delivery.py), and [console integration](../robot/jetson/perception/enrollment_console.py).

**Home scenario demonstration.** On 2026-10-04, the project owner reported a complete home simulation: the robot found the selected recipient, approached, played the request, and the person acknowledged it. This complements the retained partial-trial artifacts. Trial count, timing, independently measured stopping distance, and a recording are not documented. Acknowledgement was observed by a person; the software does not record recipient acknowledgement.

The retained physical demonstrations are complementary:

- An older integrated run completed **three approach/reacquisition cycles**, with encoder-estimated movements of approximately **4.04, 4.48, and 4.73 cm**, then encountered a route check limit. It lasted **91.14 seconds** and did not establish arrival.
- A September 12 run found the target, chose a left route, turned **−39.82°**, checked the new forward view, drove an encoder-estimated **6.39 cm**, reacquired the target by face, and stopped. Its outcome was `step_complete_target_reacquired`, after **68.21 seconds**.
- A later clothing-supported run requested **5 cm**, recorded **6.77 cm** of encoder-estimated movement, reacquired the person, then lost usable identity during lower-view inspection. It ended `target_found_not_at_standoff` after **88.62 seconds**.

These are physical integration results, not a complete arrival benchmark. Sources: three-cycle mission report — `docs/perception/integrated_live_20260905/multistep-route-retry-live-20260905/mission.json`, September 12 one-step summary — `artifacts/mom-approach-near-floor-retry-20260912/summary.json`, and later approach result and limitations, lines 2618–2622 — `docs/DEVELOPMENT_LOG.md` (local development evidence) (source line 2618).

[Back to reading guide](README.md)
