# Project scope and technical goal

These findings explain the inspected working tree, not a freshly verified live robot. They distinguish implementation, recorded software checks, physical demonstrations, and unverified assumptions. Refer to the [full dossier](FULL_TECHNICAL_DOSSIER.md) for the complete argument and the [snapshot](SOURCE_SNAPSHOT.md) for provenance. Source paths refer to the original repository; code and raw data are not bundled here.

**A. Technical goal and demonstrated scope**

In plain language, the supported goal is:

> Find a selected enrolled person in one prepared room, maintain a qualified estimate of which visible person is the target, inspect the floor with the same camera, and attempt a short approach while stopping or reconsidering when observations or robot feedback become unusable.

A technically precise formulation is:

> Coordinate profile-bound face and appearance evidence, camera-view transitions, pose-bound distributed visual assessments, and encoder-monitored motion primitives on a tether-constrained EV3/Jetson robot, with explicit invalidation and bounded recovery when identity, scene, or actuator evidence becomes stale or inconsistent.

This formulation preserves the central engineering problem while avoiding capabilities the evidence does not establish.

| Scope level | What the evidence supports |
|---|---|
| Intended application | Assistive communication followed by delivery to a chosen caregiver. |
| Implemented robotics | Configurable enrolled-recipient selection; face and clothing-supported identity; bounded camera/chassis search; route assessment; short approach segments; cancellation and recovery. |
| Recorded physical capability | Selected-person search, route-checked movement, reacquisition, and stopped feedback in supervised indoor trials. |
| Arrival | Implemented stopping policies, including an explicitly approximate outcome. Reliable physical standoff is unverified. |
| Assistance-message delivery | A separate app prepares and speaks reviewed messages locally. No connected robot-delivery path was found. |
| Acknowledgement | No integrated caregiver receipt/acknowledgement mechanism was found. |
| Hospital deployment or clinical benefit | Not established in the inspected code, reports, or development history. |

The distinction between robotics and communication is explicit in the project itself. The communication app describes itself as a stationary slice with no robot commands or caregiver delivery connected. Its confirmation endpoint binds reviewed text to a revision and permits audio generation; it does not dispatch a robot mission. The delivery plan describes `DeliverMessage`, recipient playback, and acknowledgement as later work. See [communication scope, lines 1–6 and 141–156](<reference_docs/communication/README.md>) (source line 1), message confirmation and audio generation, lines 599–703 — `communication/backend/app.py:599`, and [planned robot delivery, lines 239–245](<reference_docs/docs/ASSISTIVE_COMMUNICATION_PLAN.md>) (source line 239).

The furthest useful demonstrations are complementary:

- An older integrated run completed **three approach/reacquisition cycles**, with encoder-estimated movements of approximately **4.04, 4.48, and 4.73 cm**, then encountered a route check limit. It lasted **91.14 seconds** and did not establish arrival.
- A September 12 run found the target, chose a left route, turned **−39.82°**, checked the new forward view, drove an encoder-estimated **6.39 cm**, reacquired the target by face, and stopped. Its outcome was `step_complete_target_reacquired`, after **68.21 seconds**.
- A later clothing-supported run requested **5 cm**, recorded **6.77 cm** of encoder-estimated movement, reacquired the person, then lost usable identity during lower-view inspection. It ended `target_found_not_at_standoff` after **88.62 seconds**.

These are physical integration results, not a complete arrival benchmark. Sources: three-cycle mission report — `docs/perception/integrated_live_20260905/multistep-route-retry-live-20260905/mission.json`, September 12 one-step summary — `artifacts/mom-approach-near-floor-retry-20260912/summary.json`, and [later approach result and limitations, lines 2618–2622](<reference_docs/docs/DEVELOPMENT_LOG.md>) (source line 2618).

[Back to reading guide](README.md)
