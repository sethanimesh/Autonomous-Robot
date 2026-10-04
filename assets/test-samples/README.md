# Real test sample provenance

These two 640 × 480 webcam captures were selected for the project README from supervised home-room tests. The published copies preserve the original image bytes and existing overlays; no visual alterations were applied.

## Seated-recipient observation — 5 September 2026

![Seated-recipient observation](seated-recipient-20260905.jpg)

Original source: `docs/perception/seated_test_20260905/raised.jpg`.

The raised camera view shows a seated participant with a face box and facial landmarks. The adjacent `raised_observation.json` records a confirmed recipient, five supporting observations in a five-observation window, a three-observation requirement, and stopped track/head feedback. Those historical settings differ from the current recognition defaults.

The image has no separate exposure timestamp or binding to an individual confirmation result. It illustrates the recorded component test; the associated telemetry establishes the repeated-confirmation result.

## Floor-level observation — 6 September 2026

![Floor-level observation](floor-observation-20260906.jpg)

Original source: `docs/perception/integrated_live_20260906/room-search-20260906-052038/final.jpg`.

The low-angle view shows a cable, footwear and furniture at floor level. The adjacent `ui-find-mission.json` reports a paused search after the cable-heading limit was exceeded. `after-health.json` records a streaming camera and stopped robot. No approach or message delivery occurred in this run, and the image has no separate exposure timestamp.

This sample illustrates the scene visible to the camera and a bounded search outcome. It is not a route-clearance assessment. These component and room-search samples are documented separately from the owner-reported complete home caregiver demonstration.

## Integrity

The original source captures and raw telemetry remain in their local experiment directories. The selected images are included here so README rendering does not depend on those directories being published.

| Published image | SHA-256 |
|---|---|
| `seated-recipient-20260905.jpg` | `862f063a469c39da619630daeefd70bc7354fb44c757180c1b2650db44d10653` |
| `floor-observation-20260906.jpg` | `f9bb20c25fe3cf74d0adaf17eb061ad6f98f2111fcbbbdf653cdcbd92a80076d` |
