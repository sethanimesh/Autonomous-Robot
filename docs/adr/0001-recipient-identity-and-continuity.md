# ADR 0001: Bind the mission to an enrolled recipient

## Status and context

Accepted in the current implementation; documented retrospectively on 2026-10-04.

A hospital assistance request names a responsible caregiver. Several people may be visible, and the closest person may be unrelated to that request. The prototype needs recipient selection independent of proximity, with continuity when a known person's face is temporarily hidden.

## Alternatives

| Option | Advantage | Cost |
|---|---|---|
| Approach nearest detected person | Simple; needs no enrollment | Does not satisfy a named-recipient request |
| Require face evidence for all continuity | Clear identity evidence boundary | Loses candidates whenever the face turns away |
| Use appearance alone | Useful across back/partial views | Similar clothes/uniforms can confuse identity |
| Enrolled face identity plus appearance memory | Stronger initial anchor with useful continuity | Requires enrollment, freshness checks and evidence-source semantics |

## Decision and rationale

Bind selection and requests to a profile ID/revision. Use repeated facial observations and a winning-profile margin for face confirmation. Store face-associated appearance memory in SQLite and track qualified body/clothing observations separately.

This addresses the selected-person problem while allowing recovery from face occlusion. It is a rationale based on implemented mechanisms; no controlled trial establishes superiority over every alternative.

## Evidence and current policy

- [Face matcher](../../robot/jetson/perception/family_faces.py): current family defaults require two supporting observations in five; older trials used other settings.
- [Profile store](../../robot/jetson/perception/family_store.py) and [wardrobe binding](../../robot/jetson/perception/wardrobe_tracking.py).
- [Family observer](../../robot/jetson/perception/family_observer.py) preserves the distinction between facial confirmation and clothing identity.
- [Partial-clothing tests](../../tests/test_partial_clothing_tracking.py) and [recorded continuity comparison](../../share/09_EVALUATION_AND_RECORDED_RESULTS.md).

**Current behavior:** appearance identity can authorize acquisition, approach, arrival and message-delivery identity checks without a new facial observation in that mission. Saved outfits remain available for this purpose. Appearance identity is represented separately from fresh facial confirmation.

## Trade-offs and consequences

The robot preserves a selected identity rather than retargeting the nearest person. Enrollment revisions, ambiguity, motion boundaries and evidence age must be checked throughout the mission. Appearance memory extends visibility but introduces outfit persistence and uniform-confusion risks. One recorded tracker improvement does not establish wrong-recipient error rates.

## Revisit conditions

Revisit approach/delivery authorization if distractor trials with similar clothing produce identity switches, old outfits survive changed circumstances, or stronger fresh-face requirements are needed for the intended operating scenario. Evaluate that policy change separately from tracking continuity.
