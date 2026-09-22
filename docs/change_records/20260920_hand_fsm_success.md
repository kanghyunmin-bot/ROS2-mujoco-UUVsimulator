# Reference-view FSM refinement and physical release validation

Changes: simulator-only white-shaft observations associated with the yellow
float; reduced front approach PWM (1540 maximum, 1510 minimum before dead-zone
compensation); no forward motion in hand alignment until both image errors
are in the existing dead bands. Camera reference profile remains unchanged.

New independent `release_hand_contact` evidence uses the collision manifest's
102 CAD hand colliders, not all 777 vehicle release probes. Existing release
physics and force thresholds are unchanged. Both contact flags and inactive
magnetic constraint are required for successful labels. A hull release is a
failure. Old labels without hand evidence must not be treated as newly verified
hand successes.

The one-second post-release recording tail now keeps neutral RC fresh, and the
recorder closes before the FSM/control owner is released. Previously RC went
stale during that tail and the recorder interrupted with sampling_discontinuity.

First v10 success: `outputs/auto-collection/gui-20260920-100152-e3d03d/`.
Dataset: `outputs/vla-demonstrations/sim_20260920T100155Z_91257125/staging/episode_000000`.
282 frames, 564 decoded images, valid state/actions and 0.1-second sample timing.
Force 39.16 N, release_hand_contact=true, release_rake_contact=true, eq_active=false.
No INSERT_FORK transition occurred: release happened during approach. This is
verified hand-caused removal, not proof of controlled seating in the fork gap.

Tests: 10 outcome/contact-evidence tests, 4 shaft observations, 4 yellow ellipse
tests, isolated ROS front-loss handoff and stop-before-fine-alignment tests,
existing magnetic load/rope/float physics checks. The real model has 102 hand
colliders as a strict subset of 777 release probes. No training was launched.

Repeat run: `outputs/auto-collection/gui-20260920-100354-2edab0/`.

## Completed v10 repeat run

All three trials satisfy the strict CAD-hand contact and inactive-constraint
release definition. All 750 samples and 1,500 images pass validation. Combined
with the first success there are four consecutive physical contact releases.

| Trial | Frames | Hand-contact release | Release load | Reset |
|---|---:|---|---:|---|
| 1 | 273 | Yes | 33.30 N | Verified |
| 2 | 234 | Yes | 43971.00 N | Verified |
| 3 | 243 | Yes | 48.24 N | Verified |

Important limitation: releases happened during approach/reacquisition, not a
completed INSERT_FORK sequence. Do not describe this as verified precision
seating. Trial 2 has a 43,971 N release-load spike versus 33.30/48.24 N in the
other repeats; its contact dynamics need review before training. The outcome
label is preserved and the issue is explicit in validation_review.json. No
training was started. Final GUI API state: complete, running=false, armed=false.
