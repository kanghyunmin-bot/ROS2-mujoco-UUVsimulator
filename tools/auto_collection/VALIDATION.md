# Validation history — 2026-09-20

## Current camera / v10 FSM

The v10 stop-and-align teacher achieved one verified hand-contact release in
`gui-20260920-100152-e3d03d`: 282 frames, 564 decoded images at 640x360, finite
23-state/4-action samples, exact 0.1-second ROS sample spacing, correct PWM
normalization. Magnetic load at release was 39.16 N; both release contact flags
were true and the constraint was inactive. Map reset and disarm were verified.
This release occurred during APPROACH_BUOY, before the dedicated insertion
state, so it establishes physical hand-caused detachment, not reliable precision
insertion. A repeated three-trial run is recorded separately below.

The preceding v9 release had no CAD-hand contact at release and was correctly
rejected, even though the broader hull-release contact flag was true.

## Earlier unmodified-controller validation


Isolated ROS domain 142; GUI host port 18878. Original simulator was not controlled.
The real visual FSM used `YOLO/best.pt` (buoy=0, stick=1), with its upstream defaults.

| Trial | Frames | Result | End condition | Reset |
|---|---:|---|---|---|
| 1 | 602 | Failure | 60 simulation seconds | Verified |
| 2 | 603 | Failure | 60 simulation seconds | Verified |
| 3 | 574 | Failure | FSM COMPLETE, no verified release | Verified |

The teacher reached APPROACH_BUOY but repeatedly lost the target; none reached
ALIGN_STICK or INSERT_FORK. FSM COMPLETE was correctly saved as failure because
there was no verified physical detachment. That earlier batch did **not** prove successful collection.

All 3,558 JPEGs decoded at 640×360. All 1,779 states/actions were finite, shaped
23/4, and recorded action values matched PWM normalization. Final vehicle state:
disarmed, GUI control disabled, recorder stopped, session unlocked. All three
maps reset. Seven label/evidence tests passed; existing contact snapshot and
400-case/600-step wrench equivalence checks passed with zero physics difference.

Run evidence: `outputs/auto-collection/validation-3c/`.
Dataset: `outputs/vla-demonstrations/sim_20260920T072648Z_23f19dac/staging/`.
These are validation failures, not successful training demonstrations.

After the batch started, the runner gained model hashing, a post-release hand-frame
wait, explicit reset-response checking, and final disarm confirmation. The batch
executed its already loaded version; these additions and a positive end-to-end
release remain outside the three-trial validation. Hand images are recorded
evidence, not a trained visual release classifier.

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
