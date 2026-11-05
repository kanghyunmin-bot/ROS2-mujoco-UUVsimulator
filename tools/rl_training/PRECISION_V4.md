# Precision v4 — randomized target and sustained approach

Implemented 2026-09-23. This is a new training configuration, **not a claim of an
improved trained policy**. Original VLA weights, legacy/v3 reward code and saved
checkpoints remain intact. The GUI's saved preset retains the two release-seed
workers and three precision-seed workers; it does not start training by itself.

## Placement and reproducibility

Each isolated rollout requests a seeded target position in world X
[-1.5, 1.5] m, Y [-1, 1] m. The yellow float, its rope's fixed anchor/subtree and
free-joint pose translate together; depth and relative attachment are unchanged.
Reset restores the original scene before applying the new translation. MuJoCo
constants are refreshed without losing the requested pose or rewinding time.
Samples must be at least 1.25 m horizontally from the initial vehicle position.
Bounds are limited to the central research pool; arbitrary scenes are unsupported.

`spawn_region` contains `x_m`, `y_m`, `seed`. This opt-in setting is valid only
with `precision_v4`; manual and v3 resets remain fixed. Each isolated container
has its own request/result files. Invalid/missing requests fail rather than
silently running a fixed layout. Every rollout records actual placement in
`spawn.json` and `episode.json`.

Training seeds change with cycle/worker and never overlap evaluation seeds.
Incumbent/candidate evaluations use the same fixed layout bank and inference
seeds. This is a **model-selection bank**, not an independent final test set;
fresh unseen layouts are still required for generalization claims.

## Deployable residual observation: 23 → 49

The original 23 sensor values are preserved. Each RGB camera adds 13 values:

| Per-camera inputs | Count |
| --- | ---: |
| Yellow-target detection flag, normalized center offset, width/height, area fraction, image-border flag | 7 |
| Associated bright-shaft flag, normalized center offset, width/height, PCA line angle divided by pi | 6 |

Both cameras use the same stateless RGB extractor during collection and serving.
Unavailable visual detections are zeros with explicit flags; malformed/missing
camera inputs are rejected. The image reference points are provisional, not a
calibrated metric fork pose. Shaft cues require a nearby detected yellow float;
occluded/out-of-frame targets cannot be inferred from simulation truth.

These are **color/shape heuristics**, not a newly trained perception network and
not reliable under all lighting or real water conditions. The original VLA still
receives both full RGB images. Simulator slot offsets and contact evidence remain
reward-only; no ground-truth position is passed into the actor.

Input expansion uses zero initial weights on the 26 new columns, transfers old
sensor/base-action weights and residual/brake outputs, and resets critic/Adam.
Old deterministic actions are initially preserved within floating-point tolerance.
New saved checkpoints use `uuv.residual_rl.v3` and observation contract
`rgb_target_cues_v1`; this schema version is distinct from the reward version.
V1/v2 deployment remains supported. Old GUI models do not acquire new behavior
until the expanded policy has actually been trained and separately selected.

## No positive process credit for a fast fly-by

Approach/alignment/aiming/insertion profiles and their weights (1/2/1.5/3) remain.
V4 first requires all of the following continuously for at least 0.75 s:

- Target ahead of the slot within X [-0.02, 1.0] m.
- Lateral and height errors each ≤ 0.05 m; target bearing error ≤ 0.20 rad.
- Slot-relative target displacement speed ≤ 0.08 m/s, yaw rate ≤ 0.15 rad/s.
- Attached target, with valid advancing samples no more than 0.25 s apart.

The speed is a sampled finite difference in the rotating slot frame, not a
continuous-time maximum. It includes lateral/vertical motion, not just closing.
The minimum profile score across the dwell window is compared to the best
previously qualified score. Only its positive record improvement is paid.
Failed qualification resets dwell; fast passes, transient alignment and fleeting
contact receive no positive process reward. Stationary/repeated quality cannot
farm rewards. Existing time/overspeed/unaligned-advance/yaw costs remain.

**Scope:** actual release terminal rewards remain separate. An episode that
approaches properly and later fails may retain earlier earned progress; there is
no retrospective reward erasure. This strict gate can make learning sparse and
must be monitored before relaxing it. It is a reward condition, not a hidden
truth-based inference controller or a proven collision-avoidance system.

## Verification

- Unit regressions cover fast passing, fleeting contact, dwell breaks, slow
  qualified approach, repeated-score suppression, RGB-only features, observation
  expansion, old-action migration and new-checkpoint serving parity.
- Actual MuJoCo tests cover repeatable spawn, unchanged constraint error,
  float/anchor translation, attachment stability, original-layout restoration and
  invalid requests without moving the scene. Existing reset tests remain valid.
- Isolated smoke run `outputs/rl-smoke-20260923-002422`: one worker, 2 s episodes,
  four rounds (initial baseline, training, incumbent, candidate), one PPO update;
  completed without errors and removed its worker container. Stored batch shape
  was [11,49], with nonzero learned visual-input weights after the update.
- Training placement was [-0.463756, -0.414102] m; incumbent and candidate both
  used [-1.454191, 0.878653] m. No releases occurred in this short wiring test;
  equal outcomes correctly did not promote the candidate.

Run a bounded integration check with `start_smoke.py --precision_v4` after saving
an explicit v4 GUI configuration. Longer training, near-target visual validation
and independent randomized performance evaluation remain necessary.
