# Precision v3 — 2026-09-22

## Run intent and seed identity

- `release-best`, 2 environments: `20260922-025444-1e76ee/success-135-110-25000/collection-00046.pt`.
  Historical single evaluation: 5/6 general releases, zero slot-region releases.
- `precision-candidate`, 3 environments: `20260922-143636-d2e575/success-135-110-25000/collection-00012.pt`.
  Historical single evaluation: 1/5 hand-view releases, 2/5 total, zero slot-region releases.
  This is a provisional candidate, NOT a demonstrated precision-insertion champion.
- Both use the same frozen 135-demonstration, 25k-step VLA. Frozen inference is
  shared to avoid duplicate GPU allocations. Residual policies, optimizers,
  on-policy trajectories, evaluations, and checkpoints remain independent.
- Original seeds/runs are immutable. Actor/features and legacy deterministic
  actions are preserved on migration. Critic and Adam are reset because both
  reward semantics and the latent action dimension changed. Source update counts
  are recorded separately from new update counts. This is a warm start, not an
  exact optimizer resume. Starting this preset again re-seeds these source files.

## Continuous stage shaping

The rod's closest centerline point is transformed into the robot-fixed slot
frame. Axes are forward X, left Y, up Z; units are metres. This avoids targeting
the rod's geometric center instead of its graspable segment. The existing strict
slot-region release classifier and its 8 x 5 x 5 cm draft box are NOT enlarged.

`precision_reward.py` contains the complete formula. Profiles overlap smoothly;
there are no action-switching FSM stages. Let `n=sigmoid((0.65-distance)/0.15)`,
`L=exp(-(y/0.05)^2)`, `H=exp(-(z/0.05)^2)`,
`A=exp(-(atan2(y,max(x,0.08))/0.20)^2)`.

| Profile | Score | Maximum new-progress reward |
| --- | --- | --- |
| Approach | exp(-distance/1 m) | 1 |
| Lateral/height alignment | n L H | 2 |
| Entry-direction aiming | n L H A | 1.5 |
| Insertion proxy | exp(-(x/0.22)^2) L H A sigmoid((0.04-x)/0.025) | 3 |

Only improvement over each profile's episode best earns reward. Positive stage
rewards total at most 7.5, less the initial scores. Waiting, repeated crossing,
retreat/approach cycles cannot farm the same stage bonus. This is bounded
record-progress shaping, NOT policy-invariant potential shaping. It can bias
the learned task and must be evaluated empirically.

Closing speed is the decrease of slot-relative X per actual elapsed simulator
second, including motion caused by vehicle rotation. Target speed is
`0.04 + 0.18*(1-n) + 0.04*L*H*A` m/s. Excess closing speed, advancing while
misaligned, and yaw rates exceeding 0.15 rad/s incur smooth, bounded-rate costs
multiplied by elapsed simulator time. These are experimental shaping targets,
not hard safety limits or calibrated hardware commands.

Only pre-release samples contribute geometry progress. Release-induced rod
motion cannot generate insertion bonuses. Intended slot release +20; geometric
hand-view release +4; other timely release +2 (mutually exclusive). Only slot
release earns up to +1 speed bonus. First actual right-fork contact +0.5;
timeout -2, invalid release -5, time cost -0.015/s; rediscovery disabled.

All scales/weights are initial engineering hypotheses, NOT measured optimal
clearances or tuned performance claims. Contact-force shaping, sustained seating
and recovery curriculum are not implemented. No contact geometry, magnetic
release threshold or simulator dynamics are modified.

## PPO and action authority

Four residual axes remain bounded by 0.15. A fifth latent Gaussian output learns
forward-command attenuation: first clip the legacy four-axis action, then
multiply its forward component by `1-max(0,tanh(brake))`. This also attenuates
reverse command magnitude; it cannot reverse direction by itself. Zero mean
brake preserves the migrated deterministic action exactly. Training explores
braking; the observation contains no privileged geometry or stage label.

Learning rate 1e-4, clipping epsilon 0.15, four epochs, gradient clipping 0.5,
entropy coefficient 0.001, KL limit 0.005. Epoch backtracking restores both
weights and Adam, halves the learning rate and retries at most six extra times.
The drift bound is 0.003 on the actual composed deterministic action on the
current training batch, including the brake (NOT a global state-space bound).
Time-aware GAE uses gamma 0.995 and lambda 0.97 per 0.1 simulator seconds.

The residual still receives only 23 sensor values + 4 VLA actions, not images
or explicit slot offsets. The base VLA sees images. This information bottleneck
remains; richer rewards do not prove the residual can infer precision errors.
Neither VLA weight fine-tuning nor CAP equivalence is claimed.

## Collection, evaluation and retention

Initial baseline -> training collection -> synchronous PPO -> incumbent evaluation
twice -> candidate evaluation twice -> decision. Weights are fixed throughout
each collection/evaluation. Evaluation rollouts never enter PPO. Two repeats
produce 4 paired trials for release-best and 6 for precision-candidate; inference
seeds are matched per worker/repeat, while physics timing/noise may differ.

Promotion requires no loss in cumulative slot/hand-or-slot/total release counts
and improvement in at least one count. Pure shaping-score improvements do not
replace `accepted.pt`. Equal counts retain the learner; count regressions restore
incumbent actor AND optimizer. Unequal sample counts defer the decision. Repeats
are a small screening sample, not statistical proof. Precision profile scores
are diagnostics, not certified slot insertion or a substitute success metric.

`accepted.pt`, `training-latest.pt`, `collection-*`, `update-*`, training batches,
episode reward terms and source lineage are saved independently per group.
The v2 checkpoint schema marks the five-latent action policy; the deployment
loader supports both legacy v1 and v2. Do not load v2 with old four-latent clients.

## Verification completed

- 44 focused reward/PPO/recovery/deployment/recording tests and 6 GUI backend
  tests passed; Python compilation and served JavaScript syntax checks passed.
- Two regression tests fail with pre-change behavior (no brake; promote on
  all-failed reward improvement) and pass with the changes.
- Actual MuJoCo scene checks passed for closest-centerline telemetry, privilege
  gate, fork-region classification, right/left contact evidence and visibility.
- `outputs/rl-precision-verify-XH8KdU`: five environments, two-second plumbing
  trials, 30 completed episodes, one real PPO update per independent policy.
  Candidate evaluations used 4 and 6 trials. Both comparisons had no releases:
  accepted checkpoints remained unchanged; both learners were retained.
- PPO batches contained exactly 22 and 33 training transitions, respectively;
  evaluation transitions were excluded. All recorded reward sums and geometry
  fields were checked. Both seeds preserve the original deterministic actions
  exactly on 128 generated sensor/action inputs; Adam starts empty.
- Retained KL: 0.001366 / 0.004897. Actual mean-action drift on each training
  batch: 0.002990 / 0.001441, within the configured limits.
- Short trials validate plumbing, NOT task success, precision or sim-to-real
  transfer. Long-run success and the proposed reward scales remain unvalidated.
