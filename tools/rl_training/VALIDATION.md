# Connected reinforcement-learning validation — 2026-09-21

- Actual one-environment VLA rollout, 2 simulation seconds, 20 transitions:
  `outputs/rl-smoke-20260921-195652`. Recorder save and PPO update completed;
  actor weights changed. Original VLA was frozen.
- Actual six-environment rollout, two models × three environments, 2 simulation
  seconds each: `outputs/rl-smoke-20260921-195944`. All six saved and closed;
  59-model actor updated with 56 transitions, 84-model actor with 57 transitions.
  Both actor weight tensors changed. Successful detachment count was zero;
  this verifies the plumbing, not task success or improved policy performance.
- Separate full model copies exceeded 16 GB VRAM during inference. Bitwise
  equality checks permitted sharing the frozen backbone only; the six-environment
  rerun passed after that change. Independent action heads and transforms remain.
- HTTP GUI start/stop test passed:
  `outputs/rl-training/20260921-200151-0b8d2d`. Original GUI configuration restored
  after test: two models × three environments, 35 seconds, 100 trials per environment.
- Actual consecutive-rollout test passed: `outputs/rl-smoke-20260921-200547`.
  Two 2-second episodes, two PPO updates, recorder session reuse and map reset
  between episodes completed. Worker cleanup preserved neutral ARM state only
  between owned simulation episodes and disarmed on the final episode.
- Browser verified enabled start, disabled stop after cleanup and restored settings.
- Reward unit tests: five; residual PPO tests: three; existing GUI manager tests: five.
- Trial containers were removed; the existing user-operated GUI container remained.

Implementation limits: height reference is the provisional calibrated fork-gap
point. Visibility reward uses geometric projection and occlusion rays, not a learned
visual detector. Right-fork contact plus release is not exclusive causal attribution
when simultaneous hull contact exists. 35-second task success rates and long-run
training quality have not been established by the short tests.

### Recorder automatic-save recovery (2026-09-21)

Run 20260921-220713-d731fa, environment 0, rollout 8 failed after the
collector skipped a stale RC override sample and saved 286 frames with
`termination_reason=sampling_discontinuity`. The worker then issued a duplicate
stop request. `recording.finish_recording` now handles this specific automatic
save (including a stop-request race), rejects old/empty/other failed saves, and
marks `recording_complete=false` plus `recording_warning.json`. Sensor-validated
RL transitions retain their own task outcome; the incomplete demonstration
retains its failure flag and must not be represented as a complete success.

Five recorder lifecycle tests and five reward tests pass. Replaying the actual
failure status reproduces the former duplicate-stop error and passes the new
finalizer. This does not establish the cause of RC delay or a long-run fix for
sampling discontinuities; collector timing checks remain enabled.

### Region release and multi-point visibility (2026-09-22)

Right-fork tier now uses the rod centerline intersecting a vehicle-fixed box
at release time, latched in the physics runtime before constraint release.
Center FLU [m]: [0.335511938, -0.091927344, -0.104344226]; half extents [m]:
[0.04, 0.025, 0.025]. Extents are an initial geometric criterion, not measured
contact tolerance. Actual contact evidence remains separate. GUI shows dimensions
and a schematic, not a scene overlay. Old recorded tiers retain their old meaning.

Visibility now rays toward multiple float/body and PVC points, accepts any visible
point and retains occlusion checks. It is still geometric visibility and can miss
unsampled visible surfaces; no pixel segmentation or image-based certification.
Existing pre-release 0.25 s lookback remains. Actual scene tests confirm rod
inside/outside region (lateral/forward), and nine reward tests pass. Full rollout
classification and action/reward timing correction remain unverified/outstanding.


## 2026-09-22 learning-stall repair

See [PPO_FIXES_20260922.md](PPO_FIXES_20260922.md) for the current per-epoch
backtracking guard, fresh incumbent/candidate comparison, transparent optical
window visibility fix, and regression/real-batch replay evidence. Historical
update counters include rejected attempts and must not be read as weight changes.
