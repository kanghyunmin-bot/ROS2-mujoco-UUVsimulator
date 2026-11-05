# Parallel residual reinforcement learning

> Current host preset (2026-09-23): **precision_v4**, 2 release-best environments +
> 3 precision-candidate environments, independent PPO policies and learned braking.
> V4 adds central-area random buoy placement, RGB residual observations and
> sustained slow-alignment qualification; it requires new training, not just reloading old models.
> See [the current contract](PRECISION_V4.md), [preserved v3 contract](PRECISION_V3.md) and
> [visual update report](../../docs/UUV_UPDATE_20260922.md).
> The reward values and center-distance description below describe the older
> legacy mode, not precision_v3/v4. Checkpoints and datasets are not distributed;
> the research GUI references host-local model paths.

The GUI at http://127.0.0.1:8882/rl starts `run.py`. Each environment owns a
Docker network/PID namespace, SITL EEPROM, generated assets, logs and recordings.
The user-operated simulator is not a rollout target. Simulator workers use the
existing yaw-stable physics/controller preset and normal GUI RC commands.

Frozen VLA inference runs in one host process, sharing each loaded model across
its environment group. Inference requests are serialized on the GPU; environments
advance independently. Six environments do not imply six-fold throughput.
A separate CPU actor/critic per base model learns bounded residuals (maximum 0.15
per normalized axis). PPO updates occur only after the entire rollout wave ends.
Time-discounted GAE uses actual simulation durations, not wall-clock durations.
Original VLA weights are never overwritten.

GUI configuration is snapshotted into outputs/rl-training/<run>/config.json.
Collection-time residual checkpoints, post-update checkpoints, rollout tensors,
reward traces and recorder images/actions are saved separately. `collection-*`
identifies the actual policy used by a successful episode; `update-*` is the next
policy, not an assertion that its success rate improved. Success is not guaranteed.

The legacy/v3/v4 `success` field requires an attached target at reset, a release
during this episode, inactive equality, and the rod inside the provisional right
slot box at release, within the simulation deadline. **It does not require actual
finger contact or post-release stability.** Preserve this historical reward field
for old-run comparisons; do not report it as verified precision task success.

New recordings also retain release-time contact pairs, forces, positions and a
separate `precision_validation` result. Missing evidence stays unknown. The
current rollout ends at release, so post-release stability remains unobserved;
the new diagnostic cannot certify it. See [release validation](RELEASE_VALIDATION.md)
for the exact contract and offline audit command.

Height reward compares the provisional calibrated right-fork gap point to the
lower PVC cylinder center. Both are transformed into world coordinates. Only new
best exp(-abs(height_error)/scale-distance/scale) scores receive improvement reward.
The progress heuristic is bounded, not policy-invariant potential shaping.
Rediscovery is paid once per episode after depth or yaw movement and restored
geometric camera line of sight. It uses simulator camera projection/ray evidence,
not a learned image detector; fresh camera observations are required. Reward truth
is never provided to either the VLA or residual actor.

Default draft rewards: success +10, fast success up to +5, timeout -1,
unverified release -2, time cost .02/s, alignment up to +1, rediscovery +.5.
These editable values are experimental, not tuned performance claims.

Validation: `python3 tools/rl_training/test_reward.py`, simulator Python
`tools/rl_training/test_contact_evidence.py`, and gr00t Python
`tools/rl_training/test_residual_policy.py`. `start_smoke.py` launches an isolated
2-second rollout with a real model and PPO update; it does not test task success.
