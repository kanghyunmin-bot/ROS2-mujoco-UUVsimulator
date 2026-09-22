# Parallel residual reinforcement learning

> Current preset (2026-09-22): **precision_v3**, 2 release-best environments +
> 3 precision-candidate environments, independent PPO policies and learned braking.
> See [the current contract](PRECISION_V3.md) and
> [visual update report](../../docs/UUV_UPDATE_20260922.md).
> The reward values and center-distance description below describe the older
> legacy mode, not precision_v3. Checkpoints and datasets are not distributed;
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

Success requires an attached yellow target at reset, a release during this episode,
actual equality release, and right CAD finger contact with the lower PVC stem at
release, within the configured simulation deadline. This is contact evidence, not
proof of exclusive causality if the hull is simultaneously contacting the buoy.

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
