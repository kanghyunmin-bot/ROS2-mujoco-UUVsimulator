# Reward draft v2 — 135 demonstration model

The GUI preset uses 6 environments, a 35 simulation-second deadline and 100
waves. Start is blocked until supervised training has completed and all final
model shards exist. No automatic RL launch is scheduled.

| Term | Default | Evidence / bound |
|---|---:|---|
| Verified right-fork release | +20 | Released constraint, release event in episode and within deadline, right fork / PVC contact at release |
| Speed | up to +5 | Remaining fraction of 35 seconds; verified success only |
| Approach progress | at most +1 | Increase in best exp(-fork-stem distance / 1 m) |
| Height + distance progress | at most +2 | Increase in best exp(-abs(z error)/0.1 m - distance/1 m) |
| Initial right-fork/PVC contact | +2 once | Actual simulator contact before release |
| Rediscovery | +0.5 once | Valid cameras, prior loss, depth change 0.03 m or yaw change 0.1 rad, then visibility restored |
| Time | -0.02 / simulation second | Actual elapsed simulation time |
| Timeout | -2 | No verified release within deadline |
| Unverified / late release | -5 | Detachment without required evidence |

All intermediate positive rewards sum to at most 5.5. Repeated approach/retreat,
contact cycling and visibility cycling do not generate repeated bonuses.
No global centering, clipping rejection, forced left-side success gate or reward
for spinning alone was added. Right seating remains valid.

Limitations: contact at release is evidence, not exclusive causal attribution.
Visibility is geometric center/frustum/raycast visibility, not a trained image
classifier; partial views can disagree. Fork reference position is calibrated
geometry, not learned perception. Residual PPO keeps the VLA frozen, uses state
and base action and limits correction to 0.15. These reward changes do not fix
those representation limits or establish success-rate improvement. Long-run RC
sampling latency remains unverified. Incomplete recordings retain failure labels.

Supervised training uses 135 unique success manifests, 14,522 frames, existing
reviewed intervals only at 1.1x (979 annotated frames), all others at 1x. It does
not invent phase labels for unreviewed episodes. Source groups remain recorded
in selection.json; sampling is uniform over complete chunks before emphasis,
not equal group probabilities. No held-out success evaluation is claimed.
