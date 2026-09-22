# Actual simulator evidence · 2026-09-22

No generated concept images are included. Original recordings were read without
starting, resetting, stopping or controlling any running simulator.

## Camera material

- Run: `20260922-155955-6c1b18`, `env-1`, `rollout-00012`.
- Recorder session: `sim_20260922T070050Z_52b0e0f1`, `episode_000012`.
- `front-approach.jpg`: original `ego/frame_000020.jpg` (640 × 360).
- `hand-approach.jpg`: original `buoy_release/frame_000044.jpg` (640 × 360).
- The two stills show different moments; they are not a synchronized pair.
- `dual-camera-rollout.mp4`: all 49 recorded image pairs, left front / right hand,
  ordered by frame number at nominal 10 Hz (4.9 seconds).
  CPU-only H.264 encoding, one thread, no generated frames or geometric edits.
  Playback uses nominal sample cadence, not wall-clock simulator speed.
- Episode evidence: `released=true`, `release_tier=hand_view`, strict
  `success=false`, `release_right_fork_region=false`,
  `release_right_fork_stem_contact=false`. It is NOT precision insertion proof.
- This example was selected for visual clarity, not randomly sampled. It is not
  an aggregate success-rate evaluation.

## Dashboard

`rl-dashboard.png` is a read-only browser screenshot of the live growth-curve
section on 2026-09-22 around 17:27 KST. It mixes labeled training and evaluation
series; aggregate counts are not held-out success rates. The run was configured
for five environments (2+3); at inspection four remained active after one
infrastructure failure. No reset or restart was performed.

## Interactive explanation

`../../interactive/uuv-learning.html` is an offline explanation of equations and
approval rules, not live telemetry. Example evaluation counts are synthetic and
labeled as examples. It does not send robot or training commands.
