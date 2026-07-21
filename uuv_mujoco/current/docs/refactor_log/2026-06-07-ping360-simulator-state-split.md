# Ping360 Simulator State Split

Date: 2026-06-07

Scope: active runtime `uuv_mujoco/current`, backed by the compatibility
directory `uuv_mujoco/v2.2`.

## Change

`bridge/ping360_sim.py` is now focused on MuJoCo sensor lifecycle and scan
updates. Mutable Ping360 state moved into focused modules:

- `bridge/ping360_history.py`: polar image, range, and intensity history buffers.
- `bridge/ping360_sweep.py`: full-scan and sector-bounce angle state.
- `bridge/ping360_samples.py`: held/updated sample and status payload creation.

The public simulator class remains:

- `Ping360Simulator`

Compatibility imports from `ping360_sim` for `Ping360Config`, `Ping360Sample`,
and `PING360_GRADS_PER_REV` remain available.

## Verification

Commands run:

```text
python3 -m compileall -q uuv_mujoco/current/bridge/ping360_sim.py ...
PYTHONPATH=uuv_mujoco/current /Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python - <<'PY'  # MuJoCo XML smoke
python3 uuv_mujoco/current/tools/refactor_inventory.py --root uuv_mujoco/current --limit 30
```

Results:

- Ping360 simulator split smoke: PASS.
- Ping360 sample/status split smoke: PASS.
- A minimal MuJoCo model with `base_link`, `ping360_site`, and a wall geom
  produced an active Ping360 simulator, an updated first profile, a held
  non-updated sample before the next profile period, and a valid status dict.
- `ping360_sim.py` dropped out of the top 35 hotspot table.

## Contract Notes

This refactor does not change raycast accumulation, ROS topics, scan image
rendering, or Ping360 config parsing. It only separates mutable sweep/history
state and sample/status payload creation from the simulator lifecycle class.
