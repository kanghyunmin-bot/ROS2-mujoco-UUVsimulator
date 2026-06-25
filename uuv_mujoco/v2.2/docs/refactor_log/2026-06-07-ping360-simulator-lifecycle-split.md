# Ping360 Simulator Lifecycle Split

Date: 2026-06-07

Scope: `uuv_mujoco/current/bridge`

## Change

`bridge/ping360_sim.py` remains the public MuJoCo Ping360 simulator class, while
focused helpers now own lifecycle subcontracts:

- `ping360_model_ids.py`: MuJoCo site/body lookup for the configured sensor.
- `ping360_runtime_state.py`: geomgroup, RNG, sweep state, effective settings,
  and rolling history creation/refresh.
- `ping360_update_cycle.py`: scan generation, history update, and updated-sample
  construction.

## Contract

This is a behavior-preserving refactor.  It does not change Ping360 range,
angle, timing, noise, MuJoCo raycast, ROS message, or status payload semantics.

## Verification

```text
/Users/kanghyunmin/.venvs/mujoco311/bin/python <focused Ping360 lifecycle smoke>
python3 -m compileall -q uuv_mujoco/current uuv_control_gui.py
git diff --check
```

Result: Ping360 lifecycle smoke `PASS`, compileall `PASS`, diff check `PASS`.
