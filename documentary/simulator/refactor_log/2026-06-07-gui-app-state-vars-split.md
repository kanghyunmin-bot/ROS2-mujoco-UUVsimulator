# GUI App State Variable Split

Date: 2026-06-07

Scope: active runtime under `sim/current`, backed by the compatibility
directory `uuv_mujoco/v2.2`.

## What Changed

- Split GUI Tk variable initialization out of `gui/app_state.py`.
- Reduced `gui/app_state_vars.py` to the public initialization facade.
- Added `gui/app_state_core_vars.py` for always-on visibility, command-axis,
  and vehicle-status variables.
- Added `gui/app_state_feature_vars.py` for replay, physics, AutoTune, runtime
  panel, Ping360, and AutoTune monitor variables.
- Added `gui/app_state_tk_vars.py` for small Boolean/Double/StringVar helpers.

## Contract Preserved

- `gui/app_state.py` still calls `initialize_gui_vars(self)`.
- Existing Tk variable names are preserved.
- GUI app root/runtime state remains owned by `gui/app_state.py`.
- `gui/app_state_vars.py` no longer imports `.runtime`, so the variable contract
  can be smoke-tested without loading ROS2/rclpy.

## Verification

```text
python3 -m compileall -q \
  sim/current/gui/app_state.py \
  sim/current/gui/app_state_vars.py \
  sim/current/gui/app_state_core_vars.py \
  sim/current/gui/app_state_feature_vars.py \
  sim/current/gui/app_state_tk_vars.py

PYTHONPATH=/Users/kanghyunmin/Desktop/uuv_sim/sim/current \
python3 - <<'PY'
# Fake Tk vars smoke: app_state_vars_smoke PASS
PY

python3 sim/current/tools/refactor_inventory.py \
  --root sim/current --limit 25
```

Result: app-state modules no longer appear in the top 25 hotspot inventory.
