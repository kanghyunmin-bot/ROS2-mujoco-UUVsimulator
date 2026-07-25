# GUI Physics Window Split

Date: 2026-06-07

Scope: active runtime under `sim/current`, backed by the compatibility
directory `uuv_mujoco/v2.2`.

## What Changed

- Reduced `gui/physics_window.py` to the public GUI physics window methods.
- Added `gui/physics_window_shell.py` for Toplevel construction, header buttons,
  status label, canvas, scrollbar, and scroll-frame binding.
- Added `gui/physics_window_rows.py` for parameter table headers, parameter
  rows, active/inactive row display, and footer controls.

## Contract Preserved

- `PhysicsMixin` still imports `_show_physics_window`, `_close_physics_window`,
  and `_set_physics_status` from `gui/physics_window.py`.
- Existing physics editor button behavior is preserved: Reload, Apply, Apply +
  Restart, Close, inactive current-mode rows, and scroll-frame sizing.
- The current-mode inactive-row contract still delegates to
  `physics_param_io`/`physics_param_status`.

## Verification

```text
python3 -m compileall -q \
  sim/current/gui/physics_window.py \
  sim/current/gui/physics_window_shell.py \
  sim/current/gui/physics_window_rows.py \
  sim/current/gui/physics_mixin.py

PYTHONPATH=/Users/kanghyunmin/Desktop/uuv_sim/sim/current \
python3 - <<'PY'
# PhysicsMixin and helper import surface smoke:
# physics_window_surface_smoke PASS
PY

python3 sim/current/tools/refactor_inventory.py \
  --root sim/current --limit 30
```

Result: `gui/physics_window.py` no longer appears in the top 30 hotspot
inventory.
