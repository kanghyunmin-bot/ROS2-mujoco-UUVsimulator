# Active Runtime Metadata And GUI Process/Control Split

Date: 2026-06-07

Scope: active runtime under `sim/current`, backed by the compatibility
directory `uuv_mujoco/v2.2`.

## Changes

- Added `uuv_mujoco/RUNTIME_VERSION.json` so the active runtime provenance is
  explicit even while the physical backing directory remains named `v2.2`.
- Updated setup and closed-loop audit resolution so `sim/current` is the
  active path and `v2.2` is only a compatibility fallback.
- Split `gui/sim_stack_process_mixin.py` into launch, reset, and status mixins.
- Split `gui/control_display_mixin.py` into pilot command, panel toggle,
  drawing, feedback, and UI update mixins.

## Contract Notes

- Controller parity observation remains real `/mavros/rc/out` against SITL
  MAVLink `SERVO_OUTPUT_RAW` telemetry.
- Plant input remains raw ArduSub JSON servo packets.
- No ArduPilot source, submodule pointer, PWM remap, or ALT_HOLD shim was
  changed.

## Validation

```bash
python3 -m py_compile \
  sim/current/gui/control_display_mixin.py \
  sim/current/gui/control_pilot_mixin.py \
  sim/current/gui/control_toggle_mixin.py \
  sim/current/gui/control_draw_mixin.py \
  sim/current/gui/control_feedback_mixin.py \
  sim/current/gui/control_update_mixin.py \
  sim/current/gui/sim_stack_process_mixin.py \
  sim/current/gui/sim_stack_status_mixin.py \
  sim/current/gui/sim_stack_launch_mixin.py \
  sim/current/gui/sim_stack_reset_mixin.py \
  sim/current/tools/audit_closed_loop_contract.py \
  sim/current/tools/dev_os_compat_system.py \
  uuv_control_gui.py
python3 -m json.tool uuv_mujoco/RUNTIME_VERSION.json
python3 sim/current/tools/refactor_inventory.py --limit 15 --format markdown
```
