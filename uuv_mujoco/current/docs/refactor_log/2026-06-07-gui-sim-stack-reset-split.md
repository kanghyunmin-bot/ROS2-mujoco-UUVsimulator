# GUI Sim Stack Reset Split

Date: 2026-06-07

Scope: `uuv_mujoco/current/gui`

## Change

`SimStackResetMixin` now keeps the GUI-facing reset/stop method surface while
reset execution details are split into focused helpers:

- `sim_stack_reset_commands.py`: blocking reset/stop command execution and
  async reset process creation.
- `sim_stack_reset_output.py`: `[reset]` log-line filtering and 150-character
  status shortening.
- `sim_stack_reset_result.py`: reset return-code handling, GUI status updates,
  event pushes, ownership flag updates, and async control refresh.
- `sim_stack_reset_mixin.py`: public mixin methods and wiring to GUI state.

## Contract

This is behavior-neutral.  It preserves:

- GUI-owned process termination before reset
- docker backend stop before reset
- `reset_uuv_sim.sh --wipe-eeprom`
- `[reset]` line forwarding into GUI events/status
- success state when the external stack exits
- external-process-still-running state when reset succeeds but the stack stays
  alive
- nonzero return-code failure event/status
- best-effort root `after(0, _refresh_sim_stack_controls)` refresh

It does not change ArduPilot, SITL, RC override, plant inputs, or physics
coefficients.

## Verification

```text
PYTHONPATH=/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current /Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python <fake GUI reset smoke>
python3 -m compileall -q /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current /Users/kanghyunmin/Desktop/uuv_sim/uuv_control_gui.py
PYTHONPATH=/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools python3 /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools/audit_code_contract_sources.py --out-dir /private/tmp/uuv_source_audit_spaghetti_20260607_sim_stack_reset_split
python3 /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools/check_runtime_readiness_policy.py
python3 /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools/verify_ardusub_thruster_contract.py --quiet
/Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools/physics_contract_audit.py --output-dir /private/tmp/uuv_physics_audit_spaghetti_20260607_sim_stack_reset_split --simulate-s 0.05
git -C /Users/kanghyunmin/Desktop/uuv_sim diff --check
```

Result: fake GUI reset smoke `PASS`, source audit `fail=0 pass=11 warn=5`,
readiness `PASS`, thruster contract `OK`, physics contract audit completed,
and `gui/sim_stack_reset_mixin.py` is no longer in the top 50 refactor
inventory hotspot list.
