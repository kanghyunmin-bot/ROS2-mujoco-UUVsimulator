# GUI Forced Environment Split

Date: 2026-06-07

Scope: active GUI runtime under `uuv_mujoco/current/gui`.

## What Changed

- Split GUI-started RC override contract into `sim_stack_env_forced_rc.py`.
- Split EKF, Bar30, ExternalNav, and surface/rangefinder contract into `sim_stack_env_forced_ekf.py`.
- Split command readiness, spin cadence, pilot mode, and scheduler contract into `sim_stack_env_forced_command.py`.
- Kept `sim_stack_env_forced.py` as the public entry point for:
  - `base_forced_gui_contract()`
  - `apply_real_start_contract()`
  - `apply_run_mode_contract()`
  - `apply_forced_gui_contract()`

This is a structure-only refactor. It preserves the forced GUI-start key/value
contract for RC override, SITL EKF flags, Bar30 pressure, command forwarding,
auto-ready behavior, closed-loop JSON servo fallback, and plant-replay RCOU
override gating.

## Focused Smoke

```text
PYTHONPATH=/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current python3 - <<'PY'
from gui.sim_stack_env_forced import apply_forced_gui_contract, base_forced_gui_contract
# verifies manual_baro and poshold_extnav flags, Bar30 pressure override,
# real-start CSV enablement, closed_loop fallback, and plant_replay override.
PY
```

Result:

```text
sim_stack_env_forced focused smoke: PASS
```

## Contract Gates

```text
python3 -m compileall -q /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current /Users/kanghyunmin/Desktop/uuv_sim/uuv_control_gui.py
PYTHONPATH=/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools python3 /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools/audit_code_contract_sources.py --out-dir /private/tmp/uuv_source_audit_spaghetti_20260607_gui_forced_env_split
python3 /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools/check_runtime_readiness_policy.py
python3 /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools/verify_ardusub_thruster_contract.py --quiet
/Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools/physics_contract_audit.py --output-dir /private/tmp/uuv_physics_audit_spaghetti_20260607_gui_forced_env_split --simulate-s 0.05
git -C /Users/kanghyunmin/Desktop/uuv_sim diff --check
```

Results:

```text
contract_source_audit: {"fail": 0, "pass": 11, "warn": 5}
runtime_readiness_policy=PASS
[thruster-contract] OK
physics_contract_audit: wrote static_force_balance.csv/json
diff --check: PASS
```
