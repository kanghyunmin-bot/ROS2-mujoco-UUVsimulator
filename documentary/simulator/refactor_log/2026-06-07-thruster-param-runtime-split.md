# Thruster Parameter Runtime Split

Date: 2026-06-07

Scope: active runtime under `sim/current`.

## What Changed

- Split thruster parameter state creation into `sim/runtime/thruster_param_runtime_state.py`.
- Split JSON loading plus profile/environment direct-gain override application into `sim/runtime/thruster_param_runtime_loader.py`.
- Split runtime summary logging into `sim/runtime/thruster_param_runtime_summary.py`.
- Kept `sim/runtime/thruster_param_runtime.py` as the public `ThrusterParameterRuntime` facade.

This is a structure-only refactor. It does not change thruster coefficients,
servo mapping, direct T200 PWM-to-force curves, reverse asymmetry values,
first-order actuator time constants, ArduPilot source, or the ArduPilot
submodule pointer.

## Focused Smoke

```text
PYTHONPATH=/Users/kanghyunmin/Desktop/uuv_sim/sim/current python3 - <<'PY'
from sim.runtime.thruster_param_runtime import ThrusterParameterRuntime
# temporary JSON payload verifies global, per-thruster direct_gain_scale,
# reverse_asymmetry, tau_up/tau_down, and summary log paths.
PY
```

Result:

```text
thruster_param_runtime focused smoke: PASS
```

## Contract Gates

```text
python3 -m compileall -q /Users/kanghyunmin/Desktop/uuv_sim/sim/current /Users/kanghyunmin/Desktop/uuv_sim/uuv_control_gui.py
PYTHONPATH=/Users/kanghyunmin/Desktop/uuv_sim/sim/current/tools python3 /Users/kanghyunmin/Desktop/uuv_sim/sim/current/tools/audit_code_contract_sources.py --out-dir /private/tmp/uuv_source_audit_spaghetti_20260607_thruster_runtime_split
python3 /Users/kanghyunmin/Desktop/uuv_sim/sim/current/tools/check_runtime_readiness_policy.py
python3 /Users/kanghyunmin/Desktop/uuv_sim/sim/current/tools/verify_ardusub_thruster_contract.py --quiet
/Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python /Users/kanghyunmin/Desktop/uuv_sim/sim/current/tools/physics_contract_audit.py --output-dir /private/tmp/uuv_physics_audit_spaghetti_20260607_thruster_runtime_split --simulate-s 0.05
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
