# Physics Contract Runner Split

Date: 2026-06-07

Scope: active runtime only, under `uuv_mujoco/current`.

## Why

`tools/physics_contract_runner.py` combined MuJoCo scene/profile loading,
runtime body contract application, depth candidate selection, force-balance
calculation, neutral open-plant simulation dispatch, report assembly, file
output, and terminal printing in one function.

That made the static physics audit harder to maintain even though the audit is
supposed to be a contract check rather than a place to hide plant mismatch.

## Changed

- Added `tools/physics_contract_runner_context.py` for scene/profile/model/body
  contract context assembly.
- Added `tools/physics_contract_runner_depths.py` for Bar30/start-depth and
  audited-depth selection.
- Added `tools/physics_contract_runner_balances.py` for force-balance rows.
- Added `tools/physics_contract_runner_outputs.py` for report assembly, CSV/JSON
  writing, and terminal output.
- Kept `tools/physics_contract_runner.py` as the public runner preserving
  `run_physics_contract_audit(args)`.

## Validation

Focused smoke:

```text
physics_contract_runner_split_smoke PASS
```

Full gates:

```text
python3 -m compileall -q /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current /Users/kanghyunmin/Desktop/uuv_sim/uuv_control_gui.py
PYTHONPATH=/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools python3 /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools/audit_code_contract_sources.py --out-dir /private/tmp/uuv_source_audit_spaghetti_20260607_physics_runner_split
python3 /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools/check_runtime_readiness_policy.py && python3 /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools/verify_ardusub_thruster_contract.py --quiet
/Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools/physics_contract_audit.py --output-dir /private/tmp/uuv_physics_audit_spaghetti_20260607_physics_runner_split --simulate-s 0.05
```

Results:

```text
compileall PASS
source contract audit: {"fail": 0, "pass": 11, "warn": 5}
runtime_readiness_policy=PASS
[thruster-contract] OK
physics contract audit PASS, static force balance required_scale=1.000000
```

## Contract Notes

This pass changes audit orchestration only.  It does not change hydrostatic
coefficients, body mass/inertia/CoM/CoB values, buoyancy scale, start-depth
rules, neutral open-plant simulation math, actuator contracts, RC override
timing, or ArduPilot/SITL code.
