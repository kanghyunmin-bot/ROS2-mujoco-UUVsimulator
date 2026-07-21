# Roll Stability Sweep Loop Split

Date: 2026-06-07

Scope: active tools under `uuv_mujoco/current/tools`.

## What Changed

- Moved sweep path construction into `roll_stability_sweep_paths.py`.
- Moved original profile/scene/mapping snapshot and restoration helpers into `roll_stability_sweep_files.py`.
- Moved candidate apply/run/summary/final-result loop into `roll_stability_sweep_loop.py`.
- Kept `roll_stability_sweep.py` as the public CLI for argument parsing and candidate selection.

This is a structure-only refactor. It preserves candidate order, temporary
file edit order, reset behavior, launcher readiness behavior, summary writing,
and guaranteed original-file restoration.

## Focused Smoke

```text
python3 /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools/roll_stability_sweep.py --help
PYTHONPATH=/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools python3 - <<'PY'
# fake-run candidate loop verifies summary calls and original profile/scene/mapping restoration.
PY
```

Results:

```text
roll_stability_sweep.py --help: PASS
roll_stability_sweep loop focused smoke: PASS
```

## Contract Gates

```text
python3 -m compileall -q /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current /Users/kanghyunmin/Desktop/uuv_sim/uuv_control_gui.py
PYTHONPATH=/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools python3 /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools/audit_code_contract_sources.py --out-dir /private/tmp/uuv_source_audit_spaghetti_20260607_sweep_autotune_split
python3 /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools/check_runtime_readiness_policy.py
python3 /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools/verify_ardusub_thruster_contract.py --quiet
/Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools/physics_contract_audit.py --output-dir /private/tmp/uuv_physics_audit_spaghetti_20260607_sweep_autotune_split --simulate-s 0.05
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
