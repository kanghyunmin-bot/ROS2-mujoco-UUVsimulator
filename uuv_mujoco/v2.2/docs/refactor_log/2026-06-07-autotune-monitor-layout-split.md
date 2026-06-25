# Auto-Tune Monitor Layout Split

Date: 2026-06-07

Scope: active GUI runtime under `uuv_mujoco/current/gui`.

## What Changed

- Split shell/header/progress/window reuse into `autotune_monitor_layout_shell.py`.
- Split candidate table and chart canvas layout into `autotune_monitor_layout_candidates.py`.
- Split live-log text area and scrollbars into `autotune_monitor_layout_log.py`.
- Kept `autotune_monitor_layout.py` as the layout facade.
- Kept `autotune_monitor_window.py` as the public lifecycle/reset/finish entry point.

This is a structure-only refactor. It preserves window title, geometry,
minimum size, delete protocol, owner attribute names, tree columns, chart
canvas settings, log text settings, reset behavior, and finish status mapping.

## Focused Smoke

```text
PYTHONPATH=/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current python3 - <<'PY'
# fake-owner smoke verifies existing-window raise, hide, reset, and failure status mapping.
PY
python3 -m compileall -q .../autotune_monitor_window.py .../autotune_monitor_layout*.py
```

Results:

```text
autotune_monitor_layout split focused smoke: PASS
compileall: PASS
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
