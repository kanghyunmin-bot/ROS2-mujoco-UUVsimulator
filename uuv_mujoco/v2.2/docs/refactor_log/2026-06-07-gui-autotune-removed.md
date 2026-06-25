# GUI Auto-Tune Removed

Date: 2026-06-07

Scope: `uuv_mujoco/current/gui`

## Change

The GUI auto-tune feature was removed from the active GUI runtime.  This was
not a visual hide-only change:

- `UuvControlGui` no longer inherits `AutoTuneMixin`.
- The tuning panel now builds only the physics tuning section.
- GUI auto-tune Tk variables, process/thread state, monitor state, lifecycle
  cleanup, path constants, and Python resolver exports were removed.
- GUI auto-tune launch, subprocess, panel, monitor, candidate, chart, log, and
  layout helper modules were deleted.

Non-GUI research/tuning tools are left intact unless they are explicitly moved
or removed in a separate pass.

## Contract

This reduces GUI surface area and removes a stale workflow from normal
operation.  It does not change MuJoCo plant input, SITL MAVLink telemetry,
ArduPilot source, controller parity metrics, or runtime physics coefficients.

## Verification

```text
rg -n "autotune|Auto Tune|Auto-tune|auto tune|AUTOTUNE|resolve_autotune_python" /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/gui /Users/kanghyunmin/Desktop/uuv_sim/uuv_control_gui.py
PYTHONPATH=/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current /Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python <GUI import smoke>
python3 -m compileall -q /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current /Users/kanghyunmin/Desktop/uuv_sim/uuv_control_gui.py
PYTHONPATH=/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools python3 /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools/audit_code_contract_sources.py --out-dir /private/tmp/uuv_source_audit_spaghetti_20260607_gui_autotune_removed
python3 /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools/check_runtime_readiness_policy.py
python3 /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools/verify_ardusub_thruster_contract.py --quiet
/Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools/physics_contract_audit.py --output-dir /private/tmp/uuv_physics_audit_spaghetti_20260607_gui_autotune_removed --simulate-s 0.05
git -C /Users/kanghyunmin/Desktop/uuv_sim diff --check
```

Result: GUI auto-tune search returned no active GUI matches, GUI import smoke
`PASS`, source audit `fail=0 pass=11 warn=5`, readiness `PASS`, thruster
contract `OK`, physics contract audit completed.
