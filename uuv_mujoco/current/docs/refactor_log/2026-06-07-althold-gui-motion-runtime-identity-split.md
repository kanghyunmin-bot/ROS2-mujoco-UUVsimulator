# ALT_HOLD, GUI Motion, And Runtime Identity Split

Date: 2026-06-07

Scope: active runtime under `uuv_mujoco/current`, backed by the compatibility
directory `uuv_mujoco/v2.2`.

## Why

The workspace still exposes a `v2.2` backing directory, which is easy to mistake
for a stale runtime.  The executable contract is now the active alias
`uuv_mujoco/current` plus `uuv_mujoco/RUNTIME_VERSION.json`; the backing
directory name is retained only for compatibility with existing scripts,
reports, and debug evidence.

This pass also removes several callback/runtime monoliths from the hotspot
list without changing controller-parity, sensor, RC override, or plant-input
contracts.

## Changes

- Split `tools/althold_diagnostics_node.py` into ROS import, subscription,
  callback, and runtime/output ownership modules.
- Split `gui/node_motion_callbacks.py` into IMU, pose/odom, velocity, and
  pressure/depth callback modules.
- Removed GUI telemetry callback import-time coupling to ROS helpers by making
  RC padding and severity-name mapping local to their focused callback modules.
- Split active-runtime identity auditing into input collection, evaluation, and
  check construction modules.
- Documented that reports should describe the live runtime as
  `current-2026-06-07-uuv_sim`, not as "latest v2.2".

## Contract Guardrails

- ArduPilot source and submodule pointer were not modified.
- `uuv_mujoco/current` remains the active launch/runtime path.
- `uuv_mujoco/v2.2` remains a compatibility backing directory only.
- Controller parity observation points remain unchanged:
  real `/mavros/rc/out` versus SITL MAVLink `SERVO_OUTPUT_RAW`.
- Closed-loop plant input remains raw ArduSub JSON servo output.

## Verification

```text
python3 -m compileall -q uuv_mujoco/current uuv_control_gui.py
PYTHONPATH=uuv_mujoco/current/tools python3 uuv_mujoco/current/tools/check_runtime_freshness.py --workspace /Users/kanghyunmin/Desktop/uuv_sim --runtime-dir /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current --fetch --refresh-version
PYTHONPATH=uuv_mujoco/current/tools python3 uuv_mujoco/current/tools/audit_code_contract_sources.py --out-dir /private/tmp/uuv_source_audit_spaghetti_goal_next
python3 uuv_mujoco/current/tools/check_runtime_readiness_policy.py
python3 uuv_mujoco/current/tools/verify_ardusub_thruster_contract.py --quiet
python3 uuv_mujoco/current/tools/check_dev_os_compat.py --headless --target-os ubuntu
/Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python uuv_mujoco/current/tools/physics_contract_audit.py --output-dir /private/tmp/uuv_physics_audit_spaghetti_goal_next --simulate-s 0
git diff --check
```

Results:

- Runtime freshness: `PASS`, `uuv_mujoco/current -> v2.2`.
- Source contract audit: `fail=0`, `pass=11`, `warn=5`.
- Runtime readiness policy: `PASS`.
- ArduSub thruster contract: `OK`.
- Dev OS compatibility: `fail=0`, `pass=16`, `warn=2`.
- Static physics balance: `net_down=+0.000N`, `required_scale=1.000000`.
- Diff whitespace check: clean.

Remaining expected warnings:

- Docker daemon was not running in this shell.
- ROS2 was not sourced in this shell; launchers may source their configured
  environment.
