# GUI Layout and Commanding Split

Date: 2026-06-07

Scope: active runtime GUI code exposed through `uuv_mujoco/current`.

This change addresses two stale active-runtime hotspots while keeping the
compatibility backing directory name `uuv_mujoco/v2.2` unchanged.  The live
runtime remains:

```text
uuv_mujoco/current -> v2.2
```

## Changes

- `gui/layout_mixin.py` is now a facade.
- Layout construction moved into focused modules:
  - `gui/layout_shell.py`
  - `gui/layout_telemetry.py`
  - `gui/layout_control_core.py`
  - `gui/layout_control_replay.py`
  - `gui/layout_control_tuning.py`
  - `gui/layout_control_pilot.py`
- `gui/node_commanding.py` is now a facade.
- GUI command logic moved into focused modules:
  - `gui/node_commanding_common.py`
  - `gui/node_arm_mode_commands.py`
  - `gui/node_rc_publishers.py`

## Contract Position

This is a behavior-neutral refactor.  It does not change:

- ArduPilot source or submodule pointer.
- Controller parity observation point.
- RC override channel mapping.
- Arm/mode retry and readiness policy.
- SITL JSON servo plant input.
- Thruster/PWM mapping.

## Validation

```text
python3 -m py_compile uuv_mujoco/v2.2/gui/node_commanding.py \
  uuv_mujoco/v2.2/gui/node_commanding_common.py \
  uuv_mujoco/v2.2/gui/node_arm_mode_commands.py \
  uuv_mujoco/v2.2/gui/node_rc_publishers.py \
  uuv_mujoco/v2.2/gui/node.py uuv_control_gui.py
python3 -m compileall -q uuv_mujoco/current uuv_control_gui.py
source ./.uuv_mujoco_env.sh && "$MJ311_PYTHON" \
  uuv_mujoco/current/run_uuv_mujoco.py --help
python3 uuv_mujoco/current/tools/check_runtime_readiness_policy.py
python3 uuv_mujoco/current/tools/verify_ardusub_thruster_contract.py --quiet
PYTHONPATH=uuv_mujoco/current/tools python3 \
  uuv_mujoco/current/tools/audit_code_contract_sources.py \
  --out-dir /private/tmp/uuv_code_contract_after_layout_commanding_split
source ./.uuv_mujoco_env.sh && PYTHONPATH=uuv_mujoco/current/tools \
  "$MJ311_PYTHON" uuv_mujoco/current/tools/physics_contract_audit.py \
  --output-dir /private/tmp/uuv_physics_contract_after_layout_commanding_split \
  --simulate-s 0
python3 uuv_mujoco/current/tools/check_dev_os_compat.py --headless --target-os ubuntu
git diff --check -- <changed gui files>
```

Results:

- Python compile: PASS.
- Full active-runtime compileall: PASS.
- Runtime help entrypoint: PASS.
- Runtime readiness policy: PASS.
- ArduSub thruster contract: PASS.
- Code contract audit: `fail=0`, `pass=10`, `warn=5`.
- Static physics audit: neutral static balance `net_down=+0.000N`,
  `required_scale=1.000000`.
- Dev OS compatibility: `fail=0`, `pass=16`, `warn=2`.

The remaining dev OS warnings are environmental, not refactor failures:

- Docker daemon is not running.
- ROS2 is not sourced in the current shell, although the GUI launcher can source
  its configured environment.
