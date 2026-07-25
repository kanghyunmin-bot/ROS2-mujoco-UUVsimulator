# Contract Audit, Runtime CLI, Diagnostics, and Bridge Config Split

Date: 2026-06-07

Scope: active runtime and tooling exposed through `sim/current`.

## Changes

- `tools/audit_code_contract_checks.py` reduced to a source-audit assembler.
- New source-audit modules:
  - `tools/audit_code_contract_paths.py`
  - `tools/audit_code_contract_source_identity.py`
  - `tools/audit_code_contract_firmware_checks.py`
  - `tools/audit_code_contract_runtime_checks.py`
  - `tools/audit_code_contract_thruster_gate_checks.py`
- `sim/runtime/cli.py` reduced to parser assembly.
- Runtime CLI option groups moved into:
  - `sim/runtime/cli_common.py`
  - `sim/runtime/cli_profile.py`
  - `sim/runtime/cli_ros2.py`
  - `sim/runtime/cli_sensor_video.py`
  - `sim/runtime/cli_sitl.py`
  - `sim/runtime/cli_initial_state.py`
  - `sim/runtime/cli_viewer.py`
- `tools/althold_diagnostics_logger.py` reduced to CLI/lifecycle.
- ALT_HOLD diagnostics split into:
  - `tools/althold_diagnostics_contract.py`
  - `tools/althold_diagnostics_node.py`
  - `tools/althold_diagnostics_output.py`
- `bridge/ros2_bridge_config.py` reduced to a compatibility export surface.
- Bridge configuration split into:
  - `bridge/ros2_bridge_config_mavros.py`
  - `bridge/ros2_bridge_config_pressure.py`
  - `bridge/ros2_bridge_config_frames.py`

## Contract Position

This is a behavior-neutral refactor.  It does not change:

- ArduPilot source or submodule pointer.
- Controller parity observation layer.
- Plant input layer.
- RC override mapping.
- Bar30 JSON position.z contract.
- Static-pressure source default.
- Thruster/PWM mapping.

The source contract audit was updated only to follow the new source locations.
The accepted source-audit result remains `fail=0`, `pass=10`, `warn=5`.

## Validation

```text
python3 -m compileall -q sim/current uuv_control_gui.py
source ./.uuv_mujoco_env.sh && "$MJ311_PYTHON" \
  sim/current/run_uuv_mujoco.py --help
python3 sim/current/tools/check_runtime_readiness_policy.py
python3 sim/current/tools/verify_ardusub_thruster_contract.py --quiet
PYTHONPATH=sim/current/tools python3 \
  sim/current/tools/audit_code_contract_sources.py \
  --out-dir /private/tmp/uuv_code_contract_after_contract_cli_diag_bridge_split_v2
source ./.uuv_mujoco_env.sh && PYTHONPATH=sim/current/tools \
  "$MJ311_PYTHON" sim/current/tools/physics_contract_audit.py \
  --output-dir /private/tmp/uuv_physics_contract_after_contract_cli_diag_bridge_split \
  --simulate-s 0
python3 sim/current/tools/check_dev_os_compat.py --headless --target-os ubuntu
git diff --check -- <changed files>
```

Results:

- Active runtime compileall: PASS.
- Runtime help entrypoint: PASS.
- Runtime readiness policy: PASS.
- ArduSub thruster contract: PASS.
- Code contract audit: `fail=0`, `pass=10`, `warn=5`.
- Static physics audit: neutral static balance `net_down=+0.000N`,
  `required_scale=1.000000`.
- Dev OS compatibility: `fail=0`, `pass=16`, `warn=2`.
- Diff whitespace check: PASS.

The remaining dev OS warnings are environmental:

- Docker daemon is not running.
- ROS2 is not sourced in the current shell.
