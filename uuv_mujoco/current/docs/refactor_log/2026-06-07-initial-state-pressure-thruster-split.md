# Initial State, Pressure Config, and Thruster Runtime Split

Date: 2026-06-07

Scope: active runtime startup, Bar30/IMU/vertical bridge configuration, and
MuJoCo thruster actuator update code under `uuv_mujoco/current`.

## Changed

- Split `sim/runtime/initial_state.py` into focused helpers:
  - `initial_state_depths.py`
  - `initial_state_pose.py`
  - `initial_state_pose_depth.py`
  - `initial_state_pose_transforms.py`
  - `initial_state_hold_capture.py`
- Split `bridge/ros2_bridge_config_pressure.py` into:
  - `ros2_bridge_config_baro.py`
  - `ros2_bridge_config_imu.py`
  - `ros2_bridge_config_vertical.py`
  - a small compatibility facade in `ros2_bridge_config_pressure.py`
- Updated the source-contract audit path registry and active runtime checks so
  Bar30/AP_Baro evidence is read from `ros2_bridge_config_baro.py` and
  `/mavros/imu/static_pressure` default evidence is read from
  `ros2_bridge_config_imu.py`.
- Split `sim/runtime/thruster_actuator_runtime.py` into:
  - `thruster_actuator_immersion.py`
  - `thruster_actuator_forces.py`
  - `thruster_actuator_visuals.py`
  - `thruster_command_targets.py`

## Validation

```text
python3 -m compileall -q \
  uuv_mujoco/current/sim/runtime/initial_state.py \
  uuv_mujoco/current/sim/runtime/initial_state_depths.py \
  uuv_mujoco/current/sim/runtime/initial_state_pose.py \
  uuv_mujoco/current/sim/runtime/initial_state_pose_depth.py \
  uuv_mujoco/current/sim/runtime/initial_state_pose_transforms.py \
  uuv_mujoco/current/sim/runtime/initial_state_hold_capture.py \
  uuv_mujoco/current/bridge/ros2_bridge_config.py \
  uuv_mujoco/current/bridge/ros2_bridge_config_pressure.py \
  uuv_mujoco/current/bridge/ros2_bridge_config_baro.py \
  uuv_mujoco/current/bridge/ros2_bridge_config_imu.py \
  uuv_mujoco/current/bridge/ros2_bridge_config_vertical.py \
  uuv_mujoco/current/sim/runtime/thruster_actuator_runtime.py \
  uuv_mujoco/current/sim/runtime/thruster_actuator_immersion.py \
  uuv_mujoco/current/sim/runtime/thruster_actuator_forces.py \
  uuv_mujoco/current/sim/runtime/thruster_actuator_visuals.py \
  uuv_mujoco/current/sim/runtime/thruster_command_targets.py

PYTHONPATH=uuv_mujoco/current python3 <initial-state, pressure-config, and thruster import smokes>

PYTHONPATH=uuv_mujoco/current/tools \
python3 uuv_mujoco/current/tools/audit_code_contract_sources.py \
  --out-dir /private/tmp/uuv_code_contract_after_pressure_config_split

python3 uuv_mujoco/current/tools/verify_ardusub_thruster_contract.py --quiet

python3 uuv_mujoco/current/tools/refactor_inventory.py \
  --root uuv_mujoco/current --limit 25 --format markdown
```

Observed local smoke results:

```text
initial-state smoke: entry/depth/pose helpers callable
pressure config smoke: {'facade': True, 'split': True}
thruster smoke: {'runtime': 'ThrusterActuatorRuntime', 'methods': True, 'helpers': True}
source audit: {"fail": 0, "pass": 11, "warn": 5}
thruster-contract: OK
```

## Boundary

No ArduPilot source, ArduPilot submodule pointer, controller shim, PWM remap, or
physics coefficient changed.  This was a behavior-neutral split to make startup,
sensor, and actuator contracts easier to inspect.
