# ROS2 Bridge and Dynamic Fluidcoef Split

Date: 2026-06-07

## Scope

This is a behavior-neutral refactor inside the active runtime
`sim/current` with the physical backing directory still named
`uuv_mujoco/v2.2`.  It does not modify ArduPilot, the ArduPilot submodule
pointer, controller parity shims, PWM correction, output remaps, or plant input
semantics.

## ROS2 Bridge

Changed:

- Added `bridge/ros2_bridge_runtime_setup.py` for base bridge state, command
  runtime state, spin state, and MAVROS/pressure/frame contract application.
- Added `bridge/ros2_bridge_sensor_setup.py` for MuJoCo sensor ID lookup and
  optional Ping360 runtime setup.
- Added `bridge/ros2_bridge_method_bindings.py` for the compatibility method
  binding table previously embedded inside `Ros2Bridge`.
- Kept `bridge/ros2_bridge.py` as the public constructor facade.

Effect:

- `bridge/ros2_bridge.py`: `330 LOC / 3` branches -> `153 LOC / 1` branch.
- `Ros2Bridge` method surface is preserved by binding the same split-module
  implementations after class definition.

## Dynamic Fluidcoef

Changed:

- Added `sim/physics/dynamic_fluidcoef_runtime_config.py` for environment and
  profile parsing, transient state allocation, smoothing knobs, and debug flag
  setup.
- Kept `sim/physics/dynamic_fluidcoef_runtime.py` focused on per-step
  `geom_fluid` updates.

Effect:

- `sim/physics/dynamic_fluidcoef_runtime.py`: `297 LOC / 15` branches ->
  `139 LOC / 13` branches.
- The load calculation and MuJoCo `model.geom_fluid[idx, 1:6]` application are
  unchanged.

## Validation

```text
python3 -m py_compile \
  sim/current/bridge/ros2_bridge.py \
  sim/current/bridge/ros2_bridge_runtime_setup.py \
  sim/current/bridge/ros2_bridge_sensor_setup.py \
  sim/current/bridge/ros2_bridge_method_bindings.py \
  sim/current/sim/physics/dynamic_fluidcoef_runtime.py \
  sim/current/sim/physics/dynamic_fluidcoef_runtime_config.py

source ./.uuv_mujoco_env.sh
PYTHONPATH="sim/current:${PYTHONPATH:-}" "${MJ311_PYTHON:-python3}" - <<'PY'
from bridge.ros2_bridge import Ros2Bridge
required = [
    "_init_ros", "_safe_publish", "_on_mavros_rc_override",
    "_on_mavros_cmd_arming", "_estimate_sitl_vertical",
    "_build_and_send_sitl_sensor_snapshot", "_flush_ros_publish_jobs",
    "set_sitl_servo_handler", "set_replay_rcout_handler",
    "sitl_vehicle_armed", "sitl_vehicle_mode", "spin_once", "publish",
    "force_next_publish", "reset_odometry", "shutdown",
]
print({name: callable(getattr(Ros2Bridge, name, None)) for name in required})
PY

python3 -m compileall -q sim/current uuv_control_gui.py
PYTHONPATH=sim/current/tools python3 \
  sim/current/tools/audit_code_contract_sources.py \
  --out-dir /private/tmp/uuv_code_contract_after_ros2_bridge_split
python3 sim/current/tools/check_runtime_readiness_policy.py
python3 sim/current/tools/verify_ardusub_thruster_contract.py --quiet
python3 sim/current/tools/check_dev_os_compat.py --headless --target-os ubuntu
python3 sim/current/tools/refactor_inventory.py --limit 40
git diff --check
```

Observed:

- Compile and import-surface checks passed.
- Source-contract audit: `fail=0`, `pass=11`, `warn=5`.
- Runtime readiness policy: `PASS`.
- Thruster contract: `OK`.
- Development OS compatibility: `fail=0`, `pass=16`, `warn=2`.
- `git diff --check`: passed.
