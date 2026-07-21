# ROS2 MAVROS command services split

Date: 2026-06-07

## Scope

- Added focused command service modules:
  - `bridge/ros2_command_payload.py`
  - `bridge/ros2_mavros_setpoint_services.py`
  - `bridge/ros2_mavros_arm_mode_services.py`
  - `bridge/ros2_sitl_command_override.py`
- Kept `bridge/ros2_mavros_command_services.py` as the compatibility export
  surface consumed by `bridge/ros2_bridge_commands.py`.

## Contract

The split preserves `/mavros/setpoint_raw/local`,
`/uuv_mujoco/sitl/command_override`, `/mavros/cmd/arming`,
`/mavros/set_mode`, and `/mavros/cmd/command` callback method names used by
the `Ros2Bridge` method-binding table.

## Verification

```text
python3 -m compileall -q uuv_mujoco/current/bridge/ros2_mavros_command_services.py uuv_mujoco/current/bridge/ros2_command_payload.py uuv_mujoco/current/bridge/ros2_mavros_setpoint_services.py uuv_mujoco/current/bridge/ros2_mavros_arm_mode_services.py uuv_mujoco/current/bridge/ros2_sitl_command_override.py
PYTHONPATH=uuv_mujoco/current python3 - <<'PY'
from bridge import ros2_bridge_commands, ros2_mavros_command_services
required = ['_forward_arm_request', '_forward_mode_request', '_on_mavros_cmd_arming', '_on_mavros_command_long', '_on_mavros_set_mode', '_on_mavros_setpoint', '_on_sitl_command_override', '_parse_command_bool', '_parse_command_override_payload']
missing = [name for name in required if not hasattr(ros2_bridge_commands, name) or not hasattr(ros2_mavros_command_services, name)]
print({'missing': missing})
PY
```

Smoke result: `missing=[]`.  The old command service hotspot no longer appears
in the top 35 inventory.
