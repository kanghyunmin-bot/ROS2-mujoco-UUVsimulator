# Hotspot Continuation Split

Date: 2026-06-10

## Scope

This pass continues reducing active-runtime structural hotspots without
changing controller, plant-input, or physics coefficients.

Split surfaces:

- Thruster debug runtime emission policy.
- Low-level MAVLink command-link state, connection, and send wrappers.
- Runtime pool geometry depth/XY overrides.
- Lazy ROS2 import loading.
- MAVROS setpoint position parsing and `CONDITION_YAW` handling.

## Files

- `sim/runtime/thruster_debug_runtime.py`
- `sim/runtime/thruster_debug_runtime_emit.py`
- `sim/transport/mavlink_command_link.py`
- `sim/transport/mavlink_command_link_state.py`
- `sim/transport/mavlink_command_link_connection.py`
- `sim/transport/mavlink_command_link_sends.py`
- `sim/physics/pool_runtime_overrides.py`
- `sim/physics/pool_runtime_geom.py`
- `sim/physics/pool_runtime_depth.py`
- `sim/physics/pool_runtime_xy.py`
- `bridge/ros2_bridge_imports.py`
- `bridge/ros2_bridge_import_core.py`
- `bridge/ros2_bridge_import_messages.py`
- `bridge/ros2_mavros_setpoint_services.py`
- `bridge/ros2_mavros_setpoint_position.py`
- `bridge/ros2_mavros_condition_yaw.py`

## Result

Removed these files from the top structural-complexity inventory:

- `sim/runtime/thruster_debug_runtime.py`
- `sim/transport/mavlink_command_link.py`
- `sim/physics/pool_runtime_overrides.py`
- `bridge/ros2_bridge_imports.py`
- `bridge/ros2_mavros_setpoint_services.py`

Current top hotspot after this pass:

- `bridge/ros2_state_sitl_vertical.py`

## Validation

```text
thruster_debug_runtime_emit_split_smoke=PASS
mavlink_command_link_mixin_split_smoke=PASS
pool_runtime_overrides_split_smoke=PASS
ros2_bridge_import_split_smoke=PASS
ros2_mavros_setpoint_split_smoke=PASS
compileall: PASS
audit_code_contract_sources.py: {"fail": 0, "pass": 15, "warn": 5}
audit_closed_loop_contract.py: real_vs_sitl_mismatches={}, missing_sitl_params=[]
gui_readiness_contract=PASS
gui_backend_selection=PASS
runtime_readiness_policy=PASS
rc_frame_contract=PASS
verify_ardusub_thruster_contract.py --quiet: OK
```

The pass preserves the RC override path, MAVLink command-link public API,
thruster debug CSV sampling cadence, pool override runtime semantics, ROS2
missing-package error behavior, and MAVROS setpoint callback names.
