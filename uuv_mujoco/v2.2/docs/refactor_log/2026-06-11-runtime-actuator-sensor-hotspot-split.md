# Runtime Actuator Sensor Hotspot Split

Date: 2026-06-11

## Scope

- Kept edits inside `uuv_mujoco/v2.2`.
- Did not modify ArduPilot source or intentionally change the submodule pointer.
- Preserved the controller/plant observation boundary:
  - controller parity remains MAVLink telemetry-to-telemetry
  - closed-loop plant input remains raw SITL JSON servo PWM
  - plant replay input remains explicit recorded RCOU/PWM

## Runtime Flow Changes

- Split `sim/runtime/simulation_step_runtime.py` state fields into
  `sim/runtime/simulation_step_state.py`.
- Split dedicated MAVLink command-link polling into:
  - `bridge/sitl_mavlink_command_receive.py`
  - `bridge/sitl_mavlink_command_handlers.py`
  - facade `bridge/sitl_mavlink_command_polling.py`
- Split runtime loop entry into:
  - `sim/runtime/runtime_loop_modes.py`
  - `sim/runtime/runtime_loop_shutdown.py`
  - facade `sim/runtime/runtime_loop_entry.py`

## Actuator Contract Changes

- Split raw SITL/plant-replay servo runtime into:
  - `sim/runtime/sitl_servo_state_data.py`
  - `sim/runtime/sitl_servo_factory.py`
  - `sim/runtime/sitl_servo_packet_apply.py`
  - `sim/runtime/sitl_servo_target_apply.py`
  - `sim/runtime/sitl_servo_labels.py`
  - facade `sim/runtime/sitl_servo_state.py`
- Added `tools/check_sitl_servo_runtime.py` to verify:
  - raw PWM packet copy
  - PWM-to-normalized command conversion
  - servo sign application
  - target scale application
  - stale-packet timeout clearing
- Split per-thruster parameter application into:
  - `sim/physics/thruster_param_gain_apply.py`
  - `sim/physics/thruster_param_optional_apply.py`
  - facade `sim/physics/thruster_param_per_thruster.py`

## Sensor/Readiness Changes

- Split DVL compatibility message builders into:
  - `bridge/ros2_dvl_header.py`
  - `bridge/ros2_dvl_velocity_msg.py`
  - `bridge/ros2_dvl_pose_msg.py`
  - facade `bridge/ros2_dvl_messages.py`
- Added `tools/check_ros2_dvl_messages.py` to verify DVL velocity, altitude,
  pose, frame_id, and `wxyz` quaternion yaw handling.
- Split operator control readiness policy into:
  - `sim/runtime/readiness_control_preconditions.py`
  - `sim/runtime/readiness_control_limited.py`
  - facade `sim/runtime/readiness_control_label.py`

## Validation

```text
compileall tools/bridge/sim/physics/gui: PASS
sitl_servo_runtime=PASS
ros2_dvl_messages=PASS
runtime_readiness_policy=PASS
thruster_param_loader=PASS
ros2_replay_rcout=PASS
ardusub_thruster_contract=OK
contract_source_audit: fail=0 pass=17 warn=6
closed_loop_contract: real_vs_sitl_mismatches={} missing_sitl_params=[]
closed_loop_contract: thruster_voltage=20.0 dynamic_fluidcoef.active=false
```

## Inventory Impact

The following files were removed from the top 45 refactor inventory:

- `sim/runtime/simulation_step_runtime.py`
- `bridge/sitl_mavlink_command_polling.py`
- `sim/runtime/sitl_servo_state.py`
- `sim/runtime/runtime_loop_entry.py`
- `sim/runtime/readiness_control_label.py`
- `bridge/ros2_dvl_messages.py`
- `sim/physics/thruster_param_per_thruster.py`

Remaining high-priority hotspots are mostly tooling/diagnostics plus selected
GUI/runtime helpers:

- `tools/filter_ping360_stl_io.py`
- `tools/roll_stability_probe_commands.py`
- `tools/physics_contract_geometry.py`
- `tools/althold_contract_plot.py`
- `gui/node.py`
- `sim/validation/plant_input_gate_csv.py`
