# Contract Hotspot Continuation: Time, Sensor, RC, Thruster, Fluid

Date: 2026-06-11

## Scope

- Kept edits inside `uuv_mujoco/v2.2`.
- Did not modify ArduPilot source.
- Preserved the controller/plant boundary:
  - controller parity: real `/mavros/rc/out` versus SITL MAVLink `SERVO_OUTPUT_RAW`
  - closed-loop plant input: raw ArduSub JSON servo PWM
  - plant replay input: explicit recorded RCOU/PWM

## Contract Coverage Confirmed

The source-contract matrix now covers these lanes as explicit gates:

- `time_phase`: sim-time sensor publishing, servo-frame replay clock, wall-time MAVLink polling, and viewer catch-up.
- `sensor_input_output`: one MuJoCo sensor snapshot feeds SITL JSON, ROS core topics, MAVROS topics, and DVL topics.
- `rc_input_output`: 18-channel RC override forwarding and `/mavros/rc/in` mirror preservation.
- `controller_output_and_plant_input`: `SERVO_OUTPUT_RAW` stays the controller parity surface, while raw JSON/replay PWM stays the plant input.
- `thruster_contract`: ArduSub motor output is converted once through the runtime thruster force path.
- `dynamic_ellipsoid_fluid`: MuJoCo five-coefficient `fluidcoef` dynamic updates are opt-in velocity-load updates; current clean baseline keeps them disabled.

## Refactor Changes

- Split arm/mode queue entrypoint logic:
  - `bridge/sitl_arm_mode_queue_arm.py`
  - `bridge/sitl_arm_mode_queue_mode.py`
  - facade `bridge/sitl_arm_mode_queue.py`
- Split axis RC health rules:
  - `tools/axis_rc_health_flags.py`
  - `tools/axis_rc_health_phase_rules.py`
  - `tools/axis_rc_health_response_rules.py`
  - facade `tools/axis_rc_health_rules.py`
- Split closed-loop profile audit helpers:
  - `tools/audit_closed_loop_profile_keys.py`
  - `tools/audit_closed_loop_profile_load.py`
  - `tools/audit_closed_loop_profile_active.py`
  - facade `tools/audit_closed_loop_profile.py`
- Split closed-loop parameter parsing:
  - `tools/audit_closed_loop_param_lines.py`
  - `tools/audit_closed_loop_param_start_sitl.py`
  - facade `tools/audit_closed_loop_param_io.py`
- Split initial Bar30/depth candidate sources:
  - `sim/runtime/initial_depth_body_components.py`
  - `sim/runtime/initial_depth_buoyancy_points.py`
  - facade `sim/runtime/initial_depth_profile_candidates.py`
- Split horizontal thruster allocation matrix helpers:
  - `sim/physics/horizontal_allocator_geometry.py`
  - facade/class `sim/physics/horizontal_allocator.py`
- Split SERVO_OUTPUT_RAW MAVLink polling:
  - `bridge/sitl_mavlink_servo_receive.py`
  - `bridge/sitl_mavlink_servo_drain.py`
  - facade `bridge/sitl_mavlink_servo_polling.py`

## Validation

```text
compileall tools/bridge/sim/physics/gui: PASS
dynamic_fluidcoef_contract=PASS
fossen_runtime_builders=PASS
runtime_readiness_policy=PASS
ros2_replay_rcout=PASS
sitl_servo_runtime=PASS
initial_hold_pose=PASS
gui_initial_depth_contract=PASS
thruster_param_loader=PASS
verify_ardusub_thruster_contract=OK
contract_source_audit: fail=0 pass=18 warn=6
closed_loop_contract: real_vs_sitl_mismatches={} missing_sitl_params=[]
closed_loop_contract: thruster_voltage=20.0 dynamic_fluidcoef.active=false
```

## Inventory Impact

Removed these previous hotspots from the current top inventory:

- `sim/runtime/initial_depth_profile_candidates.py`
- `sim/physics/horizontal_allocator.py`
- `bridge/sitl_mavlink_servo_polling.py`
- `tools/axis_rc_health_rules.py`
- `tools/audit_closed_loop_param_io.py`
- `tools/audit_closed_loop_profile.py`

The remaining top hotspots are mostly GUI diagnostics, Ping360 helpers, viewer drawing, thruster debug row formatting, and selected command/filter utilities.
