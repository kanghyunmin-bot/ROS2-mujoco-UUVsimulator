# Contract Surface Hotspot Split

Date: 2026-06-10

## Scope

This pass reduces structural hotspots around the active runtime contracts that
must stay explicit before any MuJoCo physics tuning:

- sensor I/O: IMU source, SITL JSON accel contract, ROS IMU accel surface, and
  ROS static-pressure source selection
- command readiness: auto-ready wait gates, arm request, and mode request
- time/phase: sensor replay interpolation index, numeric interpolation, and
  interpolated frame assembly
- hydrodynamics profile parsing: scalar, array, and vector profile field
  parsing used by ellipsoid/hydrodynamics config
- thruster validation: golden-check CSV loading, phase windows, column
  selection, row filtering, and metrics

The split is behavior-neutral. It does not change ArduPilot, controller parity
comparison surfaces, RC remaps, PWM correction, or plant input semantics.

## Files

- `bridge/ros2_bridge_config_imu.py`
- `bridge/ros2_bridge_config_imu_source.py`
- `bridge/ros2_bridge_config_imu_sitl.py`
- `bridge/ros2_bridge_config_imu_ros_surface.py`
- `bridge/ros2_bridge_config_static_pressure.py`
- `bridge/sitl_auto_ready_sequence.py`
- `bridge/sitl_auto_ready_gates.py`
- `bridge/sitl_auto_ready_actions.py`
- `bridge/sitl_replay_interpolation.py`
- `bridge/sitl_replay_interp_index.py`
- `bridge/sitl_replay_interp_math.py`
- `bridge/sitl_replay_interp_frame.py`
- `physics/sim_profile_parse_common.py`
- `physics/sim_profile_parse_scalar.py`
- `physics/sim_profile_parse_array.py`
- `physics/sim_profile_parse_vectors.py`
- `tools/control_loop_golden_thrusters.py`
- `tools/control_loop_golden_thruster_io.py`
- `tools/control_loop_golden_phase_windows.py`
- `tools/control_loop_golden_thruster_columns.py`
- `tools/audit_code_contract_paths.py`
- `tools/audit_code_contract_runtime_static_pressure.py`

## Result

Removed these files from the top structural-complexity inventory:

- `bridge/ros2_bridge_config_imu.py`
- `bridge/sitl_auto_ready_sequence.py`
- `physics/sim_profile_parse_common.py`
- `bridge/sitl_replay_interpolation.py`
- `tools/control_loop_golden_thrusters.py`

The source-contract audit still directly covers timing, RC in/out, raw plant
input ownership, Bar30/static-pressure output, thruster mapping, and opt-in
dynamic MuJoCo ellipsoid `fluidcoef` updates.

## Validation

```text
compileall: PASS
sim_profile_parser_split_smoke=PASS
sitl_replay_interpolation_split_smoke=PASS
control_loop_golden_thrusters_split_smoke=PASS
gui_readiness_contract=PASS
gui_backend_selection=PASS
runtime_readiness_policy=PASS
rc_frame_contract=PASS
verify_ardusub_thruster_contract.py --quiet: OK
audit_code_contract_sources.py: {"fail": 0, "pass": 15, "warn": 5}
audit_closed_loop_contract.py: real_vs_sitl_mismatches={}, missing_sitl_params=[]
```
