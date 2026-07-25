# Contract Axes Continuation: Time, Sensor I/O, RC I/O, Thruster, Dynamic Fluid

Date: 2026-06-11

## Scope

- Edits stayed inside `uuv_mujoco/v2.2`.
- ArduPilot source and the submodule pointer were not intentionally changed.
- No ALT_HOLD shim, PWM remap, or output correction was added.
- This pass was structural: make contract lanes easier to audit and preserve existing behavior.

## Contract Lanes Touched

- Time/phase:
  - Split source-audit time checks into `tools/audit_code_contract_runtime_time_sources.py`.
  - Split passive viewer execution from runtime wiring into `sim/runtime/simulation_loop_viewer_runner.py`.
  - Split catch-up/sleep loops into `sim/runtime/simulation_loop_catchup.py`.
- Sensor I/O:
  - Split Bar30 initial-hold velocity handling from pressure/depth construction into `bridge/ros2_sitl_sensor_vertical_hold.py`.
  - Split Ping360 blind-zone/noise/fade helpers into `bridge/ping360_profile_noise.py`.
- RC input/output:
  - Split axis command clamp/value helpers into `tools/axis_rc_command_values.py`.
  - Split OverrideRCIn builders into `tools/axis_rc_override_messages.py`.
  - Split ManualControl builders into `tools/axis_rc_manual_messages.py`.
  - Split SITL vehicle heartbeat target matching into `bridge/sitl_vehicle_heartbeat_target.py`.
- Thruster contract:
  - Split active runtime thruster command, force write, and wrench evidence in the source audit.
  - Added `tools/audit_code_contract_thruster_conversion.py`.
  - Split command-to-force conversion in `sim/physics/thruster_force_model.py` into performance-curve and polynomial fallback paths.
- Dynamic ellipsoid/fluid/CFD:
  - Split dynamic fluidcoef pattern loop into `sim/physics/dynamic_fluidcoef_setup_patterns.py`.
  - Split dynamic fluidcoef enable-state handling into `sim/physics/dynamic_fluidcoef_setup_enable.py`.
  - Split CFD dynamic wrench axis table validation into `sim/physics/cfd_dynamic_wrench_axis_tables.py`.
- Static plant/hydrostatic:
  - Split hydrostatic component mass and buoyancy-share numeric validation into `physics/sim_profile_hydrostatic_component_numbers.py`.
- Dev OS/runtime paths:
  - Split active runtime alias and path-group checks into:
    - `tools/dev_os_compat_sitl_alias.py`
    - `tools/dev_os_compat_path_checks.py`

## Validation

```text
compileall tools/gui/bridge/sim/physics: PASS
audit_code_contract_sources: fail=0 pass=18 warn=6
audit_closed_loop_contract: real_vs_sitl_mismatches={} missing_sitl_params=[]
check_dev_os_compat: fail=0 pass=16 warn=2
dynamic_fluidcoef_contract=PASS
fossen_runtime_builders=PASS
physics_contract_geometry=PASS
physics_runtime_hydrostatic=PASS
hydrostatic_buoyancy_points=PASS
immediate_sensor_replay_reply=PASS
runtime_readiness_policy=PASS
sitl_servo_runtime=PASS
gui_readiness_contract=PASS
gui_backend_selection=PASS
rc_frame_contract=PASS
ros2_command_payload=PASS
ros2_sitl_command_override=PASS
sitl_command_link_readiness=PASS
thruster_param_loader=PASS
verify_ardusub_thruster_contract=OK
filter_ping360_stl_io=PASS
```

Known warnings:

- Docker CLI exists, but the local Docker daemon socket was not reachable in this shell.
- ROS2 was not sourced in this shell; the GUI launcher may source its configured environment.

## Inventory Impact

Removed or lowered these contract-related hotspots from the current top inventory:

- `tools/audit_code_contract_thruster_gate_checks.py`
- `tools/audit_code_contract_runtime_time.py`
- `sim/physics/dynamic_fluidcoef_setup.py`
- `bridge/ros2_sitl_sensor_vertical.py`
- `sim/runtime/simulation_loop_runtime.py`
- `sim/runtime/simulation_loop_clocks.py`
- `sim/physics/cfd_dynamic_wrench_axis_parser.py`
- `tools/dev_os_compat_sitl_paths.py`
- `bridge/sitl_vehicle_heartbeat_filter.py`
- `tools/axis_rc_messages.py`
- `physics/sim_profile_hydrostatic_component_fields.py`
- `sim/physics/thruster_force_model.py`
- `bridge/ping360_profile_signal.py`

The next highest items are mostly GUI mixins, GUI widgets, replay worker glue, startup subscriptions, and formatting/logging utilities. Those are lower risk than the controller/plant contract lanes but still need cleanup before calling the workspace low-spaghetti.
