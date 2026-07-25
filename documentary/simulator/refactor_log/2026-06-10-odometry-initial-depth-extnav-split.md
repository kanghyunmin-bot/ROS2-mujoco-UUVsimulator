# Odometry, Initial Depth, And ExternalNav Contract Split

Date: 2026-06-10

## Scope

- Kept edits inside `uuv_mujoco/v2.2`.
- Preserved the controller-parity and plant-input boundaries.
- Did not modify ArduPilot source or the submodule pointer.

## Changes

- Split odometry publish builders into:
  - `bridge/ros2_publish_odometry_messages.py`
  - `bridge/ros2_publish_odometry_tf.py`
  - compatibility facade `bridge/ros2_publish_builder_odometry.py`
- Removed an unnecessary runtime import from the odometry builder by moving
  `RosPublishState` behind `TYPE_CHECKING`.  Importing the builder no longer
  pulls in `mujoco` through `ros2_sitl_sensor_feed`.
- Added `tools/check_odometry_publish_builders.py` to verify:
  - lazy message cache behavior
  - `odom` versus `map` frames
  - zero DVL fallback twist
  - simulated odometry angular velocity
  - `map -> odom -> base_link` TF shape
- Split GUI initial-depth contract into:
  - `gui/sim_stack_initial_depth_sources.py`
  - `gui/sim_stack_initial_depth_base_link.py`
  - compatibility facade `gui/sim_stack_initial_depth_contract.py`
- Added `tools/check_gui_initial_depth_contract.py` to verify:
  - explicit launch args win
  - real-start state is handled by the launcher
  - Bar30 depth is preferred over base-link debug depth
  - base-link hold-until-release emits the expected launch args
- Split ExternalNav contract into:
  - `bridge/sitl_external_nav_contract_errors.py`
  - `bridge/sitl_external_nav_contract_freshness.py`
  - compatibility facade `bridge/sitl_external_nav_contract.py`
- Added `tools/check_sitl_external_nav_contract.py` to verify:
  - disabled/fault/no-send/stale error paths
  - grace-period behavior
  - native VPD replay freshness before first event and stale replay sample

## Validation

```text
compileall tools/bridge/sim/physics/gui: PASS
contract_source_audit: fail=0 pass=16 warn=5
closed_loop_contract: real_vs_sitl_mismatches={} missing_sitl_params=[]
odometry_publish_builders=PASS
gui_initial_depth_contract=PASS
sitl_external_nav_contract=PASS
runtime_readiness_policy=PASS
gui_readiness_contract=PASS
gui_backend_selection=PASS
rc_frame_contract=PASS
ardusub_thruster_contract=OK
```

## Current Inventory Impact

The following previous top hotspots are no longer in the top 20 inventory:

- `bridge/ros2_publish_builder_odometry.py`
- `gui/sim_stack_initial_depth_contract.py`
- `bridge/sitl_external_nav_contract.py`

Remaining high-score areas are now concentrated around Ping360 STL IO, Fossen
residual setup, real-start measurements, initial hold pose, thruster curve/load
parsing, and replay RCOU handling.
