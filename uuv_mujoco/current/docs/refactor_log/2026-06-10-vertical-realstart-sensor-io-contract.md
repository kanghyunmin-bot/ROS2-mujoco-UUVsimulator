# Vertical, Real-Start, and Sensor I/O Contract Split

Date: 2026-06-10

## Scope

This pass keeps controller and plant behavior unchanged.  It reduces hotspot
size around the Bar30/vertical startup path and adds source-contract coverage
for the sensor input/output split that must stay fixed before plant replay,
HAN, or CFD tuning is trusted.

Covered contracts:

- SITL JSON sensor input is generated from the MuJoCo snapshot as timestamp,
  FRD IMU, position, velocity, attitude, and quaternion.
- Bar30 pressure/depth is still the vertical datum for the SITL pressure path.
- ROS core, MAVROS, and DVL observation topics are published from the same
  snapshot used for SITL sensor input.
- RC in/out, raw plant input, thruster conversion, timing, and dynamic
  `fluidcoef` gates remain separate source-audit checks.

## Files

- `bridge/ros2_state_sitl_vertical.py`
- `bridge/ros2_state_sitl_baro.py`
- `bridge/ros2_state_sitl_velocity.py`
- `bridge/ros2_state_sitl_frames.py`
- `tools/real_start_baro.py`
- `tools/real_start_baro_candidates.py`
- `tools/real_start_baro_row.py`
- `tools/audit_code_contract_paths.py`
- `tools/audit_code_contract_runtime_sensor_io.py`
- `tools/audit_code_contract_runtime_surface_ext.py`
- `docs/architecture/ACTIVE_CONTRACT_WORKLIST.md`
- `docs/architecture/SPAGHETTI_AUDIT.md`

## Result

Removed these previous hotspots from the top structural inventory:

- `bridge/ros2_state_sitl_vertical.py`
- `tools/real_start_baro.py`
- `tools/real_start_baro_candidates.py`

The new sensor I/O audit module is also below the top 20 inventory after being
split into snapshot/SITL, JSON payload, ROS publish, and evidence helpers.

## Validation

```text
ros2_state_sitl_vertical_split_smoke=PASS
real_start_baro_row_split_smoke=PASS
compileall: PASS
audit_code_contract_sources.py: {"fail": 0, "pass": 16, "warn": 5}
audit_closed_loop_contract.py: real_vs_sitl_mismatches={}, missing_sitl_params=[]
gui_readiness_contract=PASS
gui_backend_selection=PASS
runtime_readiness_policy=PASS
rc_frame_contract=PASS
verify_ardusub_thruster_contract.py --quiet: OK
```
