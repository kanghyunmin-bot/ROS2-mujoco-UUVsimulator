# Profile, Sensor, and RCOut Contract Split

Date: 2026-06-10

## Scope

This pass targets contract-bearing runtime surfaces rather than display-only
hotspots:

- profile selection: profile alias/listing, built profile body, and thruster
  voltage override are separated so static and dynamic hydrodynamics experiments
  keep a visible runtime contract.
- sensor I/O: core ROS message factories for IMU, `/depth`, `/depth/pose`,
  `/mavros/imu/static_pressure`, ground truth, and sim time are separated from
  the lazy publish cache.
- controller-parity observation: SITL MAVLink `SERVO_OUTPUT_RAW` to
  MAVROS-compatible `/mavros/rc/out` mirroring now separates timestamp source,
  RCOut message construction, and event-publish policy.

The split is behavior-neutral. It does not change ArduSub parameters,
controller output remaps, PWM correction, plant input ownership, or MuJoCo
fluid coefficients.

## Files

- `sim/physics/profile_runtime.py`
- `sim/physics/profile_runtime_types.py`
- `sim/physics/profile_runtime_listing.py`
- `sim/physics/profile_runtime_selection.py`
- `sim/physics/profile_runtime_voltage.py`
- `bridge/ros2_publish_builder_core.py`
- `bridge/ros2_publish_core_cache.py`
- `bridge/ros2_publish_core_factories.py`
- `bridge/ros2_rcout_telemetry.py`
- `bridge/ros2_rcout_stamp.py`
- `bridge/ros2_rcout_message.py`
- `bridge/ros2_rcout_publish.py`

## Result

Removed these files from the top structural-complexity inventory:

- `sim/physics/profile_runtime.py`
- `bridge/ros2_publish_builder_core.py`
- `bridge/ros2_rcout_telemetry.py`

One concrete coupling bug was removed: importing
`bridge/ros2_publish_builder_core.py` no longer imports `RosPublishState` at
runtime and therefore no longer requires MuJoCo just to unit-test core message
builders.

## Validation

```text
compileall: PASS
profile_runtime_split_smoke=PASS
ros2_publish_core_split_smoke=PASS
rcout_telemetry_split_smoke=PASS
gui_readiness_contract=PASS
gui_backend_selection=PASS
runtime_readiness_policy=PASS
rc_frame_contract=PASS
verify_ardusub_thruster_contract.py --quiet: OK
audit_code_contract_sources.py: {"fail": 0, "pass": 15, "warn": 5}
audit_closed_loop_contract.py: real_vs_sitl_mismatches={}, missing_sitl_params=[]
```
