# ROS2 Sensor Message Builder Split

Date: 2026-06-07

## Scope

Split `bridge/ros2_sensor_messages.py` into focused IMU, fluid pressure, range,
battery, and MAVROS VFR HUD message builder modules while preserving the
historical compatibility import surface.

## Files

- `bridge/ros2_sensor_messages.py`
  - Compatibility export surface.
- `bridge/ros2_imu_messages.py`
  - IMU message fields and April 1 real-bag covariance contract.
- `bridge/ros2_pressure_messages.py`
  - `FluidPressure` static/atm pressure builder.
- `bridge/ros2_range_messages.py`
  - DVL altitude/range builder.
- `bridge/ros2_battery_messages.py`
  - Battery state builder.
- `bridge/ros2_vfr_hud_messages.py`
  - MAVROS VFR HUD compatibility builder.

## Contract Notes

- No ROS field names, covariance values, frame IDs, range limits, pressure
  variance values, or VFR HUD depth semantics were changed.
- Existing imports through `bridge.ros2_standard_messages` and
  `bridge.ros2_sensor_messages` remain valid.
- The split is behavior-neutral and exists to keep Bar30/static-pressure, IMU,
  and range/DVL message contracts independently auditable.

## Verification

```text
ros2_sensor_messages facade smoke: PASS
python3 -m compileall -q sim/current uuv_control_gui.py: PASS
git diff --check: PASS
check_runtime_freshness.py --fetch --refresh-version --warn-only: WARN current-dirty, dirty_paths=695, active_runtime_dirty_paths=677
refactor_inventory.py: bridge/ros2_sensor_messages.py removed from top 45
```
