# ROS2 Bridge Spin/Publish Split

Date: 2026-06-07

## Scope

Split `bridge/ros2_bridge_spin_publish.py` into focused public-loop helpers
without changing `Ros2Bridge.spin_once()` or `Ros2Bridge.publish(...)`
behavior.

## Files

- `bridge/ros2_bridge_spin_publish.py`
  - Compatibility export surface.
- `bridge/ros2_bridge_sitl_poll.py`
  - Shared SITL servo polling.
- `bridge/ros2_bridge_cmd_timeout.py`
  - Active command timeout clearing.
- `bridge/ros2_bridge_spin_executor.py`
  - ROS executor spin cadence, callback execution, and spin-error handling.
- `bridge/ros2_bridge_spin_once.py`
  - Public `spin_once()` policy.
- `bridge/ros2_bridge_publish_timing.py`
  - Sensor publish rate gate.
- `bridge/ros2_bridge_publish_stamp.py`
  - ROS timestamp acquisition and failure handling.
- `bridge/ros2_bridge_publish_ros.py`
  - Static context and queued ROS publication for a prepared snapshot.
- `bridge/ros2_bridge_publish.py`
  - Public `publish(...)` policy.

## Contract Notes

- `spin_once()` still polls SITL servo before ROS callback work.
- Command timeout clearing still runs when ROS is disabled, ROS is unhealthy,
  the executor thread owns spinning, or inline spin completes.
- ROS callback exceptions still disable bridge callbacks by setting
  `_ros_ok=False` and reporting once.
- `publish(...)` still rate-gates by `last_pub_t` and `sensor_dt` before
  sending a SITL sensor snapshot.
- Timestamp acquisition failures still set `_ros_ok=False` and skip ROS publish
  jobs.

## Verification

```text
ros2 bridge spin/publish split smoke: PASS
python3 -m compileall -q sim/current uuv_control_gui.py: PASS
git diff --check: PASS
check_runtime_freshness.py --warn-only: WARN current-dirty, dirty_paths=665, active_runtime_dirty_paths=647
refactor_inventory.py: bridge/ros2_bridge_spin_publish.py removed from top 45
```
