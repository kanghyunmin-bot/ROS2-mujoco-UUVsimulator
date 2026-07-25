# ROS2 Math Helper Split

Date: 2026-06-07

## Scope

Split `bridge/ros2_math.py` into focused RC, scalar, quaternion, rotation,
pressure, and loose-message helper modules while preserving the historical
`bridge.ros2_math` import surface.

## Files

- `bridge/ros2_math.py`
  - Compatibility export surface.
- `bridge/ros2_math_rc.py`
  - RC channel clamp, channel lookup, and normalized PWM conversion.
- `bridge/ros2_math_scalar.py`
  - Finite-value and wrapped-angle helpers.
- `bridge/ros2_math_quat.py`
  - Quaternion/rotation-matrix conversion and yaw extraction.
- `bridge/ros2_math_rotation.py`
  - RPY degrees to rotation matrix conversion.
- `bridge/ros2_math_pressure.py`
  - Absolute pressure from positive-down depth conversion.
- `bridge/ros2_message_setters.py`
  - Optional ROS message field setters.

## Contract Notes

- No RC override mapping, PWM span, Bar30 pressure equation, IMU quaternion
  equation, frame transform, or ROS message field name was changed.
- Existing imports from `bridge.ros2_math` and `.ros2_math` remain valid.
- The split is intentionally behavior-neutral so future contract work can audit
  RC math, pressure math, and quaternion math independently.

## Verification

```text
ros2_math facade smoke: PASS
python3 -m compileall -q sim/current uuv_control_gui.py: PASS
git diff --check: PASS
check_runtime_freshness.py --fetch --refresh-version --warn-only: WARN current-dirty, dirty_paths=690, active_runtime_dirty_paths=672
refactor_inventory.py: bridge/ros2_math.py removed from top 45
```
