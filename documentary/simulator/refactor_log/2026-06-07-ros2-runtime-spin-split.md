# ROS2 Runtime Spin Split

Date: 2026-06-07

## Scope

Split `bridge/ros2_runtime_spin.py` into focused ROS context, safe-publish,
spin-state, spin-loop, and thread-start helpers without changing the
`Ros2Bridge` method binding names.

## Files

- `bridge/ros2_runtime_spin.py`
  - Compatibility export surface.
- `bridge/ros2_runtime_context_errors.py`
  - ROS context shutdown error classification.
- `bridge/ros2_runtime_safe_publish.py`
  - Safe publisher wrapper and publish-failure state update.
- `bridge/ros2_runtime_spin_state.py`
  - Spin context predicate, cached ExternalNav send, and spin-error marker.
- `bridge/ros2_runtime_spin_loop.py`
  - Dedicated executor spin-loop policy.
- `bridge/ros2_runtime_spin_thread.py`
  - Dedicated executor thread startup.

## Contract Notes

- Publish failures still set `_ros_ok=False` and report once.
- ROS context shutdown exceptions still stop the spin loop without marking a
  bridge error.
- Non-shutdown spin exceptions still set `_ros_ok=False` and report once.
- Cached ExternalNav output is still sent after each successful executor spin.
- Thread startup still no-ops when no executor exists or a spin thread already
  exists.

## Verification

```text
ros2 runtime spin split smoke: PASS
python3 -m compileall -q sim/current uuv_control_gui.py: PASS
git diff --check: PASS
check_runtime_freshness.py --warn-only: WARN current-dirty, dirty_paths=670, active_runtime_dirty_paths=652
refactor_inventory.py: bridge/ros2_runtime_spin.py removed from top 45
```
