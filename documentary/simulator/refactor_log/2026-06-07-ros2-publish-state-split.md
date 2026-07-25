# ROS2 Publish State Split

Date: 2026-06-07

## Change

- Added `bridge/ros2_publish_state.py`.
- Moved ROS-frame derived state preparation, DVL odometry integration,
  static-pressure source selection, and MAVROS setpoint update out of
  `bridge/ros2_publish_runtime.py`.
- Updated `tools/audit_code_contract_sources.py` so the
  `/mavros/imu/static_pressure` Bar30 evidence follows the new source path.

## Metrics

Before:

- `bridge/ros2_publish_runtime.py`: `545 LOC`, `53` branch nodes.
- `flush_ros_publish_jobs()`: `510 LOC`.

After:

- `bridge/ros2_publish_runtime.py`: `521 LOC`, `51` branch nodes.
- `flush_ros_publish_jobs()`: `485 LOC`.

This is a small but contract-safe split.  The next larger step is to move the
lazy message-builder cache out of `flush_ros_publish_jobs()`.

## Validation

```bash
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 -m compileall -q uuv_mujoco/v2.2
env PYTHONPYCACHEPREFIX=/private/tmp/pycache PYTHONPATH=uuv_mujoco/v2.2 /Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python3.11 -c "from bridge.ros2_bridge import Ros2Bridge; from bridge import ros2_publish_runtime, ros2_publish_state; print(Ros2Bridge.__name__, callable(ros2_publish_runtime.flush_ros_publish_jobs), callable(ros2_publish_state.prepare_ros_publish_state))"
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 uuv_mujoco/v2.2/tools/audit_code_contract_sources.py --out-dir /private/tmp/uuv_code_contract_after_ros2_publish_state_split_v2
git diff --check -- uuv_mujoco/v2.2/tools/audit_code_contract_sources.py uuv_mujoco/v2.2/bridge/ros2_publish_state.py uuv_mujoco/v2.2/bridge/ros2_publish_runtime.py
```

Results:

- compileall: pass
- import smoke: `Ros2Bridge True True`
- contract source audit: `fail=0`, `pass=10`, `warn=5`
- diff check: pass
