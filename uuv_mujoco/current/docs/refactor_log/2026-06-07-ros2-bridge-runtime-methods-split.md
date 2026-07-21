# ROS2 bridge runtime methods split

Date: 2026-06-07

## Scope

- Added focused ROS2 runtime helper modules:
  - `bridge/ros2_runtime_env.py`
  - `bridge/ros2_runtime_spin.py`
  - `bridge/ros2_runtime_mavros_state.py`
  - `bridge/ros2_runtime_static_context.py`
- Kept `bridge/ros2_bridge_runtime_methods.py` as the compatibility export
  surface consumed by `bridge/ros2_bridge_method_bindings.py`.

## Contract

The split preserves ROS rate/env helpers, safe publish, executor spin,
MAVROS state construction, sensor slicing, robot description loading, and
static context publication method names.

## Verification

```text
/Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python - <<'PY'
import sys
from pathlib import Path
sys.path.insert(0, str(Path('uuv_mujoco/current').resolve()))
from bridge import ros2_bridge_runtime_methods
required = ['env_to_rate_hz','env_to_clamped_float','is_ros_context_shutdown_error','safe_publish','start_ros_spin_thread','ros_spin_loop','ros_topic_due','ros_imu_accel_surface','sensor_slice_method','build_mavros_state','load_robot_description_text','publish_static_context']
missing = [name for name in required if not hasattr(ros2_bridge_runtime_methods, name)]
print({'missing': missing})
PY
```

Smoke result: `missing=[]`.  The runtime Python is used because this import
surface depends on MuJoCo through `bridge/ros2_mujoco_model.py`.
