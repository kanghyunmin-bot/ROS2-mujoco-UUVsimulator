# ROS2 Publish Builder Split

Date: 2026-06-07

## Change

- Split lazy ROS message construction out of `bridge/ros2_publish_runtime.py`
  into output-surface builder modules:
  - `bridge/ros2_publish_builder_core.py`
  - `bridge/ros2_publish_builder_status.py`
  - `bridge/ros2_publish_builder_dvl.py`
  - `bridge/ros2_publish_builder_mavros.py`
  - `bridge/ros2_publish_builder_odometry.py`
  - `bridge/ros2_publish_builder_ping360.py`
- Kept `bridge/ros2_publish_builders.py` as a small aggregator.
- `bridge/ros2_publish_runtime.py` now owns only state preparation, rate-limited
  scheduling, and publish-queue flushing.

## Metrics

Before this split:

- `bridge/ros2_publish_runtime.py`: `521 LOC`, `51` branch nodes.
- `flush_ros_publish_jobs()`: `485 LOC`.
- `bridge/ros2_publish_builders.py`: `523 LOC`, largest function
  `_build_mavros_builders()` at `156 LOC`.

After this split:

- `bridge/ros2_publish_runtime.py`: `42 LOC`, `2` branch nodes.
- `flush_ros_publish_jobs()`: `31 LOC`.
- `bridge/ros2_publish_builders.py`: `22 LOC`.
- Largest publish-builder surface:
  `bridge/ros2_publish_builder_mavros.py`, `174 LOC`, `13` branch nodes,
  largest function `156 LOC`.

## Validation

```bash
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 -m compileall -q uuv_mujoco/v2.2
env PYTHONPYCACHEPREFIX=/private/tmp/pycache PYTHONPATH=uuv_mujoco/v2.2 /Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python3.11 -c "from bridge.ros2_bridge import Ros2Bridge; from bridge.ros2_publish_builders import build_ros_publish_builders; print(Ros2Bridge.__name__, callable(build_ros_publish_builders))"
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 uuv_mujoco/v2.2/tools/check_runtime_readiness_policy.py
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 uuv_mujoco/v2.2/tools/verify_ardusub_thruster_contract.py --quiet
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 uuv_mujoco/v2.2/tools/audit_code_contract_sources.py --out-dir /private/tmp/uuv_code_contract_after_ros2_publish_builder_split
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 uuv_mujoco/v2.2/tools/check_dev_os_compat.py --headless --target-os ubuntu
git diff --check -- uuv_mujoco/v2.2/bridge/ros2_publish_runtime.py uuv_mujoco/v2.2/bridge/ros2_publish_builders.py uuv_mujoco/v2.2/bridge/ros2_publish_builder_core.py uuv_mujoco/v2.2/bridge/ros2_publish_builder_status.py uuv_mujoco/v2.2/bridge/ros2_publish_builder_dvl.py uuv_mujoco/v2.2/bridge/ros2_publish_builder_mavros.py uuv_mujoco/v2.2/bridge/ros2_publish_builder_odometry.py uuv_mujoco/v2.2/bridge/ros2_publish_builder_ping360.py
```

Results:

- compileall: pass
- import smoke: `Ros2Bridge True`
- readiness policy: pass
- thruster contract: pass
- contract source audit: `fail=0`, `pass=10`, `warn=5`
- dev OS compatibility: `fail=0`, `pass=16`, `warn=2`
- diff check: pass
