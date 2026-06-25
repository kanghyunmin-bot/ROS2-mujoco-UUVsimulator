# Sensor Replay And ROS2 Init Split

Date: 2026-06-07

Scope: active runtime bridge code under `uuv_mujoco/current/bridge`.

## What changed

- Split live SITL sensor snapshot generation out of
  `bridge/ros2_sitl_sensor_feed.py` into:
  - `bridge/ros2_sitl_sensor_types.py`
  - `bridge/ros2_sitl_sensor_kinematics.py`
  - `bridge/ros2_sitl_sensor_vectors.py`
  - `bridge/ros2_sitl_sensor_vertical.py`
  - `bridge/ros2_sitl_sensor_transport.py`
- Split controller-parity sensor replay runtime out of
  `bridge/sitl_sensor_replay_runtime.py` into:
  - `bridge/sitl_sensor_replay_clock.py`
  - `bridge/sitl_sensor_replay_frame_policy.py`
  - `bridge/sitl_sensor_replay_state.py`
  - `bridge/sitl_sensor_replay_status_runtime.py`
- Split ROS2 bridge initialization out of `bridge/ros2_bridge_init.py` into:
  - `bridge/ros2_bridge_imports.py`
  - `bridge/ros2_bridge_message_bindings.py`
  - `bridge/ros2_bridge_context_runtime.py`
  - `bridge/ros2_bridge_static_context.py`
  - `bridge/ros2_bridge_startup_log.py`

## Contract boundaries preserved

- No topic names, message fields, Bar30 pressure conversion, ENU/NED/FRD/BMJ
  transforms, or replay timing equations were intentionally changed.
- The public bridge method bindings still call
  `_build_and_send_sitl_sensor_snapshot`, `_sensor_replay_frame_at`, and
  `_sensor_replay_payload_timestamp_for_sim_t` through the same compatibility
  surfaces.
- ROS2 imports remain lazy; importing the modules without a sourced ROS2 shell
  still succeeds.
- ArduPilot source and submodule pointer were not touched.

## Verification

```text
python3 -m compileall -q uuv_mujoco/current uuv_control_gui.py
PYTHONPATH=uuv_mujoco/current/tools python3 \
  uuv_mujoco/current/tools/audit_code_contract_sources.py \
  --out-dir /private/tmp/uuv_source_audit_sensor_replay_init_split
PYTHONPATH=uuv_mujoco/current /Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python \
  - <<'PY'
from bridge.ros2_sitl_sensor_feed import build_and_send_sitl_sensor_snapshot
from bridge.sitl_sensor_replay_runtime import _sensor_replay_frame_at
from bridge.ros2_bridge_init import configure_ros_runtime_state, init_ros_runtime
print(callable(build_and_send_sitl_sensor_snapshot), callable(_sensor_replay_frame_at), callable(init_ros_runtime))
PY
python3 uuv_mujoco/current/tools/check_runtime_readiness_policy.py
python3 uuv_mujoco/current/tools/verify_ardusub_thruster_contract.py --quiet
python3 uuv_mujoco/current/tools/check_runtime_freshness.py \
  --workspace /Users/kanghyunmin/Desktop/uuv_sim \
  --runtime-dir /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current \
  --fetch --refresh-version
/Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python \
  uuv_mujoco/current/tools/physics_contract_audit.py \
  --output-dir /private/tmp/uuv_physics_audit_sensor_replay_init_split \
  --simulate-s 0
python3 uuv_mujoco/current/tools/check_dev_os_compat.py --headless --target-os ubuntu
git diff --check
python3 uuv_mujoco/current/tools/refactor_inventory.py --limit 35
```

Results:

- Compile/import: pass.
- Source contract audit: `fail=0`, `pass=11`, `warn=5`.
- Runtime readiness policy: pass.
- ArduSub thruster contract: pass.
- Runtime freshness: pass, `uuv_mujoco/current -> v2.2`.
- Static physics audit: neutral force balance still
  `net_down=+0.000N`, `required_scale=1.000000`.
- Ubuntu compatibility: `fail=0`, `pass=16`, `warn=2`
  (`docker_daemon`, `ros2_env`).
- `git diff --check`: pass.
- Removed from top 35 hotspot inventory:
  `bridge/ros2_sitl_sensor_feed.py`,
  `bridge/sitl_sensor_replay_runtime.py`, and
  `bridge/ros2_bridge_init.py`.
