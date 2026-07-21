# Ping360 and GUI ROS Process Split

Date: 2026-06-07

## Scope

Behavior-neutral split inside the active runtime `uuv_mujoco/current`.  This
does not change Ping360 settings, ROS topic names, MAVROS process commands,
simulator command contracts, ArduPilot source, or plant input semantics.

## Ping360

Changed:

- Added `bridge/ping360_beam_model.py` for local beam directions, angle
  conversion, MuJoCo raycast, and geom reflectivity lookup.
- Added `bridge/ping360_profile.py` for return strength, profile accumulation,
  blind-zone handling, and noise synthesis.
- Kept `bridge/ping360_sim.py` focused on scan timing, rolling profile/image
  buffers, range/intensity buffers, and sample publication state.

Effect:

- `bridge/ping360_sim.py`: `286 LOC / 38` branches -> `138 LOC / 12` branches.

## GUI ROS Process Controls

Changed:

- Added `gui/ros_panel_mixin.py` for ROS2 panel visibility, running-state
  checks, status variables, and button refresh.
- Added `gui/ros_logged_process_mixin.py` for logged process launch and
  background log watching.
- Added `gui/ros_package_mixin.py` for ROS package build and external MAVROS
  launch/stop controls.
- Added `gui/rviz_process_mixin.py` for RViz launch/stop controls.
- Reduced `gui/ros_process_mixin.py` to the compatibility composition surface.

Effect:

- `gui/ros_process_mixin.py`: `281 LOC / 45` branches -> `16 LOC / 0` branches.

## Validation

```text
python3 -m py_compile \
  uuv_mujoco/current/bridge/ping360_sim.py \
  uuv_mujoco/current/bridge/ping360_profile.py \
  uuv_mujoco/current/bridge/ping360_beam_model.py \
  uuv_mujoco/current/gui/ros_process_mixin.py \
  uuv_mujoco/current/gui/ros_panel_mixin.py \
  uuv_mujoco/current/gui/ros_logged_process_mixin.py \
  uuv_mujoco/current/gui/ros_package_mixin.py \
  uuv_mujoco/current/gui/rviz_process_mixin.py

PYTHONPATH="uuv_mujoco/current:${PYTHONPATH:-}" \
  /Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python - <<'PY'
from gui.ros_process_mixin import RosProcessMixin
required = [
    "_toggle_ros2_panel", "_ros_pkg_running", "_ros_build_running",
    "_rviz_running", "_gui_external_mavros_controls_enabled",
    "_set_ros_pkg_status", "_set_rviz_status", "_refresh_ros2_buttons",
    "_start_logged_ros_process", "_watch_ros_process", "_build_ros_pkg",
    "_toggle_ros_pkg_stack", "_start_ros_pkg_stack", "_toggle_rviz",
    "_start_rviz",
]
print({name: callable(getattr(RosProcessMixin, name, None)) for name in required})
PY

python3 -m compileall -q uuv_mujoco/current uuv_control_gui.py
PYTHONPATH=uuv_mujoco/current/tools python3 \
  uuv_mujoco/current/tools/audit_code_contract_sources.py \
  --out-dir /private/tmp/uuv_code_contract_after_ping360_gui_process_split
python3 uuv_mujoco/current/tools/check_runtime_readiness_policy.py
python3 uuv_mujoco/current/tools/verify_ardusub_thruster_contract.py --quiet
python3 uuv_mujoco/current/tools/check_dev_os_compat.py --headless --target-os ubuntu
python3 uuv_mujoco/current/tools/refactor_inventory.py --limit 20
git diff --check
```

Observed:

- Compile and GUI import-surface checks passed.
- Source-contract audit: `fail=0`, `pass=11`, `warn=5`.
- Runtime readiness policy: `PASS`.
- Thruster contract: `OK`.
- Development OS compatibility: `fail=0`, `pass=16`, `warn=2`.
- `git diff --check`: passed.
