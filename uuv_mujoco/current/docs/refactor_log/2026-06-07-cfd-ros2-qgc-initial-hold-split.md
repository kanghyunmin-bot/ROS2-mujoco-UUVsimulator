# CFD, ROS2 Runtime, QGC Video, And Initial Hold Split

Date: 2026-06-07

Scope: active runtime through `uuv_mujoco/current`, backed by
`uuv_mujoco/v2.2`.

## Why

The active runtime still had several branch-heavy files mixing unrelated
responsibilities:

- CFD dynamic wrench mixed runtime type, table interpolation, profile/env
  parsing, and force evaluation.
- ROS2 bridge runtime mixed optional message probing, subscriber-demand
  caching, publish queueing, and static context publishing.
- QGC video FFmpeg helpers mixed encoder discovery, command construction, and
  process lifecycle.
- Initial depth hold mixed pose/depth holding with release velocity application.

These are all runtime-adjacent contract surfaces.  The refactor is intentionally
behavior-neutral: no ArduPilot changes, no controller-parity shim, no actuator
remap, and no hydrodynamic coefficient tuning.

## Changes

- Added `sim/physics/cfd_dynamic_wrench_types.py`.
- Added `sim/physics/cfd_dynamic_wrench_table.py`.
- Added `sim/physics/cfd_dynamic_wrench_force.py`.
- Added `sim/physics/cfd_dynamic_wrench_profile.py`.
- Reduced `sim/physics/cfd_dynamic_wrench.py` to a compatibility facade.
- Added `bridge/ros2_optional_message.py`.
- Added `bridge/ros2_publisher_demand.py`.
- Added `bridge/ros2_publish_queue.py`.
- Added `bridge/ros2_static_context_publisher.py`.
- Reduced `bridge/ros2_bridge_runtime.py` to a compatibility facade.
- Added `bridge/qgc_video_ffmpeg_probe.py`.
- Added `bridge/qgc_video_ffmpeg_cmd.py`.
- Added `bridge/qgc_video_ffmpeg_process.py`.
- Reduced `bridge/qgc_video_ffmpeg.py` to a compatibility facade.
- Added `sim/runtime/initial_hold_pose.py`.
- Added `sim/runtime/initial_hold_release.py`.
- Kept `sim/runtime/initial_hold.py` as the public `InitialDepthHoldState`
  surface.

## Inventory Effect

Removed from the top 40 hotspot inventory:

```text
sim/physics/cfd_dynamic_wrench.py   157 LOC / 20 branches
bridge/ros2_bridge_runtime.py       156 LOC / 24 branches
bridge/qgc_video_ffmpeg.py          160 LOC / 13 branches
sim/runtime/initial_hold.py         161 LOC / 15 branches
```

Current high-branch targets after this pass:

```text
gui/physics_param_io.py                 152 LOC / 25 branches
physics/sim_profile_hydrostatic_points.py 146 LOC / 32 branches
sim/physics/thruster_performance.py     146 LOC / 16 branches
bridge/ping360_settings.py              146 LOC / 15 branches
bridge/ros2_publish_schedule.py         152 LOC / 15 branches
sim/physics/actuator_geometry.py        152 LOC / 16 branches
```

## Verification

Targeted smokes:

```text
cfd_dynamic_wrench table/force/runtime smoke: PASS
ros2_bridge_runtime demand/queue/static-context smoke: PASS
qgc_video_ffmpeg command-shape smoke: PASS
initial_hold pose/release smoke: PASS
```

Full gates:

```text
python3 -m compileall -q uuv_mujoco/current uuv_control_gui.py
PYTHONPATH=uuv_mujoco/current/tools python3 uuv_mujoco/current/tools/audit_code_contract_sources.py --out-dir /private/tmp/uuv_source_audit_spaghetti_20260607_cfd_bridge_video_hold_split
PYTHONPATH=uuv_mujoco/current/tools python3 uuv_mujoco/current/tools/check_runtime_freshness.py --workspace /Users/kanghyunmin/Desktop/uuv_sim --runtime-dir /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current --fetch --refresh-version
python3 uuv_mujoco/current/tools/check_runtime_readiness_policy.py
python3 uuv_mujoco/current/tools/verify_ardusub_thruster_contract.py --quiet
python3 uuv_mujoco/current/tools/check_dev_os_compat.py --headless --target-os ubuntu
/Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python uuv_mujoco/current/tools/physics_contract_audit.py --output-dir /private/tmp/uuv_physics_audit_spaghetti_20260607_cfd_bridge_video_hold_split --simulate-s 0
git diff --check
```

Results:

- Compile: `PASS`.
- Source contract audit: `fail=0`, `pass=11`, `warn=5`.
- Runtime freshness: `PASS`, `uuv_mujoco/current -> v2.2`.
- Runtime readiness: `PASS`.
- ArduSub thruster contract: `OK`.
- Dev OS compatibility: `fail=0`, `pass=16`, `warn=2`.
- Static physics balance: `net_down=+0.000N`, `required_scale=1.000000`.
- Diff whitespace check: clean.

Expected environment warnings:

- Docker daemon was not running in this shell.
- ROS2 was not sourced in this shell; launchers may source their configured
  environment.
