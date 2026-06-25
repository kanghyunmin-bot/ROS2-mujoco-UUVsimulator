# Runtime Loader, Replay Loader, QGC, AutoTune, and MAVROS Builder Split

Date: 2026-06-07

## Scope

Continue reducing active-runtime coupling under `uuv_mujoco/current` while
preserving the compatibility backing directory `uuv_mujoco/v2.2`.

This pass is behavior-neutral. It does not change ArduPilot, controller-parity
observation points, RC remapping, PWM calibration, JSON servo plant input, or
hydrodynamic coefficients.

## Current Runtime Freshness

The live runtime is:

```text
uuv_mujoco/current -> v2.2
```

`v2.2` is still the compatibility backing directory name. Current launch and
validation paths must use `uuv_mujoco/current` plus the freshness preflight, not
the backing directory name as a freshness signal.

Observed freshness gate:

```text
[uuv_mujoco] runtime freshness: PASS
[uuv_mujoco] uuv_mujoco/current -> v2.2
[uuv_mujoco] branch=uuv_sim
[uuv_mujoco] HEAD=e60cd9383c214a7d6e4700cdeb5c2f2a8cf6d328
[uuv_mujoco] origin/uuv_sim=e60cd9383c214a7d6e4700cdeb5c2f2a8cf6d328
```

## Changed Files

- `gui/runtime_python_path.py`: Python import-path sanitizing and ROS2 site-path
  helpers.
- `gui/runtime_ros_core.py`: ROS2 core imports and fallbacks.
- `gui/runtime_mavros.py`: MAVROS message/service imports and fallbacks.
- `gui/runtime.py`: compatibility facade for existing GUI imports.
- `bridge/sitl_replay_row_parsers.py`: row-to-record parsing for sensor frames
  and native VPD replay events.
- `bridge/sitl_replay_loaders.py`: CSV file IO, sorting, and logging facade.
- `bridge/qgc_video_ffmpeg.py`: ffmpeg availability, command construction, and
  subprocess start helpers.
- `bridge/qgc_video_stream.py`: QGC streamer state and public API facade.
- `sim/physics/dynamic_fluidcoef_runtime_knobs.py`: runtime update/transient
  knobs.
- `sim/physics/dynamic_fluidcoef_runtime_state.py`: runtime state arrays and
  buffers.
- `sim/physics/dynamic_fluidcoef_runtime_config.py`: top-level runtime config
  facade.
- `gui/autotune_launch.py`: AutoTune launch request and command construction.
- `gui/autotune_subprocess.py`: AutoTune subprocess start/open helpers.
- `gui/autotune_process_runtime.py`: AutoTune process lifecycle facade.
- `bridge/ros2_publish_mavros_cache.py`: lazy MAVROS-compatible message builder
  cache.
- `bridge/ros2_publish_builder_mavros.py`: MAVROS builder facade.

## Contract Notes

- MAVROS-compatible publish builder keys remain unchanged:
  `mavros_state`, `mavros_vfr_hud`, `mavros_imu`, `mavros_imu_raw`,
  `mavros_static_pressure`, `mavros_atm_pressure`, `mavros_battery`,
  `mavros_local_pose`, `mavros_local_vel`, `mavros_local_vel_body`,
  `mavros_local_vel_body_cov`, `mavros_local_odom`, and
  `mavros_vision_pose`.
- MAVROS frame IDs remain `fcu_link`, `map`, and `base_link` as before.
- Static pressure uses the same pressure builder and `state.static_pressure_pa`
  source.
- SITL replay row parsing keeps the same pressure/depth conversion contract:
  `pressure = surface_pressure + rho * g * depth`.
- Native VPD replay timing still converts real timestamps to replay time using
  `t_replay_s = t_s - real_start_s`.
- QGC video RTP output still ends at `rtp://<host>:<port>?pkt_size=<pkt_size>`.
- Dynamic fluid coefficient runtime equations and smoothing are unchanged; only
  setup ownership moved.
- AutoTune arguments and default behavior are unchanged.

## Validation

```text
/Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python -m compileall -q \
  uuv_mujoco/current/bridge/ros2_publish_builder_mavros.py \
  uuv_mujoco/current/bridge/ros2_publish_mavros_cache.py \
  uuv_mujoco/current/bridge/ros2_publish_builders.py

python3 -m compileall -q uuv_mujoco/current uuv_control_gui.py

PYTHONPATH=uuv_mujoco/current/tools python3 \
  uuv_mujoco/current/tools/audit_code_contract_sources.py \
  --out-dir /private/tmp/uuv_source_audit_runtime_loader_qgc_autotune_mavros_split

python3 uuv_mujoco/current/tools/check_runtime_readiness_policy.py
python3 uuv_mujoco/current/tools/verify_ardusub_thruster_contract.py --quiet
python3 uuv_mujoco/current/tools/check_runtime_freshness.py \
  --workspace /Users/kanghyunmin/Desktop/uuv_sim \
  --runtime-dir /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current \
  --fetch --refresh-version

/Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python \
  uuv_mujoco/current/tools/physics_contract_audit.py \
  --output-dir /private/tmp/uuv_physics_audit_runtime_loader_qgc_autotune_mavros_split \
  --simulate-s 0

python3 uuv_mujoco/current/tools/check_dev_os_compat.py --headless --target-os ubuntu
python3 uuv_mujoco/current/tools/refactor_inventory.py --limit 30
```

Observed status:

```text
compileall: PASS
source audit: fail=0 pass=11 warn=5
runtime_readiness_policy=PASS
thruster-contract: OK
runtime freshness: PASS
physics static balance: net_down=+0.000 N at scene and auto wet depths
dev-os compatibility: fail=0 pass=16 warn=2
```

The remaining dev-os warnings are external environment state:

```text
docker_daemon: Docker daemon socket unavailable
ros2_env: ROS2 not sourced in current shell
```

## Current Hotspot Snapshot

The files split in this pass are no longer the old active hotspots, but two new
small ownership targets remain visible:

- `bridge/ros2_publish_mavros_cache.py`: `188 LOC / 1` branch; mostly repeated
  lazy builder methods and safe to leave unless the inventory threshold is
  tightened further.
- `bridge/qgc_video_ffmpeg.py`: `160 LOC / 13` branches; command construction
  can be split again if QGC video becomes a runtime failure point.

Top current hotspots by the inventory are:

```text
tools/roll_stability_runner.py          196 LOC / 19 branches
bridge/ros2_publish_mavros_cache.py    188 LOC / 1 branch
tools/dev_os_compat_system.py          176 LOC / 23 branches
physics/sim_profile_defaults.py        174 LOC / 0 branches
bridge/ros2_state_vertical.py          173 LOC / 22 branches
```
