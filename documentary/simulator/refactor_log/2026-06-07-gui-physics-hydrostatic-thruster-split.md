# GUI Physics, Hydrostatic Parser, And Thruster Performance Split

Date: 2026-06-07

Scope: active runtime through `sim/current`, backed by
`uuv_mujoco/v2.2`.

## Why

The next branch-heavy runtime-adjacent files mixed contract parsing with UI or
runner behavior:

- GUI physics parameter loading mixed current-mode status, nested JSON access,
  user input parsing, profile writing, backup creation, and error logging.
- Hydrostatic profile parsing mixed normalization, distributed buoyancy points,
  and body-component mass/buoyancy parsing.
- Thruster performance loading mixed default config, JSON loading, curve row
  cleanup, nearest-voltage selection, and plant-replay direct-mode enforcement.

These surfaces are directly relevant to future plant tuning and sim-to-real
work, so the refactor keeps behavior stable while making the contract path more
inspectable.

## Changes

- Added `gui/physics_param_status.py`.
- Added `gui/physics_param_format.py`.
- Added `gui/physics_param_parse.py`.
- Added `gui/physics_param_apply.py`.
- Reduced `gui/physics_param_io.py` to a compatibility facade.
- Added `physics/sim_profile_hydrostatic_normalize.py`.
- Added `physics/sim_profile_hydrostatic_buoyancy_points.py`.
- Added `physics/sim_profile_hydrostatic_body_components.py`.
- Reduced `physics/sim_profile_hydrostatic_points.py` to a compatibility
  facade.
- Added `sim/physics/thruster_performance_config.py`.
- Added `sim/physics/thruster_performance_loader.py`.
- Added `sim/physics/thruster_performance_selector.py`.
- Reduced `sim/physics/thruster_performance.py` to a compatibility facade.

## Inventory Effect

Removed from the top 35 hotspot inventory:

```text
gui/physics_param_io.py                    152 LOC / 25 branches
physics/sim_profile_hydrostatic_points.py  146 LOC / 32 branches
sim/physics/thruster_performance.py        146 LOC / 16 branches
```

Current next branch-heavy targets after this pass:

```text
sim/physics/actuator_geometry.py       152 LOC / 16 branches
bridge/ros2_publish_schedule.py        152 LOC / 15 branches
bridge/ping360_settings.py             146 LOC / 15 branches
tools/dev_os_compat_common.py          142 LOC / 20 branches
bridge/sitl_sensor_replay_frame_policy.py 143 LOC / 11 branches
```

## Verification

Targeted smokes:

```text
physics_param_io helper surface smoke: PASS
hydrostatic point/body component parser smoke: PASS
thruster performance nearest-voltage/direct-mode smoke: PASS
```

Full gates:

```text
python3 -m compileall -q sim/current uuv_control_gui.py
PYTHONPATH=sim/current/tools python3 sim/current/tools/audit_code_contract_sources.py --out-dir /private/tmp/uuv_source_audit_spaghetti_20260607_gui_hydro_thruster_split
PYTHONPATH=sim/current/tools python3 sim/current/tools/check_runtime_freshness.py --workspace /Users/kanghyunmin/Desktop/uuv_sim --runtime-dir /Users/kanghyunmin/Desktop/uuv_sim/sim/current --fetch --refresh-version
python3 sim/current/tools/check_runtime_readiness_policy.py
python3 sim/current/tools/verify_ardusub_thruster_contract.py --quiet
python3 sim/current/tools/check_dev_os_compat.py --headless --target-os ubuntu
/Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python sim/current/tools/physics_contract_audit.py --output-dir /private/tmp/uuv_physics_audit_spaghetti_20260607_gui_hydro_thruster_split --simulate-s 0
```

Results:

- Compile: `PASS`.
- Source contract audit: `fail=0`, `pass=11`, `warn=5`.
- Runtime freshness: `PASS`, `sim/current -> v2.2`.
- Runtime readiness: `PASS`.
- ArduSub thruster contract: `OK`.
- Dev OS compatibility: `fail=0`, `pass=16`, `warn=2`.
- Static physics balance: `net_down=+0.000N`, `required_scale=1.000000`.

Expected environment warnings:

- Docker daemon was not running in this shell.
- ROS2 was not sourced in this shell; launchers may source their configured
  environment.
