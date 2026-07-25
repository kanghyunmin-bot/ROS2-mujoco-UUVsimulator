# 2026-06-07 MAVROS/Hydrodynamics/Schedule/Actuator Split

Scope: active runtime under `sim/current`.  The active alias still
resolves to the compatibility backing directory `uuv_mujoco/v2.2`; freshness is
verified through the alias, branch/HEAD evidence, and contract gates rather than
by treating the old directory name as a current-version claim.

## Refactor changes

- Split `bridge/ros2_publish_mavros_cache.py` into IMU, status, and local
  message-cache builders.  The public `MavrosPublishBuilderCache.builders()`
  surface and topic keys are preserved.
- Removed import-time coupling from the MAVROS cache facade by keeping
  `RosPublishState` behind type-checking.  Importing the cache no longer pulls
  MuJoCo through `bridge/ros2_publish_state.py`.
- Split `physics/sim_profile_hydrodynamics.py` into damping/added-mass parsing
  and hydrostatic/body-component assembly helpers while preserving the
  `HydrodynamicsConfig` output contract.
- Split `bridge/ros2_publish_schedule.py` into core, Ping360, DVL, MAVROS, and
  odometry schedule helpers.  The `/mavros/local_position/odom` publish order is
  preserved after `/dvl/odometry`.
- Split `sim/physics/actuator_geometry.py` into site lookup, horizontal offset,
  vertical offset, and propeller-joint helpers.  The original import surface
  remains a compatibility facade.

## Inventory impact

The following former hotspots no longer appear in the top-35 active-runtime
inventory:

- `bridge/ros2_publish_mavros_cache.py`: old `188 LOC / 1` branch.
- `physics/sim_profile_hydrodynamics.py`: old `172 LOC / 3` branches.
- `bridge/ros2_publish_schedule.py`: old `152 LOC / 15` branches.
- `sim/physics/actuator_geometry.py`: old `152 LOC / 16` branches.
- `sim/physics/actuator_geometry_offsets.py`: briefly introduced during the
  split, then split again so offset ownership is not a new hotspot.

Current top hotspots after the pass start with
`physics/sim_profile_defaults.py`, `tools/actuator_wrench_calc.py`, and
`bridge/ros2_bridge.py`.  Those are the next structural targets; this pass did
not touch controller parity, ArduPilot, submodule pointers, PWM remaps, or
plant-input semantics.

## Validation

Commands run from `/Users/kanghyunmin/Desktop/uuv_sim`:

```bash
python3 -m compileall -q sim/current uuv_control_gui.py
PYTHONPATH=sim/current/tools python3 sim/current/tools/audit_code_contract_sources.py --out-dir /private/tmp/uuv_source_audit_spaghetti_20260607_mavros_hydro_schedule_actuator_split
PYTHONPATH=sim/current/tools python3 sim/current/tools/check_runtime_freshness.py --workspace /Users/kanghyunmin/Desktop/uuv_sim --runtime-dir /Users/kanghyunmin/Desktop/uuv_sim/sim/current --fetch --refresh-version
python3 sim/current/tools/check_runtime_readiness_policy.py
python3 sim/current/tools/verify_ardusub_thruster_contract.py --quiet
python3 sim/current/tools/check_dev_os_compat.py --headless --target-os ubuntu
/Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python sim/current/tools/physics_contract_audit.py --output-dir /private/tmp/uuv_physics_audit_spaghetti_20260607_mavros_hydro_schedule_actuator_split --simulate-s 0
```

Results:

- Compile: pass.
- Source contract audit: `fail=0`, `pass=11`, `warn=5`.
- Runtime freshness: pass, `sim/current -> v2.2`,
  `HEAD == origin/uuv_sim == e60cd9383c214a7d6e4700cdeb5c2f2a8cf6d328`.
- Runtime readiness policy: pass.
- ArduSub thruster contract: pass.
- Development OS compatibility: `fail=0`, `pass=16`, `warn=2`
  (`docker_daemon`, `ros2_env` only).
- Physics static contract audit: pass, mass `15.000 kg`, neutral
  `buoyancy_scale=1.000000`, auto Bar30 start-depth `0.600 m`.

## Runtime freshness rule

Do not describe the active code as "latest v2.2".  Describe it as the active
runtime alias:

- Execution target: `sim/current`
- Current backing path: `uuv_mujoco/v2.2`
- Freshness evidence:
  `uuv_mujoco/RUNTIME_VERSION.json`, branch/HEAD check, and source-contract
  audit output.

Follow-up launcher fix in this pass:

- `sim/run_mujoco.sh`
- `sim/start_sitl_mujoco.sh`
- `sim/start_docker_sitl_mujoco.sh`
- `sim/reset_sim.sh`

These root wrappers already ran `check_runtime_freshness.py --fetch`, but did
not pass `--refresh-version`.  They now match the GUI/current-runtime launchers
and refresh `uuv_mujoco/RUNTIME_VERSION.json` whenever the active alias,
local `HEAD`, and `origin/uuv_sim` are consistent.
