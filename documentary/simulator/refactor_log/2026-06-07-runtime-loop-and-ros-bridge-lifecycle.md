# Runtime Loop And ROS Bridge Lifecycle Split

Date: 2026-06-07

Scope: behavior-neutral refactor inside `sim/current` / legacy folder
`uuv_mujoco/v2.2`.

## Changed

- Added `sim/runtime/runtime_loop_entry.py`.
  - Owns headless vs viewer loop selection.
  - Calls `ViewerRuntimeLoop` and `run_headless_loop` without changing their
    timing behavior.
  - Runs shutdown callbacks in `finally` and sets the shared stop event.
- Added `sim/runtime/ros_bridge_runtime.py`.
  - Owns mutable optional ROS/SITL bridge state.
  - Centralizes publish, QGC video bridge sharing, spin, failure disable, and
    shutdown policy.
  - Removes runner-local `nonlocal ros_bridge` publish/spin/shutdown callbacks.
- Updated `run_uuv_mujoco.py` to delegate those responsibilities.

## Measured Impact

Before this split:

- `run_uuv_mujoco.py`: `807 LOC`, `34` branches, `main()` `726 LOC`.

After this split:

- `run_uuv_mujoco.py`: `775 LOC`, `25` branches, `main()` `694 LOC`.

## Validation

Commands run:

```bash
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 -m compileall -q sim/current
env PYTHONPYCACHEPREFIX=/private/tmp/pycache PYTHONPATH=sim/current /Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python3.11 -c "from sim.runtime.ros_bridge_runtime import RosBridgeRuntime; import run_uuv_mujoco; print(RosBridgeRuntime(None).get(), hasattr(run_uuv_mujoco, 'main'))"
env PYTHONPYCACHEPREFIX=/private/tmp/pycache /Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python3.11 sim/current/run_uuv_mujoco.py --help
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 sim/current/tools/check_runtime_readiness_policy.py
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 sim/current/tools/verify_ardusub_thruster_contract.py --quiet
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 sim/current/tools/audit_code_contract_sources.py --out-dir /private/tmp/uuv_code_contract_after_ros_bridge_runtime_split
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 sim/current/tools/check_dev_os_compat.py --headless --target-os ubuntu
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 sim/current/tools/refactor_inventory.py --root sim/current --limit 30
```

Results:

- Compile: pass.
- Runner import and `--help`: pass.
- Headless cleanup smoke: pass.
- ROS bridge failure-disable smoke: pass.
- Runtime readiness policy: pass.
- ArduSub thruster contract: pass.
- Code contract audit: `fail=0`, `pass=10`, `warn=5`.
- Dev OS compatibility: `fail=0`, `pass=16`, `warn=2`.
