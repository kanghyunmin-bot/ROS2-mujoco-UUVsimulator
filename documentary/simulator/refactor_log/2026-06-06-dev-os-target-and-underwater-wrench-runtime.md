# Dev OS Target Gate and Underwater Wrench Runtime

Date: 2026-06-06

## Scope

This pass stayed inside `uuv_mujoco/v2.2`.  It did not edit ArduPilot source
and did not change the ArduPilot submodule pointer.

## Changes

- Extracted underwater hydrostatic/hydrodynamic force application from
  `run_urdf_full.py` into `sim/runtime/underwater_wrench_runtime.py`.
- Kept the existing `hydrostatic_runtime` and `hydro_runtime` objects as the
  source of coefficients so this split does not tune or change physics values.
- Rewired thruster debug, descent guard, and viewer buoyancy debug to read
  `UnderwaterWrenchRuntime.last_buoy_force` and `last_buoy_point`.
- Added `--target-os` to `tools/check_dev_os_compat.py`.
- Added Ubuntu migration checks for launcher markers, Docker SITL host routing,
  JSON port routing, and documentation presence.
- Relaxed macOS `mjpython` probing from hard failure to warning when the
  executable exists but restricted shells cannot run it, while the selected
  runtime Python still imports `mujoco.viewer`.
- Updated `docs/contracts/DEV_OS_COMPATIBILITY.md` with Ubuntu migration
  commands and the macOS `mjpython` probe caveat.
- Extracted the headless/viewer loop into
  `sim/runtime/simulation_loop_runtime.py`, including viewer-only vector
  normalization for debug geometry.
- Renamed the primary runtime entrypoint to `run_uuv_mujoco.py`.
- Kept `run_urdf_full.py` as a compatibility wrapper and updated launch/reset
  process detection to recognize both names.

## Validation

Commands run:

```bash
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 -m compileall -q uuv_mujoco/v2.2
git diff --check -- uuv_mujoco/v2.2/run_urdf_full.py uuv_mujoco/v2.2/sim/runtime/underwater_wrench_runtime.py uuv_mujoco/v2.2/tools/check_dev_os_compat.py uuv_mujoco/v2.2/docs/contracts/DEV_OS_COMPATIBILITY.md
python3 uuv_mujoco/v2.2/tools/check_dev_os_compat.py --require-viewer
python3 uuv_mujoco/v2.2/tools/check_dev_os_compat.py --headless --target-os ubuntu
python3 uuv_mujoco/v2.2/tools/audit_code_contract_sources.py --out-dir /private/tmp/uuv_code_contract_after_dev_os_underwater_split_v2
python3 uuv_mujoco/v2.2/tools/check_runtime_readiness_policy.py
python3 uuv_mujoco/v2.2/tools/verify_ardusub_thruster_contract.py --quiet
python3 uuv_mujoco/v2.2/tools/refactor_inventory.py --root uuv_mujoco/v2.2 --limit 15
python3 uuv_mujoco/v2.2/run_uuv_mujoco.py --help
python3 uuv_mujoco/v2.2/run_urdf_full.py --help
```

Observed results:

- compileall: pass
- diff whitespace check: pass
- macOS viewer preflight: `fail=0`, `warn=3`
- Ubuntu target headless preflight: `fail=0`, `warn=2`
- source contract audit: `fail=0`, `warn=5`
- runtime readiness policy: pass
- ArduSub thruster contract: pass
- `run_urdf_full.py`: `1288 LOC -> 984 LOC`
- `run_urdf_full.py` largest symbol: `main 1196 LOC -> 898 LOC`
- after viewer loop split and rename, `run_uuv_mujoco.py`: `862 LOC`
- after viewer loop split and rename, `run_uuv_mujoco.py` largest symbol:
  `main 781 LOC`
- `run_urdf_full.py` is now a thin compatibility wrapper.
- after step runtime split, `run_uuv_mujoco.py`: `808 LOC`
- after step runtime split, `run_uuv_mujoco.py` largest symbol:
  `main 726 LOC`

Current warnings:

- Docker daemon is not running in the current shell.
- ROS2 is not sourced in the current shell; launcher-managed setup remains the
  expected GUI path.
- `mjpython -c ...` exits silently in the restricted shell, but runtime Python
  imports `mujoco.viewer`.

## Remaining Risk

`run_uuv_mujoco.py` is still large.  The next meaningful split should move the
remaining ROS publish lifecycle, shutdown lifecycle, or setup wiring into
runtime objects, while keeping plant input as raw SITL JSON servo and
preserving the existing contract gates.
