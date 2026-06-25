# 2026-06-07 Dynamic Fluidcoef / Viewer / Profile / Dev-OS Split

Scope: active runtime under `uuv_mujoco/current`, backed by the compatibility
directory `uuv_mujoco/v2.2`.

This pass reduced branch-heavy files without changing ArduPilot, the ArduPilot
submodule pointer, controller parity shims, PWM remaps, JSON servo plant-input
semantics, or hydrodynamic coefficient values.

## Changes

- Split `sim/physics/dynamic_fluidcoef_runtime.py`:
  - `sim/physics/dynamic_fluidcoef_runtime_update.py` owns update scheduling,
    load blend calculation, and target `geom_fluid` application.
  - `sim/physics/dynamic_fluidcoef_runtime_transient.py` owns transient
    onset/rearm/retrigger and decay math.
  - `sim/physics/dynamic_fluidcoef_runtime_logging.py` owns debug output.
  - The public `DynamicFluidcoefRuntime.update()` order is preserved.
- Split `sim/runtime/viewer_controls.py`:
  - `sim/runtime/viewer_control_config.py` owns initial camera/follow settings.
  - `sim/runtime/viewer_control_keys.py` owns keycode-to-action mapping.
  - `sim/runtime/viewer_control_camera.py` owns camera application/release.
  - `ViewerControlState` remains the runner-facing API.
- Split `tools/audit_closed_loop_profile.py`:
  - `tools/audit_closed_loop_thruster_curve.py` owns thruster-performance curve
    selection and force range extraction.
  - Existing imports of `selected_thruster_curve()` from
    `audit_closed_loop_profile.py` remain valid.
- Split `tools/dev_os_compat_common.py`:
  - `tools/dev_os_compat_exec.py` owns subprocess and executable path helpers.
  - `tools/dev_os_compat_python_probe.py` owns Python/mjpython candidate
    discovery and MuJoCo import probing.
  - `dev_os_compat_common.py` now stays focused on shared paths, result records,
    and target OS normalization.

## Inventory Impact

The following former top-35 active-runtime hotspots no longer appear there:

- `sim/physics/dynamic_fluidcoef_runtime.py`
- `sim/runtime/viewer_controls.py`
- `tools/audit_closed_loop_profile.py`
- `tools/dev_os_compat_common.py`

The dynamic fluidcoef split initially created a new large update helper; that
was immediately corrected by moving transient and logging ownership to separate
modules.  The split therefore did not just move the hotspot.

## Validation

Targeted smokes:

- Dynamic fluidcoef fake-runtime smoke verified update scheduling, `geom_fluid`
  writes, and transient path execution.
- Viewer controls fake-runtime smoke verified pause, label toggle, sensor
  overlay toggle, fixed stereo camera, follow camera, and free-camera release.
- Closed-loop profile smoke verified `current` runtime resolution and active
  thruster curve selection.
- Dev-OS compatibility check still reports `fail=0`, `pass=16`, `warn=2`.

Full gates run after this pass:

```bash
python3 -m compileall -q uuv_mujoco/current uuv_control_gui.py
PYTHONPATH=uuv_mujoco/current/tools python3 uuv_mujoco/current/tools/check_runtime_freshness.py --workspace /Users/kanghyunmin/Desktop/uuv_sim --runtime-dir /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current --fetch --refresh-version
PYTHONPATH=uuv_mujoco/current/tools python3 uuv_mujoco/current/tools/audit_code_contract_sources.py --out-dir /private/tmp/uuv_source_audit_spaghetti_20260607_dynamic_viewer_profile_devos_split
python3 uuv_mujoco/current/tools/check_runtime_readiness_policy.py
python3 uuv_mujoco/current/tools/verify_ardusub_thruster_contract.py --quiet
python3 uuv_mujoco/current/tools/check_dev_os_compat.py --headless --target-os ubuntu
/Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python uuv_mujoco/current/tools/physics_contract_audit.py --output-dir /private/tmp/uuv_physics_audit_spaghetti_20260607_dynamic_viewer_profile_devos_split --simulate-s 0
git diff --check
```
