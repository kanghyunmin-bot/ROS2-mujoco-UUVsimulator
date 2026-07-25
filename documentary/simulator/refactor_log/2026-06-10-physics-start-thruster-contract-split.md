# Physics, Real-Start, Initial-Hold, And Thruster Contract Split

Date: 2026-06-10

## Scope

- Kept edits inside `uuv_mujoco/v2.2`.
- Did not modify ArduPilot source or the submodule pointer.
- Preserved the controller-parity and plant-input boundaries.

## Changes

- Split residual hydrodynamics runtime builders into:
  - `sim/physics/residual_hydro_runtime_builder.py`
  - `sim/physics/fossen_residual_runtime_builder.py`
  - compatibility facade `sim/physics/fossen_residual_builders.py`
- Added `tools/check_fossen_runtime_builders.py` to verify:
  - lightweight residual wrench activation
  - Fossen damping/added-mass activation
  - added-mass matrix diagonal/coupling preservation
  - custom hydrodynamics mode disables residual/Fossen double application
- Split real-start measurement calculations into:
  - `sim/runtime/real_start_measurement_errors.py`
  - `sim/runtime/real_start_pressure.py`
  - facade `sim/runtime/real_start_measurements.py`
- Added `tools/check_real_start_measurements.py` to verify:
  - Bar30 versus base-link depth contract selection
  - pressure calculation from surface pressure, density, gravity, and depth
  - wrapped yaw attitude error
  - missing velocity evidence maps to infinity
- Split initial hold pose/depth application into:
  - `sim/runtime/initial_hold_pose_capture.py`
  - `sim/runtime/initial_hold_pose_apply.py`
  - facade `sim/runtime/initial_hold_pose.py`
- Added `tools/check_initial_hold_pose.py` with fixture/case helpers to verify:
  - captured hold pose overrides depth fallback
  - qvel/qacc are zeroed when captured pose is applied
  - Bar30 depth fallback has priority over base-link depth fallback
- Split thruster performance curve parsing into:
  - `physics/thruster_performance_parse.py`
  - `physics/thruster_performance_select.py`
  - compatibility facade `physics/thruster_performance_curves.py`
- Added `tools/check_thruster_performance_curves.py` to verify:
  - invalid arrays are rejected
  - PWM/force pairs are finite-sorted
  - single-point curves are rejected
  - nearest-voltage selection is stable
- Split thruster parameter JSON loading into:
  - `sim/physics/thruster_param_payload.py`
  - `sim/physics/thruster_param_apply.py`
  - facade `sim/physics/thruster_param_loader.py`
- Added `tools/check_thruster_param_loader.py` with fixture/case helpers to verify:
  - reset-before-load behavior
  - global gain/direct-gain scaling
  - unknown thruster entries are skipped
  - per-thruster reverse asymmetry and time constants are preserved

## Validation

```text
compileall tools/bridge/sim/physics/gui: PASS
contract_source_audit: fail=0 pass=16 warn=5
closed_loop_contract: real_vs_sitl_mismatches={} missing_sitl_params=[]
fossen_runtime_builders=PASS
real_start_measurements=PASS
initial_hold_pose=PASS
thruster_performance_curves=PASS
thruster_param_loader=PASS
runtime_readiness_policy=PASS
rc_frame_contract=PASS
ardusub_thruster_contract=OK
```

## Current Inventory Impact

The following previous top hotspots are no longer in the top 25 inventory:

- `sim/physics/fossen_residual_builders.py`
- `sim/runtime/real_start_measurements.py`
- `sim/runtime/initial_hold_pose.py`
- `physics/thruster_performance_curves.py`
- `sim/physics/thruster_param_loader.py`

The remaining top items are now concentrated around Ping360 STL IO,
SITL command override handling, hydrostatic runtime assembly, JSON servo
receiver transport, and diagnostics/plot tooling.
