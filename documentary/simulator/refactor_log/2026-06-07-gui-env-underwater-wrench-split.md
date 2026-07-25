# GUI Env Contract and Underwater Wrench Runtime Split

Date: 2026-06-07

Scope: active runtime under `sim/current`, backed by the compatibility
directory `uuv_mujoco/v2.2`.

## Changes

- Split `gui/sim_stack_env.py` into focused GUI simulator-start contract
  modules:
  - `gui/sim_stack_env_types.py`
  - `gui/sim_stack_env_flags.py`
  - `gui/sim_stack_env_contract.py`
  - `gui/sim_stack_env_args.py`
- Split `sim/runtime/underwater_wrench_runtime.py` into focused runtime force
  modules:
  - `sim/runtime/underwater_wrench_types.py`
  - `sim/runtime/underwater_hydrostatic_runtime.py`
  - `sim/runtime/underwater_hydrodynamics_runtime.py`

## Contract Notes

- GUI Start still defaults to closed-loop RC override through ArduSub, with
  `ROS2_UUV_MAVROS_RC_PWM_SPAN=400`, dedicated command link enabled, and
  plant replay disabled unless `UUV_RUN_MODE=plant_replay`.
- Initial-depth defaults remain Bar30-depth first because ALT_HOLD consumes
  pressure/depth semantics, not base-link depth alone.
- Underwater wrench ownership remains single-owner on MuJoCo `xfrc_applied`.
  The public `UnderwaterWrenchRuntime.apply(dt)` sequence is preserved while
  hydrostatic and hydrodynamic calculation branches are separated for audit.
- No controller-parity shim, PWM remap, output remap, or JSON-servo telemetry
  resampling was added.

## Verification

- `python3 -m py_compile` passed for the split GUI env and underwater wrench
  modules.
- Standalone `build_gui_sim_stack_env()` smoke preserved key closed-loop
  defaults:
  - `UUV_RUN_MODE=closed_loop`
  - `UUV_EKF_CONTRACT=poshold_extnav`
  - `ROS2_UUV_MAVROS_RC_PWM_SPAN=400`
  - `SITL_EKF3_EXTNAV=1`
  - `ROS2_UUV_ALLOW_RCOUT_PLANT_OVERRIDE=0`
- `UnderwaterWrenchRuntime` plus the new hydrostatic/hydrodynamic helper
  imports passed under the MuJoCo runtime Python.
- `run_uuv_mujoco.py --help`, `compileall`, source-contract audit, and static
  physics contract audit passed after the split.
