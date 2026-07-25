# 2026-06-07 Initial Depth, Step Callback, And Thruster Runtime Split

## Scope

- Split initial depth application into mutation, surface-hysteresis warning,
  and log-formatting helpers.
- Split step physics callback wiring into callback type definitions and the
  body-frame velocity reader while preserving the existing import surface.
- Split thruster runtime setup into param/SITL-servo binding, actuator runtime
  creation, debug runtime creation, and typed setup bundles.

## Contract Notes

- `sim.runtime.initial_state_pose_depth.apply_initial_depth_request` remains the
  public initial-depth entry point.
- Bar30 and base-link depth log text is preserved, including hold-state suffix
  and SURFACE_DEPTH hysteresis warning semantics.
- `sim.runtime.physics_step_callbacks.body_velocity_local_factory` remains
  importable for compatibility, but its implementation now lives in
  `body_velocity_local.py`.
- `sim.runtime.physics_runtime_thrusters` remains a compatibility facade.
- The thruster actuator runtime builder now accepts the argument shape used by
  `physics_runtime_factory_thrusters.py`: `thruster_global`, `thruster_scale`,
  `thruster_direct_scale`, reverse asymmetry, and tau maps.  The stale
  `thruster_param_runtime` argument shape was removed from that active builder.

## Verification

- Initial-depth dispatch smoke with fake base-state for both Bar30 and
  base-depth requests.
- Initial-depth MuJoCo smoke with `tank_current_scene.xml` and Bar30 depth set.
- Thruster runtime signature smoke verifying the active builder accepts
  `thruster_global` and no longer exposes `thruster_param_runtime`.
- `python3 -m compileall -q sim/current uuv_control_gui.py`
- `git diff --check`
- `python3 sim/current/tools/audit_code_contract_sources.py`
- `python3 sim/current/tools/check_runtime_readiness_policy.py`
- `python3 sim/current/tools/verify_ardusub_thruster_contract.py --quiet`
- `/Users/kanghyunmin/.venvs/mujoco311/bin/python sim/current/tools/physics_contract_audit.py --simulate-s 0.05`

## Inventory Effect

- `sim/runtime/initial_state_pose_depth.py` left the top hotspot list.
- `sim/runtime/physics_step_callbacks.py` left the top hotspot list.
- `sim/runtime/physics_runtime_thrusters.py` left the top hotspot list.
