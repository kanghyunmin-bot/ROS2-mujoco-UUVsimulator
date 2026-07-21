# Fluidcoef, Pilot, Viewer, and Body Distribution Split

Date: 2026-06-07

Scope:

- `sim/physics/fluidcoef_scale_runtime.py`
- `sim/physics/fluidcoef_scale_global.py`
- `sim/physics/fluidcoef_scale_per_geom.py`
- `sim/physics/fluidcoef_scale_extra.py`
- `gui/gui_axis_normalization.py`
- `gui/gui_math_helpers.py`
- `gui/gui_rc_axes.py`
- `gui/control_pilot_mixin.py`
- `gui/control_pilot_commands.py`
- `gui/control_pilot_publish.py`
- `gui/control_pilot_release.py`
- `sim/runtime/viewer_controls.py`
- `sim/runtime/viewer_control_state_factory.py`
- `sim/runtime/viewer_control_state_status.py`
- `sim/runtime/viewer_control_state_toggles.py`
- `sim/physics/body_distribution.py`
- `sim/physics/body_distribution_inertia.py`
- `sim/physics/body_distribution_apply.py`
- `sim/physics/body_distribution_log.py`
- `sim/physics/body_distribution_runtime.py`

Intent:

- Continue lowering branch-heavy runtime and GUI code without changing
  controller-parity observation points, plant-input semantics, ArduPilot source,
  PWM routing, or MuJoCo physical coefficients.
- Keep compatibility import surfaces stable while moving mixed responsibility
  blocks into focused helpers.

Contract notes:

- `mujoco_fluidcoef_geom_scales` continues to use glob-style geom matching.
- GUI pilot control still uses the same RC override versus MANUAL_CONTROL mode
  policy and still requests initial-depth release only after a non-deadband
  pilot axis is present.
- Viewer pause, follow camera, sensor overlay, and thruster-label toggles keep
  the same public methods on `ViewerControlState`.
- Distributed body component setup still preserves qpos/qvel across
  `mj_setConst`, calls `mj_forward`, and applies the same mass/CoM/parallel-axis
  inertia calculation.

Verification:

- `fluidcoef scale split smoke: PASS`
- `control pilot split smoke: PASS`
- `viewer controls split smoke: PASS`
- `body distribution split smoke: PASS`
- `python3 -m compileall -q uuv_mujoco/current uuv_control_gui.py`
- `git diff --check`
- `python3 uuv_mujoco/current/tools/audit_code_contract_sources.py`
  -> `{"fail": 0, "pass": 11, "warn": 5}`
- `python3 uuv_mujoco/current/tools/check_runtime_readiness_policy.py`
  -> `runtime_readiness_policy=PASS`
- `python3 uuv_mujoco/current/tools/verify_ardusub_thruster_contract.py --quiet`
  -> `[thruster-contract] OK`
- `/Users/kanghyunmin/.venvs/mujoco311/bin/python
  uuv_mujoco/current/tools/physics_contract_audit.py --simulate-s 0.05`

Inventory effect:

- `sim/physics/fluidcoef_scale_runtime.py` no longer appears in the top hotspot
  list.
- `sim/runtime/viewer_controls.py` no longer appears in the top hotspot list.
- `gui/control_pilot_mixin.py` no longer appears in the top hotspot list.
- `sim/physics/body_distribution.py` no longer appears in the top hotspot list.

Next candidate:

- `sim/physics/hydrostatic_runtime_values.py` remains a physics-contract
  hotspot and should be split by env/profile/default/real-start-trim ownership
  in a separate pass.
