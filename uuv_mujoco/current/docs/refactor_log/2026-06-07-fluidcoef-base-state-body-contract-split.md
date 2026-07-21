# 2026-06-07 Fluidcoef, Base State, And Body Contract Split

## Scope

- Split dynamic MuJoCo fluid coefficient pattern setup into pattern matching,
  reference-ratio preparation, per-row weight parsing, array mutation, and
  logging.
- Split MuJoCo base-state helpers into id lookup, free-joint qpos/qvel
  mutation, Bar30 pressure-site depth helpers, and public mixin methods.
- Align static body-contract audit with the runtime body-distribution inertia
  calculation instead of keeping duplicated mass/CoM/parallel-axis logic.
- Fix `sim/runtime/initial_state.py` so Bar30 real-start pressure calibration
  can use `os.environ` at runtime.

## Contract Notes

- No dynamic fluid coefficient values, multipliers, axis weights, or log text
  semantics were intentionally changed.
- `sim.physics.dynamic_fluidcoef_setup_rows.apply_dynamic_fluidcoef_pattern`
  remains the caller-facing API used by dynamic-fluidcoef setup.
- `sim.runtime.base_state.MuJoCoBaseState` keeps the same public methods:
  `create`, `base_origin_world`, `reset_free_joint_velocity`,
  `set_base_depth`, `set_base_position_xy`, `bar30_world_z`,
  `bar30_depth_now_m`, `set_bar30_depth`, and `set_base_attitude_rpy`.
- `tools.physics_contract_body.component_self_inertia_diag` remains exported
  through the existing compatibility facade, but its implementation now comes
  from the runtime body-distribution inertia module.

## Verification

- Dynamic fluidcoef pattern split smoke: applied a wildcard pattern to two
  fluid geoms and checked `reference`, `weights`, and active geom ids.
- Base-state MuJoCo smoke: loaded `tank_current_scene.xml`, applied base depth,
  xy position, Bar30 depth, and RPY attitude through `MuJoCoBaseState`.
- `python3 -m compileall -q uuv_mujoco/current uuv_control_gui.py`
- `git diff --check`
- `python3 uuv_mujoco/current/tools/audit_code_contract_sources.py`
- `python3 uuv_mujoco/current/tools/check_runtime_readiness_policy.py`
- `python3 uuv_mujoco/current/tools/verify_ardusub_thruster_contract.py --quiet`
- `/Users/kanghyunmin/.venvs/mujoco311/bin/python uuv_mujoco/current/tools/physics_contract_audit.py --simulate-s 0.05`

## Inventory Effect

- `sim/physics/dynamic_fluidcoef_setup_rows.py` left the top hotspot list.
- `sim/runtime/base_state.py` left the top hotspot list after moving public
  methods into focused mixins.
- `tools/physics_contract_body.py` left the top hotspot list and now shares the
  runtime inertia calculation source.
