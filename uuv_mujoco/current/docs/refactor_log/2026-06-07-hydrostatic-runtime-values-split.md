# Hydrostatic Runtime Values Split

Date: 2026-06-07

Scope:

- `sim/physics/hydrostatic_runtime_values.py`
- `sim/physics/hydrostatic_runtime_cob.py`
- `sim/physics/hydrostatic_runtime_restoring.py`
- `sim/physics/hydrostatic_restoring_base.py`
- `sim/physics/hydrostatic_restoring_real_start.py`

Intent:

- Remove the branch-heavy hydrostatic env/profile value reader from the active
  hotspot list without changing hydrostatic coefficients, profile defaults, or
  real-start trim semantics.
- Keep `read_hydrostatic_runtime_values()` as the compatibility surface while
  splitting CoB offset extraction, restoring base env/default extraction, and
  real-start roll/pitch trim override ownership.

Contract notes:

- `UUV_COB_TORQUE_SCALE`, `UUV_COB_X_OFFSET_M`, and `UUV_COB_Z_OFFSET_M`
  preserve the same profile fallback behavior.
- Restoring stiffness values are still clamped to non-negative values before
  active-state resolution.
- `UUV_HYDROSTATIC_RESTORING_TRIM_FROM_REAL_START` is still evaluated through
  `env_flag()` and only applies finite real-start trims when real-start is
  required.
- If both restoring stiffness values are zero or negative after clamping,
  restoring remains disabled.

Verification:

- `python3 -m compileall -q uuv_mujoco/current/sim/physics/hydrostatic_runtime_values.py uuv_mujoco/current/sim/physics/hydrostatic_runtime_cob.py uuv_mujoco/current/sim/physics/hydrostatic_runtime_restoring.py uuv_mujoco/current/sim/physics/hydrostatic_restoring_base.py uuv_mujoco/current/sim/physics/hydrostatic_restoring_real_start.py`
- `hydrostatic restoring split smoke: PASS`
- `python3 uuv_mujoco/current/tools/refactor_inventory.py --root uuv_mujoco/current --limit 35 --format markdown`

Inventory effect:

- `sim/physics/hydrostatic_runtime_values.py` no longer appears in the top
  hotspot list.
- `sim/physics/hydrostatic_runtime_restoring.py` was split again after the first
  pass because it initially became the new top hydrostatic hotspot.

Next candidate:

- `gui/physics_param_apply.py` and `gui/replay_controls.py` remain GUI-side
  branch-heavy files.
- `sim/runtime/runner_physics_setup.py` remains an execution-path setup hotspot
  and should be split only with contract audits around thruster, hydrostatic,
  and plant-input ownership.
