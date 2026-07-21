# Model Setup Physics Runtime Split

Date: 2026-06-07

Scope: active runtime physics setup under `uuv_mujoco/current/sim/physics`.

## Changes

- Split `sim/physics/model_setup.py` into focused runtime modules:
  - `sim/physics/fluid_option_runtime.py`
  - `sim/physics/pool_runtime_overrides.py`
  - `sim/physics/fluid_geom_runtime.py`
  - `sim/physics/body_tree.py`
- Kept `sim/physics/model_setup.py` as a compatibility facade for existing
  imports from `sim.runtime.model_runtime_setup` and hydrostatic setup.

## Contract Notes

- No ArduPilot source, submodule pointer, controller parity observation surface,
  plant input surface, PWM remap, or ALT_HOLD shim changed.
- MuJoCo fluid option scaling still applies density and viscosity scales before
  fluid ownership is configured.
- Plant replay still defaults to the larger pool XY runtime scale to avoid
  artificial wall contact.
- MuJoCo fluid geom size and fluidcoef scaling still return:
  `fluid_geom_ids`, `fluid_geom_names`, and `fluidcoef_static_geom_scales`.
- Body subtree mass calculation remains available through the existing
  `sim.physics.model_setup.body_subtree_mass` import path.

## Verification

```bash
python3 -m py_compile \
  uuv_mujoco/current/sim/physics/model_setup.py \
  uuv_mujoco/current/sim/physics/fluid_option_runtime.py \
  uuv_mujoco/current/sim/physics/pool_runtime_overrides.py \
  uuv_mujoco/current/sim/physics/fluid_geom_runtime.py \
  uuv_mujoco/current/sim/physics/body_tree.py \
  uuv_mujoco/current/sim/runtime/model_runtime_setup.py \
  uuv_mujoco/current/sim/runtime/physics_runtime_hydrostatic.py
PYTHONPATH=uuv_mujoco/current python3 - <<'PY'
from sim.physics.model_setup import (
    apply_fluid_geom_runtime_scales,
    apply_fluid_option_scales,
    apply_pool_runtime_overrides,
    body_subtree_mass,
    build_body_children,
)
print(callable(apply_fluid_geom_runtime_scales))
print(callable(apply_fluid_option_scales))
print(callable(apply_pool_runtime_overrides))
print(callable(body_subtree_mass))
print(callable(build_body_children))
PY
python3 uuv_mujoco/current/tools/refactor_inventory.py --root uuv_mujoco/current --limit 25
```

Results:

- `sim/physics/model_setup.py` dropped from `351 LOC / 48` branches to a
  `16 LOC` facade.
- The new largest related module, `sim/physics/fluid_geom_runtime.py`, is
  `263 LOC` and stays below the current top 25 hotspot inventory.
- Existing public functions remain callable from `sim.physics.model_setup`.
