# Physics Profile, ExternalNav, And Runtime Setup Split

Date: 2026-06-07

Scope: active runtime under `sim/current`, backed by the compatibility
directory `uuv_mujoco/v2.2`.

## Changes

- Split hydrodynamics profile parsing into:
  - `physics/sim_profile_parsing.py`
  - `physics/sim_profile_ellipsoid.py`
  - `physics/sim_profile_hydrodynamics.py`
- Split ExternalNav/VPD transport helpers into:
  - `bridge/sitl_external_nav_bootstrap.py`
  - `bridge/sitl_native_vpd_runtime.py`
  - `bridge/sitl_external_nav_synthetic_vpd.py`
  - `bridge/sitl_external_nav_cache_contract.py`
  - `bridge/sitl_external_nav_runtime.py` facade
- Split physics runtime wiring into:
  - `sim/runtime/physics_runtime_types.py`
  - `sim/runtime/physics_runtime_hydrostatic.py`
  - `sim/runtime/physics_runtime_geometry.py`
  - `sim/runtime/physics_runtime_factory.py`
  - `sim/runtime/physics_runtime_setup.py` facade

## Contract Notes

- No coefficient value, ArduSub parameter, thruster sign, PWM remap, or
  controller/plant observation point was changed.
- `SitlTransport` still binds the same ExternalNav method names through the
  compatibility facade.
- `run_uuv_mujoco.py` still imports `create_runtime_physics_setup` from the
  original public path.

## Local Validation

```bash
python3 -m py_compile \
  sim/current/physics/sim_profile_hydrodynamics.py \
  sim/current/physics/sim_profile_parsing.py \
  sim/current/physics/sim_profile_ellipsoid.py \
  sim/current/bridge/sitl_external_nav_runtime.py \
  sim/current/bridge/sitl_external_nav_bootstrap.py \
  sim/current/bridge/sitl_native_vpd_runtime.py \
  sim/current/bridge/sitl_external_nav_synthetic_vpd.py \
  sim/current/bridge/sitl_external_nav_cache_contract.py \
  sim/current/sim/runtime/physics_runtime_setup.py \
  sim/current/sim/runtime/physics_runtime_factory.py \
  sim/current/sim/runtime/physics_runtime_types.py \
  sim/current/sim/runtime/physics_runtime_hydrostatic.py \
  sim/current/sim/runtime/physics_runtime_geometry.py
PYTHONPATH=sim/current python3 - <<'PY'
from sim.runtime.physics_runtime_setup import RuntimePhysicsSetup, create_runtime_physics_setup
print(RuntimePhysicsSetup.__name__, callable(create_runtime_physics_setup))
PY
python3 sim/current/tools/refactor_inventory.py --limit 25 --format markdown
```
