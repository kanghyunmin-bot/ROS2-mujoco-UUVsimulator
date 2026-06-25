# Physics Runtime Factory Second Split

Date: 2026-06-07

## Scope

Reduce the runtime physics factory by splitting thruster runtime creation,
underwater wrench setup, and final callback/result packaging.

## Changed Files

- `sim/runtime/physics_runtime_thrusters.py`: thruster parameter runtime,
  SITL servo binding, actuator runtime creation, and thruster debug runtime
  creation.
- `sim/runtime/physics_runtime_underwater.py`: body-velocity-local factory and
  underwater wrench runtime creation.
- `sim/runtime/physics_runtime_finalize.py`: step callback wiring and
  `RuntimePhysicsSetup` packaging.
- `sim/runtime/physics_runtime_factory.py`: preserved public factory surface.

## Contract Notes

- `create_runtime_physics_setup()` remains imported through
  `sim/runtime/physics_runtime_setup.py`.
- SITL JSON servo plant input, plant replay direct RCOU, thruster parameter
  loading, and debug CSV environment behavior are unchanged.
- No hydrodynamics, hydrostatics, actuator, or descent-contract equations were
  changed.

## Validation

```text
python3 -m compileall -q uuv_mujoco/current/sim/runtime/physics_runtime_*.py
PYTHONPATH=uuv_mujoco/current python3 - <<'PY'
from sim.runtime.physics_runtime_setup import create_runtime_physics_setup, RuntimePhysicsSetup
from sim.runtime.physics_runtime_finalize import build_runtime_physics_setup_result
print(callable(create_runtime_physics_setup), callable(build_runtime_physics_setup_result), RuntimePhysicsSetup.__name__)
PY
```

Observed status:

```text
factory import surface: PASS
refactor inventory: sim/runtime/physics_runtime_factory.py removed from top 20 hotspot list
```
