# Physics Step Callback Split

Date: 2026-06-07

Scope: active runtime `sim/current`, backed by the compatibility
directory `uuv_mujoco/v2.2`.

## Change

`sim/runtime/physics_step_callbacks.py` is now a callback assembly facade.
Thruster command callbacks moved to:

- `sim/runtime/physics_step_thruster_callbacks.py`

Auxiliary underwater wrench, debug, and descent-guard callbacks moved to:

- `sim/runtime/physics_step_aux_callbacks.py`

The public entry point remains:

- `build_step_physics_callbacks`

The public dataclass remains:

- `StepPhysicsCallbacks`

## Verification

Commands run:

```text
python3 -m compileall -q sim/current/sim/runtime/physics_step_callbacks.py ...
PYTHONPATH=sim/current python3 - <<'PY'  # fake callback wiring smoke
python3 sim/current/tools/refactor_inventory.py --root sim/current --limit 30
```

Results:

- Physics step callback split smoke: PASS.
- Scheduler, actuator force update, propeller visual update, direct-command
  mixing, underwater wrench application, and debug emission were all reached
  through the assembled callbacks.
- `physics_step_callbacks.py` dropped out of the top 30 hotspot table.

## Contract Notes

This refactor does not change MuJoCo stepping order. `simulation_step_physics.py`
still calls update forces, visuals, initial hold, underwater wrench, thruster
debug emission, descent guard, and `mj_step` in the same order.
