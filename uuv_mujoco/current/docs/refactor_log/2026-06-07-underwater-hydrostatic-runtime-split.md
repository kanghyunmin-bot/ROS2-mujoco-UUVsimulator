# Underwater Hydrostatic Runtime Split

Date: 2026-06-07

## Scope

Split hydrostatic force-source calculations without changing buoyancy,
restoring torque, or waterline submerged-fraction equations.

## Changed Files

- `sim/runtime/underwater_hydrostatic_weighted.py`: weighted buoyancy point and
  body component force/torque calculations plus weighted result assembly.
- `sim/runtime/underwater_hydrostatic_restoring.py`: release-blended roll/pitch
  restoring torque.
- `sim/runtime/underwater_hydrostatic_runtime.py`: public
  `apply_hydrostatic_wrench()` selector and compatibility aliases for private
  helper names.

## Contract Notes

- CoB offsets still affect restoring force application points, not volume
  sampling points.
- `HydrostaticWrenchResult` shape is unchanged.
- Existing private helper names are kept as aliases for compatibility with any
  local diagnostics.

## Validation

```text
python3 -m compileall -q uuv_mujoco/current/sim/runtime/underwater_hydrostatic*.py
PYTHONPATH=uuv_mujoco/current python3 - <<'PY'
from sim.runtime.underwater_hydrostatic_runtime import apply_hydrostatic_wrench, _buoyancy_points_wrench, _restoring_tau_world
print(callable(apply_hydrostatic_wrench), callable(_buoyancy_points_wrench), callable(_restoring_tau_world))
PY
```

Observed status:

```text
hydrostatic import surface: PASS
refactor inventory: sim/runtime/underwater_hydrostatic_runtime.py removed from top 20 hotspot list
```
