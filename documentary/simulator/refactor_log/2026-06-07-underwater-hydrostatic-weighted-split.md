# Runtime Weighted Hydrostatic Split

Date: 2026-06-07

Scope: active runtime under `sim/current`, backed by the compatibility
directory `uuv_mujoco/v2.2`.

## What Changed

- Reduced `sim/runtime/underwater_hydrostatic_weighted.py` to the public point
  and body-component wrench entry points.
- Added `sim/runtime/underwater_hydrostatic_samples.py` for converting buoyancy
  points and body components into common weighted samples.
- Added `sim/runtime/underwater_hydrostatic_accumulator.py` for submerged
  fraction, buoyancy force, CoB torque, weighted buoyancy point, and result
  construction.

## Contract Preserved

- `body_components_wrench()`, `buoyancy_points_wrench()`, and
  `weighted_hydrostatic_result()` remain importable from
  `sim.runtime.underwater_hydrostatic_weighted`.
- CoB longitudinal/vertical offsets still move only the force point, not the
  volume sample used for near-surface submerged fraction.
- Point and body-component paths still use the same slope-scaled buoyancy
  fraction, neutral volume, buoyancy scale, and CoB torque scale.

## Verification

```text
python3 -m compileall -q \
  sim/current/sim/runtime/underwater_hydrostatic_weighted.py \
  sim/current/sim/runtime/underwater_hydrostatic_accumulator.py \
  sim/current/sim/runtime/underwater_hydrostatic_samples.py \
  sim/current/sim/runtime/underwater_hydrostatic_runtime.py

PYTHONPATH=/Users/kanghyunmin/Desktop/uuv_sim/sim/current \
python3 - <<'PY'
# underwater_hydrostatic_weighted_smoke PASS
PY
```

Result: the old weighted-hydrostatic hotspot no longer appears in the top 35
hotspot inventory.
