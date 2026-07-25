# Ellipsoid Hydrodynamics Split

Date: 2026-06-07

Scope: active runtime under `sim/current`, backed by the compatibility
directory `uuv_mujoco/v2.2`.

## What Changed

- Reduced `physics/ellipsoid_hydrodynamics.py` to the public compatibility
  facade.
- Added `physics/ellipsoid_geometry.py` for semi-axis validation, volume,
  projected-area, and depolarization-factor calculations.
- Added `physics/ellipsoid_hydro_types.py` for `EllipsoidHydroEstimate`.
- Added `physics/ellipsoid_hydro_coefficients.py` for 6DOF added-mass and
  damping coefficient assembly.

## Contract Preserved

- `physics.hydrodynamics_helpers` still re-exports the same ellipsoid helpers.
- `sim_profile_ellipsoid.resolve_ellipsoid_baseline()` still calls
  `estimate_ellipsoid_hydrodynamics()` with the same arguments.
- Semi-axis validation, volume, projected area, depolarization sum, and 6DOF
  coefficient array shapes are preserved.

## Verification

```text
python3 -m compileall -q \
  sim/current/physics/ellipsoid_hydrodynamics.py \
  sim/current/physics/ellipsoid_geometry.py \
  sim/current/physics/ellipsoid_hydro_coefficients.py \
  sim/current/physics/ellipsoid_hydro_types.py \
  sim/current/physics/hydrodynamics_helpers.py \
  sim/current/physics/sim_profile_ellipsoid.py

PYTHONPATH=/Users/kanghyunmin/Desktop/uuv_sim/sim/current \
python3 - <<'PY'
# ellipsoid_hydro_smoke PASS
# ellipsoid_profile_surface_smoke PASS
PY
```

Result: the old `physics/ellipsoid_hydrodynamics.py` hotspot is now a facade;
the coefficient assembly remains visible in a branch-free module.
