# Hydrodynamics Helper Split

Date: 2026-06-07

## Scope

Split the shared hydrodynamics helper hotspot while preserving all equations,
imports, and runtime contracts.

## Changed Files

- `physics/hydrodynamics_math.py`: `skew()`,
  `added_mass_coriolis()`, and `first_order_response()`.
- `physics/thruster_curve_helpers.py`: `shape_thruster_command()`,
  `polyval_ascending()`, and `scaled_polynomial_force()`.
- `physics/hydrostatic_fraction_helpers.py`:
  `submerged_fraction_linear()`, `submerged_fraction_ellipsoid()`, and
  `submerged_fraction()`.
- `physics/ellipsoid_hydrodynamics.py`: equivalent-ellipsoid volume,
  projected areas, depolarization factors, and baseline coefficient estimate.
- `physics/hydrodynamics_helpers.py`: compatibility export surface.

## Contract Notes

- No hydrostatic, hydrodynamic, added-mass, or thruster-force equations were
  changed.
- Existing imports such as
  `from physics.hydrodynamics_helpers import submerged_fraction` remain valid.
- No direct `import hydrodynamics_helpers` usage exists in the active runtime.

## Validation

```text
python3 -m compileall -q uuv_mujoco/current/physics/hydrodynamics*.py \
  uuv_mujoco/current/physics/thruster_curve_helpers.py \
  uuv_mujoco/current/physics/hydrostatic_fraction_helpers.py \
  uuv_mujoco/current/physics/ellipsoid_hydrodynamics.py
PYTHONPATH=uuv_mujoco/current python3 - <<'PY'
import numpy as np
from physics.hydrodynamics_helpers import (
    added_mass_coriolis,
    estimate_ellipsoid_hydrodynamics,
    scaled_polynomial_force,
    shape_thruster_command,
    submerged_fraction,
)
C = added_mass_coriolis(np.ones(6), np.array([0.1, 0.2, 0.3, 0.01, 0.02, 0.03]))
est = estimate_ellipsoid_hydrodynamics((0.5, 0.2, 0.1), 1000.0)
print(C.shape, len(est.added_mass_diag), submerged_fraction(0.0, 1.0))
PY
```

Observed status:

```text
package import smoke: PASS
added-mass Coriolis shape: (6, 6)
ellipsoid added-mass diagonal length: 6
waterline ellipsoid submerged fraction at depth=0: 0.5
refactor inventory: physics/hydrodynamics_helpers.py removed from top 20 hotspot list
```
