# Simulation profile parsing split

Date: 2026-06-07

## Scope

- Added focused profile parsing modules:
  - `physics/sim_profile_parse_common.py`
  - `physics/sim_profile_hydrostatic_points.py`
  - `physics/sim_profile_hydrostatic_restoring.py`
- Kept `physics/sim_profile_parsing.py` as a compatibility export surface.

## Contract

The split preserves the existing imports used by
`physics/sim_profile_hydrodynamics.py` and `physics/sim_profile_ellipsoid.py`.
Profile field semantics, body component filtering, buoyancy point filtering,
and hydrostatic restoring defaults are unchanged.

## Verification

```text
python3 -m compileall -q sim/current/physics/sim_profile_*.py
PYTHONPATH=sim/current python3 - <<'PY'
from pathlib import Path
from physics.sim_profile_helpers import load_sim_profiles, build_sim_profile, build_hydrodynamics_config
profiles, warning = load_sim_profiles(Path('sim/current/config/sim_profiles.json'))
profile = build_sim_profile(profiles, 'current')
config = build_hydrodynamics_config(profile)
print(len(config.body_components), len(config.buoyancy_points), config.hydrostatic_volume_source, config.model_source)
PY
```

Smoke result: `3` body components, `4` buoyancy points,
`body_components` hydrostatic source, and `ellipsoid-baseline` model source.
