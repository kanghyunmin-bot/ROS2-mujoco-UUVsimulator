# Initial Depth Candidate Split

Date: 2026-06-07

Scope: active runtime under `sim/current`, backed by
`uuv_mujoco/v2.2`.

## Changes

- Split automatic initial Bar30 depth candidate logic into:
  - `sim/runtime/initial_depth_geometry.py`
  - `sim/runtime/initial_depth_model_candidates.py`
  - `sim/runtime/initial_depth_profile_candidates.py`
- Kept `sim/runtime/initial_depth_candidates.py` as a compatibility export
  surface.
- Preserved the public `sim.runtime.initial_depth` API:
  - `compute_auto_initial_bar30_depth`
  - `vec3_from_profile`

## Verification

```bash
python3 -m compileall -q \
  sim/current/sim/runtime/initial_depth.py \
  sim/current/sim/runtime/initial_depth_candidates.py \
  sim/current/sim/runtime/initial_depth_geometry.py \
  sim/current/sim/runtime/initial_depth_model_candidates.py \
  sim/current/sim/runtime/initial_depth_profile_candidates.py \
  sim/current/sim/runtime/initial_depth_limits.py \
  sim/current/sim/runtime/initial_depth_profile.py \
  sim/current/sim/runtime/initial_state_depths.py \
  sim/current/sim/runtime/initial_state.py

PYTHONPATH=sim/current python3 - <<'PY'
# Fake MuJoCo model/data smoke for compute_auto_initial_bar30_depth.
# Expected selected Bar30 depth: 0.880 m.
PY
```

## Notes

- This pass does not change GUI Start readiness semantics, real-start state
  application, Bar30 pressure/depth conversion, or plant physics coefficients.
- The split is structural so the initial-depth contract can be audited without
  coupling MuJoCo model candidates to JSON/profile candidate parsing.
