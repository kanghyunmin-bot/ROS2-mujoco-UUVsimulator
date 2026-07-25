# Fossen Residual Hydrodynamics Split

Date: 2026-06-07

Scope: active runtime hydrodynamics code under `sim/current/sim/physics`.

## What changed

- Split Fossen/residual coefficient key constants and runtime dataclasses into
  `sim/physics/fossen_residual_types.py`.
- Split residual hydro and Fossen residual runtime coefficient construction into
  the public facade `sim/physics/fossen_residual_builders.py`.
- Further split builder internals into:
  `sim/physics/fossen_residual_config.py`,
  `sim/physics/fossen_residual_coefficients.py`,
  `sim/physics/fossen_residual_added_mass.py`, and
  `sim/physics/fossen_residual_runtime_flags.py`.
- Split body-frame residual wrench evaluation into
  `sim/physics/fossen_residual_wrench.py`.
- Reduced `sim/physics/fossen_residual.py` to compatibility exports consumed by
  `sim/runtime/hydrodynamics_runtime_wrenches.py` and
  `sim/runtime/underwater_hydrodynamics_runtime.py`.

## Contract boundaries preserved

- No coefficient values or equations changed.
- Added-mass matrix placement is unchanged.
- Custom-hydrodynamics suppression behavior is unchanged.
- HAN/CFD training/runtime separation is unchanged; this is only source
  ownership cleanup.

## Verification

```text
python3 -m py_compile \
  sim/current/sim/physics/fossen_residual.py \
  sim/current/sim/physics/fossen_residual_types.py \
  sim/current/sim/physics/fossen_residual_builders.py \
  sim/current/sim/physics/fossen_residual_config.py \
  sim/current/sim/physics/fossen_residual_coefficients.py \
  sim/current/sim/physics/fossen_residual_added_mass.py \
  sim/current/sim/physics/fossen_residual_runtime_flags.py \
  sim/current/sim/physics/fossen_residual_wrench.py \
  sim/current/sim/runtime/hydrodynamics_runtime_wrenches.py \
  sim/current/sim/runtime/underwater_hydrodynamics_runtime.py
PYTHONPATH=sim/current python3 - <<'PY'
import numpy as np
from sim.physics.fossen_residual import (
    FOSSEN_RESIDUAL_LINEAR_KEYS,
    build_fossen_residual_runtime,
    build_residual_hydro_runtime,
    fossen_residual_wrench_body,
)
print('linear_keys', len(FOSSEN_RESIDUAL_LINEAR_KEYS))
rt = build_fossen_residual_runtime(
    {'fossen_residual_hydro': {'active': True, 'linear': {'y_v': 2.0}, 'added_mass': {'active': True, 'x_u': 0.0}}},
    use_custom_hydrodynamics=False,
    env_float=lambda _name, default: default,
    env_flag=lambda _name, default: default,
)
print('active', rt.active, 'y_v', rt.linear['y_v'], 'added_mass_active', rt.added_mass_active)
w = fossen_residual_wrench_body(
    np.array([1.0, 0.5, -0.2]),
    np.array([0.1, -0.3, 0.4]),
    linear=rt.linear,
    forward_speed=rt.forward_speed,
    quadratic=rt.quadratic,
)
print('wrench', [round(float(x), 6) for x in w])
legacy = build_residual_hydro_runtime(
    {'hydro_residual_wrench': {'active': True, 'linear': {'y_v': 1.0}}},
    use_custom_hydrodynamics=False,
    env_float=lambda _name, default: default,
)
print('legacy_active', legacy.active, legacy.coeffs['y_v'])
PY
python3 -m compileall -q sim/current uuv_control_gui.py
PYTHONPATH=sim/current/tools python3 \
  sim/current/tools/audit_code_contract_sources.py \
  --out-dir /private/tmp/uuv_code_contract_after_fossen_split
python3 sim/current/tools/check_runtime_readiness_policy.py
python3 sim/current/tools/verify_ardusub_thruster_contract.py --quiet
python3 sim/current/tools/check_dev_os_compat.py --headless --target-os ubuntu
git diff --check
python3 sim/current/tools/refactor_inventory.py --limit 20
```

Results:

- Compile/import: pass.
- Numeric smoke: `y_v=2.0`, `v=0.5` produced body wrench `y=-1.0`.
- Source contract audit: `fail=0`, `pass=11`, `warn=5`.
- Runtime readiness policy: pass.
- ArduSub thruster contract: pass.
- Dev OS compatibility: `fail=0`, `pass=16`, `warn=2` (`docker_daemon`,
  `ros2_env`).
- `git diff --check`: pass.
- `sim/physics/fossen_residual.py` removed from the top hotspot list.
- `sim/physics/fossen_residual_builders.py` now keeps only public runtime
  construction flow; coefficient parsing, added-mass matrix placement, and
  activation gates are independently inspectable.
