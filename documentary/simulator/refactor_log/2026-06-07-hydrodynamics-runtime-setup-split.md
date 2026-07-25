# Hydrodynamics Runtime Setup Split

Date: 2026-06-07

Scope: active MuJoCo runtime setup under `sim/current/sim/runtime`.

## Changes

- Split `sim/runtime/hydrodynamics_runtime_setup.py` into focused modules:
  - `sim/runtime/hydrodynamics_runtime_types.py`
  - `sim/runtime/hydrodynamics_runtime_values.py`
  - `sim/runtime/hydrodynamics_runtime_wrenches.py`
  - `sim/runtime/hydrodynamics_runtime_logging.py`
- Kept `sim/runtime/hydrodynamics_runtime_setup.py` as the public assembly
  surface for `sim/runtime/physics_runtime_factory.py`.
- Preserved the `HydrodynamicsRuntimeSetup` import path by re-exporting the
  typed setup dataclass from the facade module.

## Contract Notes

- No hydrodynamic coefficient values, MuJoCo fluidcoef runtime setup, water
  current contract, CFD/Fossen residual enablement, neutral volume override,
  thruster-loop scheduler rate, or returned setup field names were intentionally
  changed.
- MuJoCo built-in ellipsoid mode still applies ambient current through
  `model.opt.wind`.
- Custom hydrodynamics mode still reports the same 6DOF added-mass/damping
  diagnostics.
- No ArduPilot source, submodule pointer, controller parity observation surface,
  JSON servo plant input, PWM remap, or ALT_HOLD shim changed.

## Verification

```bash
python3 -m py_compile \
  sim/current/sim/runtime/hydrodynamics_runtime_setup.py \
  sim/current/sim/runtime/hydrodynamics_runtime_types.py \
  sim/current/sim/runtime/hydrodynamics_runtime_values.py \
  sim/current/sim/runtime/hydrodynamics_runtime_wrenches.py \
  sim/current/sim/runtime/hydrodynamics_runtime_logging.py \
  sim/current/sim/runtime/physics_runtime_factory.py
source ./.uuv_mujoco_env.sh && "$MJ311_PYTHON" sim/current/run_uuv_mujoco.py --help
source ./.uuv_mujoco_env.sh && PYTHONPATH=sim/current/tools \
  "$MJ311_PYTHON" sim/current/tools/physics_contract_audit.py \
  --output-dir /private/tmp/uuv_physics_contract_after_hydro_runtime_split \
  --simulate-s 0
python3 sim/current/tools/refactor_inventory.py --root sim/current --limit 35
```

Results:

- `sim/runtime/hydrodynamics_runtime_setup.py` dropped from `335 LOC / 13`
  branches to `100 LOC / 0` branches.
- The new focused modules are:
  - `hydrodynamics_runtime_types.py`: `120 LOC / 0` branches
  - `hydrodynamics_runtime_values.py`: `156 LOC / 8` branches
  - `hydrodynamics_runtime_wrenches.py`: `73 LOC / 2` branches
  - `hydrodynamics_runtime_logging.py`: `135 LOC / 3` branches
- Physics contract audit preserved the neutral static force balance:
  `net_down=+0.000N`, `required_scale=1.000000`.
- `sim/runtime/hydrodynamics_runtime_setup.py` no longer appears in the top 35
  hotspot inventory.
