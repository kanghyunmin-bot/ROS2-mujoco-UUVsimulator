# Physics Neutral Buoyancy Apply Split

## Scope

Split neutral open-plant buoyancy application without changing hydrostatic
coefficients, mass properties, start-depth policy, or MuJoCo scene data.

## Files

- `tools/physics_contract_neutral_buoyancy.py`: compatibility facade preserving
  existing imports.
- `tools/physics_contract_neutral_context.py`: context dataclass and builder.
- `tools/physics_contract_neutral_components.py`: per-body-component buoyancy
  force and submerged-fraction calculation.
- `tools/physics_contract_neutral_restoring.py`: roll/pitch hydrostatic
  restoring torque.
- `tools/physics_contract_neutral_apply.py`: force/torque accumulation and
  `data.xfrc_applied` mutation.

## Contract

- No physical coefficient changes.
- No controller-parity observation-surface changes.
- No ArduPilot source or submodule pointer changes.
- Existing `apply_neutral_buoyancy()` and `build_neutral_buoyancy_context()`
  import surface remains available through
  `tools/physics_contract_neutral_buoyancy.py`.

## Verification

```bash
python3 -m compileall -q \
  uuv_mujoco/v2.2/tools/physics_contract_neutral_buoyancy.py \
  uuv_mujoco/v2.2/tools/physics_contract_neutral_apply.py \
  uuv_mujoco/v2.2/tools/physics_contract_neutral_components.py \
  uuv_mujoco/v2.2/tools/physics_contract_neutral_context.py \
  uuv_mujoco/v2.2/tools/physics_contract_neutral_restoring.py \
  uuv_mujoco/v2.2/tools/physics_contract_neutral_sim.py

/Users/kanghyunmin/.venvs/mujoco311/bin/python \
  uuv_mujoco/v2.2/tools/physics_contract_audit.py --simulate-s 0.05

python3 -m compileall -q sim/current uuv_control_gui.py
git diff --check
python3 sim/current/tools/audit_code_contract_sources.py
python3 sim/current/tools/check_runtime_readiness_policy.py
python3 sim/current/tools/verify_ardusub_thruster_contract.py --quiet
```

Observed static audit result remains:

- `net_down=+0.000N`
- `required_scale=1.000000`
- neutral open-plant drift around `+0.00001m` for the 0.05s smoke run.
