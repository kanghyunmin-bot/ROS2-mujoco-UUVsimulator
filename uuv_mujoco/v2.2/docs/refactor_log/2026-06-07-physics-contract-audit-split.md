# Physics Contract Audit Split

Date: 2026-06-07

Scope: active runtime tooling under `uuv_mujoco/current/tools`.

## Change

- Split `tools/physics_contract_audit.py` into:
  - `physics_contract_types.py`
  - `physics_contract_model.py`
  - `physics_contract_report.py`
  - `physics_contract_runner.py`
  - `physics_contract_audit.py`
- Kept the original executable name as the CLI entry point.
- Moved MuJoCo/numpy imports into the real audit path rather than the argparse
  help path.

## Contract Notes

- The tool remains an audit-only path and still does not touch ArduPilot.
- No runtime mass, inertia, CoM, CoB, buoyancy, thruster, or controller
  parameter was changed by this refactor.
- The audit still writes `static_force_balance.csv` and
  `static_force_balance.json`.

## Validation

```text
python3 -m py_compile uuv_mujoco/current/tools/physics_contract_audit.py \
  uuv_mujoco/current/tools/physics_contract_types.py \
  uuv_mujoco/current/tools/physics_contract_model.py \
  uuv_mujoco/current/tools/physics_contract_report.py \
  uuv_mujoco/current/tools/physics_contract_runner.py
PYTHONPATH=uuv_mujoco/current/tools python3 uuv_mujoco/current/tools/physics_contract_audit.py --help
source ./.uuv_mujoco_env.sh && PYTHONPATH=uuv_mujoco/current/tools "$MJ311_PYTHON" \
  uuv_mujoco/current/tools/physics_contract_audit.py \
  --output-dir /private/tmp/uuv_physics_contract_split_check --simulate-s 0
python3 uuv_mujoco/current/tools/refactor_inventory.py --limit 25 --format markdown
```

Results:

- Compile passed.
- CLI help passed without importing MuJoCo.
- Static force-balance audit executed and wrote CSV/JSON.
- `tools/physics_contract_audit.py` dropped out of the top 25 hotspot list.
- The remaining model calculation module is `tools/physics_contract_model.py`
  at `389 LOC / 32` branches.
