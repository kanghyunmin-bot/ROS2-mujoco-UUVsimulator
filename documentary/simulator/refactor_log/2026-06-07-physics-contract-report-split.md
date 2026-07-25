# Physics Contract Report Split

## Scope

Split static physics contract audit report output without changing force-balance
calculation, CSV/JSON schema, console formatting, MuJoCo scene state, or
hydrostatic parameters.

## Files

- `tools/physics_contract_report.py`: compatibility exports for
  `print_physics_contract_report()` and
  `write_static_force_balance_outputs()`.
- `tools/physics_contract_report_io.py`: `static_force_balance.csv` and
  `static_force_balance.json` writing.
- `tools/physics_contract_report_console.py`: console report orchestration.
- `tools/physics_contract_report_sections.py`: compatibility exports for
  individual report sections.
- `tools/physics_contract_report_body.py`: mass, CoM, and inertia output.
- `tools/physics_contract_report_hydro.py`: hydrostatic and start-depth output.
- `tools/physics_contract_report_balance.py`: force-balance,
  neutral-open-plant, and output-path sections.

## Contract

- `physics_contract_runner_outputs.py` import surface is unchanged.
- `static_force_balance.csv` field order still follows
  `ForceBalance.__dataclass_fields__`.
- `static_force_balance.json` still receives the `csv` path in the same report
  object.
- Console wording and numeric precision are preserved.
- No controller parity, plant input, sensor, or ArduPilot path was modified.

## Verification

```bash
python3 -m compileall -q \
  uuv_mujoco/v2.2/tools/physics_contract_report.py \
  uuv_mujoco/v2.2/tools/physics_contract_report_io.py \
  uuv_mujoco/v2.2/tools/physics_contract_report_console.py \
  uuv_mujoco/v2.2/tools/physics_contract_report_sections.py \
  uuv_mujoco/v2.2/tools/physics_contract_report_body.py \
  uuv_mujoco/v2.2/tools/physics_contract_report_hydro.py \
  uuv_mujoco/v2.2/tools/physics_contract_report_balance.py \
  uuv_mujoco/v2.2/tools/physics_contract_runner_outputs.py

/Users/kanghyunmin/.venvs/mujoco311/bin/python \
  uuv_mujoco/v2.2/tools/physics_contract_audit.py --simulate-s 0.05

python3 -m compileall -q sim/current uuv_control_gui.py
git diff --check
python3 sim/current/tools/audit_code_contract_sources.py
python3 sim/current/tools/check_runtime_readiness_policy.py
python3 sim/current/tools/verify_ardusub_thruster_contract.py --quiet
/Users/kanghyunmin/.venvs/mujoco311/bin/python \
  sim/current/tools/physics_contract_audit.py --simulate-s 0.05
```

The audit still reports `net_down=+0.000N`,
`required_scale=1.000000`, and neutral open-plant drift around `+0.00001m`.
