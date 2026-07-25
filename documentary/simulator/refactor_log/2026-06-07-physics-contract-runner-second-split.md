# Physics Contract Runner Second Split

Date: 2026-06-07

## Scope

Reduce the static physics contract runner by moving orchestration substeps into
focused modules without changing audit calculations or output shape.

## Changed Files

- `tools/physics_contract_profile_runtime.py`: simulation profile loading and
  audit-only CoB/restoring override application.
- `tools/physics_contract_start_depth.py`: start-depth candidate construction
  and automatic Bar30/base depth selection.
- `tools/physics_contract_neutral_runner.py`: neutral open-plant simulation
  dispatch.
- `tools/physics_contract_audit_report.py`: JSON report dictionary assembly.
- `tools/physics_contract_runner.py`: reduced to audit orchestration.

## Contract Notes

- Static force-balance CSV/JSON field names are unchanged.
- `--cob-x-offset`, `--cob-z-offset`, and `--cob-torque-scale` remain
  audit-only overrides.
- The MuJoCo runtime Python is required for executing the real audit because
  the macOS system Python does not provide `mujoco`.

## Validation

```text
python3 -m compileall -q sim/current/tools/physics_contract_*.py
/Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python \
  sim/current/tools/physics_contract_audit.py \
  --output-dir /private/tmp/uuv_physics_contract_after_runtime_hydrostatic_splits \
  --simulate-s 0
```

Observed status:

```text
physics contract audit: PASS
vehicle_mass_kg: 15.000
auto_bar30_depth_m: 0.600
scene_default net_down: +0.000 N
auto_fully_wet net_down: +0.000 N
```
