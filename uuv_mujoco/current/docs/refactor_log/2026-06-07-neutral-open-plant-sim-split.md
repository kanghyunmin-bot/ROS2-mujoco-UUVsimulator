# Neutral Open-Plant Simulation Split

Date: 2026-06-07

Scope: active runtime under `uuv_mujoco/current`, backed by the compatibility
directory `uuv_mujoco/v2.2`.

## What Changed

- Kept `tools/physics_contract_neutral_sim.py` as the public neutral-PWM
  open-plant simulation entry point.
- Added `tools/physics_contract_neutral_buoyancy.py` for body-component
  hydrostatic force and restoring-torque application.
- Added `tools/physics_contract_neutral_output.py` for CSV header/sample output
  and `NeutralSimSummary` construction.

## Contract Preserved

- `simulate_neutral_open_plant()` keeps the same call signature and return type.
- `physics_contract_model.py` still re-exports `simulate_neutral_open_plant()`.
- Static force-balance output CSV/JSON names and neutral open-plant summary
  fields are preserved.
- Body-component share fallback, CoB x-offset torque, slope-scaled submerged
  fraction, and hydrostatic restoring torque are preserved.

## Verification

```text
python3 -m compileall -q \
  uuv_mujoco/current/tools/physics_contract_neutral_sim.py \
  uuv_mujoco/current/tools/physics_contract_neutral_buoyancy.py \
  uuv_mujoco/current/tools/physics_contract_neutral_output.py \
  uuv_mujoco/current/tools/physics_contract_runner.py \
  uuv_mujoco/current/tools/physics_contract_model.py

/Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python \
  uuv_mujoco/current/tools/physics_contract_audit.py \
  --output-dir /private/tmp/uuv_physics_audit_spaghetti_20260607_neutral_split_static \
  --simulate-s 0

/Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python \
  uuv_mujoco/current/tools/physics_contract_audit.py \
  --output-dir /private/tmp/uuv_physics_audit_spaghetti_20260607_neutral_split_sim \
  --simulate-s 0.05
```

Result: static and nonzero neutral open-plant physics audits pass, and the old
`tools/physics_contract_neutral_sim.py` hotspot no longer appears in the top 35
hotspot inventory.
