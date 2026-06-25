# Runtime Flow, Hydrostatic, And Dynamic Fluidcoef Contract Split

Date: 2026-06-11

## Scope

- Kept edits inside `uuv_mujoco/v2.2`.
- Did not modify ArduPilot source or intentionally change the submodule pointer.
- Preserved the controller-parity and plant-input observation boundaries.

## Changes

- Split runtime hydrostatic setup into:
  - `sim/runtime/physics_runtime_hydro_config.py`
  - `sim/runtime/physics_runtime_mass_reference.py`
  - facade `sim/runtime/physics_runtime_hydrostatic.py`
- Added `tools/check_physics_runtime_hydrostatic.py` to verify:
  - thruster-performance max-force extraction
  - vehicle-subtree mass reference
  - base-body mass fallback
  - neutral-volume calculation from mass and fluid density
- Added `tools/audit_code_contract_runtime_flow_coupling.py`.
  This new source-contract check ties together the runtime order for:
  - ROS/transport spin
  - raw SITL JSON SERVO or replay RCOU application
  - thruster force update cadence
  - dynamic MuJoCo ellipsoid `fluidcoef` update
  - underwater wrench application
  - final `mj_step`
- Added `tools/check_dynamic_fluidcoef_contract.py` to verify:
  - axis-weighted load calculation
  - five-value MuJoCo `fluidcoef` order: blunt, slender, angular, Kutta, Magnus
  - sim-time update cadence for dynamic coefficient updates

## Contract Notes

- Current `current` profile still keeps `dynamic_fluidcoef.active=false`.
  The dynamic path is present and tested, but the clean plant-contract baseline
  remains the fixed five-coefficient MuJoCo ellipsoid profile.
- The source audit now explicitly covers:
  - time contract
  - sensor I/O snapshot contract
  - RC in/out forwarding and mirror contract
  - raw plant input contract
  - dynamic fluidcoef contract
  - integrated runtime flow contract

## Validation

```text
compileall targeted runtime/audit files: PASS
physics_runtime_hydrostatic=PASS
dynamic_fluidcoef_contract=PASS
contract_source_audit: fail=0 pass=17 warn=5
closed_loop_contract: real_vs_sitl_mismatches={} missing_sitl_params=[]
ros2_sitl_command_override=PASS
runtime_readiness_policy=PASS
rc_frame_contract=PASS
plant_input_gate sampled CSV: ok=true, data_rows=11, non_neutral_rows=11
ardusub_thruster_contract=OK
thruster_performance_curves=PASS
compileall tools/bridge/sim/physics/gui: PASS
```

## Current Inventory Impact

`sim/runtime/physics_runtime_hydrostatic.py` is no longer in the top 25
inventory. Remaining hotspots are concentrated around Ping360 STL IO,
JSON servo receive transport, SITL JSON diagnostics, roll-stability tooling,
and small bridge/validation utilities.
