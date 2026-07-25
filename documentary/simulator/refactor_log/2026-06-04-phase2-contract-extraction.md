# 2026-06-04 Phase 2 Contract Extraction

Goal: move real runtime contracts out of scattered bridge/debug helpers and
into `sim/contracts`, while preserving existing execution behavior.

## Actions

- Made `sim/contracts/baro.py` the canonical owner for Bar30/AP_Baro pressure
  formulas.
- Made `sim/contracts/rates.py` the canonical owner for real-robot surface
  rates.
- Converted `bridge/sitl_contract.py` into a compatibility shim.
- Migrated live `bridge/ros2_bridge.py` to import Bar30/rate contracts directly
  from `sim.contracts`.
- Migrated GUI RC constants to `sim.contracts`.
- Migrated GUI RC override sanitizing to the shared `sim.contracts.rc`
  sanitizer while preserving GUI `CHAN_NOCHANGE` padding behavior.
- Migrated `debug/controller_replay/replay_rc_override_ros2.py` to the shared
  RC sanitizer.
- Added `sim/runtime/readiness.py` as the dependency-free readiness vocabulary.
- Added `sim/validation/plant_input_gate.py` and
  `tools/check_plant_input_gate.py` to fail fast on header-only, disarmed, or
  neutral plant-input evidence.

## Validation

- `python3 -m py_compile` passed for migrated contract, bridge, GUI, debug, and
  validation modules.
- `sim.contracts` and `bridge.sitl_contract` return the same Bar30 class and
  rate dictionary through the compatibility shim.
- `tools/audit_code_contract_sources.py` passed with 10 PASS, 5 WARN, 0 FAIL.
- `tools/check_plant_input_gate.py` fails a known header-only
  `full_mujoco_rcout.csv` case.
- `tools/check_plant_input_gate.py` passes an existing non-neutral plant-input
  CSV with 42 data rows.

## Remaining warnings

- The top-level ArduPilot gitlink still differs from the checked-out ArduPilot
  commit.  Do not commit the submodule pointer.
- Local ArduSub 4.1.2 consumes RC override only through channel 16.
- `/mavros/imu/atm_pressure` remains excluded from parity and fitting targets.
- GUI helper import smoke outside a sourced ROS environment stops at missing
  `rclpy`; syntax compile passes.

## Open contract work

Split transport ownership: JSON servo receiver, JSON sensor sender, MAVLink
command sender, and SERVO_OUTPUT_RAW telemetry receiver should become separate
modules behind the current `SitlTransport` API.
