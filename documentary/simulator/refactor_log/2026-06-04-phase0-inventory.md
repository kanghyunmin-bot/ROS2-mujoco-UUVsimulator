# 2026-06-04 Phase 0 Inventory

Goal: start the large refactor without changing runtime behavior.

## Actions

- Created ownership folders for `sim`, `han`, `experiments`, and `docs`.
- Recorded the current spaghetti hotspots.
- Recorded the current runtime contract.
- Added a contract wrapper path for the existing Bar30/AP_Baro contract.
- Added an inventory tool for repeatable code-size and complexity scans.
- Migrated `tools/real_start_state.py` away from local Bar30 formula
  duplication and onto the shared `sim.contracts` wrapper.
- Added `sim/contracts/rc.py`, `rates.py`, and `observability.py`.
- Migrated `debug/controller_parity_412/sensor_replay_sitl_json.py` to the
  shared pressure and RC override contract exports.
- Migrated `debug/controller_parity_412/audit_sensor_rate_contract.py` to shared
  sensor-rate exports.

## Explicitly not changed

- No ArduPilot files were edited.
- No ArduPilot submodule pointer was intentionally changed.
- No controller output shim was added.
- No PWM remap was added.
- No MuJoCo physics coefficient was changed in this phase.
- No live import path was changed from existing runtime entry points.

## Validation

- `python3 -m py_compile` passed for the new inventory tool, new contract
  package, migrated `tools/real_start_state.py`, and migrated
  `debug/controller_parity_412/sensor_replay_sitl_json.py` plus
  `audit_sensor_rate_contract.py`.
- `sim.contracts` import smoke produced the expected Bar30 rate and zero
  surface-depth conversion.
- `sensor_replay_sitl_json.py --help` still imports and renders the CLI.
- `tools/real_start_state.py` executed against both available real-feedback CSV
  families.  The 90s CSV at `t=0` has missing sensor fields, while the 20260401
  clean-window start produced pressure, attitude, velocity, and RC outputs.
- `tools/audit_code_contract_sources.py` wrote a source audit with 10 PASS, 5
  WARN, 0 FAIL.

## Source-audit warnings to preserve

- Top-level ArduPilot gitlink differs from the checked-out ArduPilot commit.
  Do not commit this pointer change.
- Local ArduSub 4.1.2 consumes RC override only through channel 16, even though
  the replay surface carries 18 channels.
- JSON `altitude` in v2.2 payload is compatibility/debug data for this firmware;
  pressure control is through JSON `position.z` into AP_Baro_SITL.
- `/mavros/imu/atm_pressure` is excluded from parity/fitting until its real
  semantics are proven.

## Open contract work

Phase 1 should extract contract data into stable modules and then update live
code to import those modules one contract at a time, with golden-master checks
after each step.
