# 2026-06-04 Phase 5 MAVLink Telemetry Observer Split

Goal: split passive MAVLink telemetry observation out of
`bridge/sitl_transport.py` while keeping command, arm, mode, and RC override
behavior unchanged.

## Actions

- Added `sim/transport/mavlink_telemetry_observer.py`.
- Moved passive MAVLink telemetry status accumulation into
  `MavlinkTelemetryObserver`.
- Kept `SitlTransport.mavlink_telemetry_status()`,
  `_mavlink_source_matches_target()`, and `_store_ap_mavlink_telemetry()` as
  compatibility wrappers.
- Exported `MavlinkTelemetryObserver` from `sim.transport`.
- Synchronized the observer endpoint after the default MAVLink endpoint is
  resolved in `_connect_sitl_mavlink()`.

## What did not change

- `SERVO_OUTPUT_RAW` remains a telemetry/observation stream for controller
  parity.
- Closed-loop plant input still uses raw ArduSub JSON servo packets unless the
  explicit plant replay override mode is active.
- Arm/disarm, mode change, RC override, heartbeat send, and message interval
  requests are still owned by `SitlTransport`.
- No ArduPilot files were edited.
- No MuJoCo physics coefficient, actuator remap, or PWM correction was changed.

## Validation

- `python3 -m py_compile` passed for the new observer, `sim/transport` exports,
  and `bridge/sitl_transport.py`.
- Pure observer smoke verified:
  - heartbeat storage;
  - target source filtering;
  - attitude field storage;
  - RC channel storage;
  - status age and endpoint reporting.
- Source contract audit was rerun after the split: 10 PASS, 5 WARN, 0 FAIL.
- Plant-input gate checks were rerun after the split:
  - missing CSV failed with `csv_data_rows_lt_1`;
  - known non-neutral `full_mujoco_rcout.csv` passed with 42 data rows.
- Refactor inventory now reports `bridge/sitl_transport.py` at 3020 LOC and
  498 branches.

## Open contract work

Continue transport split with the command side:

1. Extract MAVLink command link connection and GCS heartbeat send.
2. Extract arm/disarm and mode command services.
3. Extract RC override stream policy into a runtime adapter that consumes
   `sim/contracts/rc.py`.
4. Only after command/telemetry boundaries are stable, move sensor replay clock
   ownership out of `SitlTransport`.
