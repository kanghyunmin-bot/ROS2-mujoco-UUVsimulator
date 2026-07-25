# 2026-06-04 MAVLink Message Interval Scheduler Split

Goal: split repeated MAVLink `SET_MESSAGE_INTERVAL` request code out of
`bridge/sitl_transport.py` without changing which messages are requested or
which streams are used for parity.

## Actions

- Added `sim/transport/mavlink_message_interval.py`.
- Moved MAVLink message interval command payload construction into
  `MavlinkMessageIntervalRequester`.
- Moved repeated AP telemetry message constant list into
  `DEFAULT_AP_TELEMETRY_MESSAGE_CONSTANTS`.
- Updated `bridge/sitl_transport.py` message-rate request methods to delegate:
  - servo-link `SERVO_OUTPUT_RAW`;
  - command-link `SERVO_OUTPUT_RAW`;
  - servo-link AP telemetry;
  - command-link AP telemetry.
- Kept existing target resolution and stream freshness policy in
  `SitlTransport`.

## What did not change

- `SERVO_OUTPUT_RAW` remains telemetry for controller parity.
- Closed-loop plant input still uses raw ArduSub JSON servo packets.
- Requested AP telemetry message set is unchanged.
- Message interval request periods are unchanged.
- No ArduPilot files were edited.
- No MuJoCo physics coefficient, actuator remap, or PWM correction was changed.

## Validation

- `python3 -m py_compile` passed for `mavlink_message_interval.py`,
  `sim/transport/__init__.py`, and `bridge/sitl_transport.py`.
- Pure requester smoke verified:
  - `SERVO_OUTPUT_RAW` interval payload;
  - request throttling by key;
  - named AP telemetry message requests;
  - missing message constants are skipped.
- Source contract audit was rerun after the split: 10 PASS, 5 WARN, 0 FAIL.
- Plant-input gate checks were rerun after the split:
  - missing CSV failed with `csv_data_rows_lt_1`;
  - known non-neutral `full_mujoco_rcout.csv` passed with 42 data rows.
- Refactor inventory now reports `bridge/sitl_transport.py` at 2926 LOC and
  486 branches.

## Open contract work

Continue extracting policy objects:

1. Sensor replay clock and immediate JSON reply scheduler.
2. ExternalNav/VPD output adapter.
3. RC override stream policy backed by `sim/contracts/rc.py`.
