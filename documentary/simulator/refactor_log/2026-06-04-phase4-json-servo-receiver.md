# 2026-06-04 Phase 4 JSON Servo Receiver Split

Goal: split JSON-SITL UDP socket ownership out of `bridge/sitl_transport.py`
without changing controller-parity or plant-input semantics.

## Actions

- Added `sim/transport/json_servo_receiver.py`.
- Moved JSON-SITL UDP socket creation, buffer sizing, nonblocking bind,
  packet receive, packet decode, send target bookkeeping, and close handling
  into `JsonServoReceiver`.
- Updated `bridge/sitl_transport.py` to delegate JSON servo UDP work through
  `JsonServoReceiver`.
- Kept legacy fields such as `self.sitl_sock`, `_sitl_client_addr`,
  `_sitl_send_target`, and `_sitl_send_counter` synchronized for compatibility
  with the existing bridge code and logs.
- Exported the receiver from `sim.transport`.

## What did not change

- Closed-loop plant input is still raw ArduSub JSON servo packets.
- Controller parity still compares real `/mavros/rc/out` against SITL MAVLink
  `SERVO_OUTPUT_RAW` telemetry.
- JSON servo fallback rules, immediate sensor replay reply, RC override, arm,
  mode, MAVLink telemetry, and sensor payload construction stayed in
  `SitlTransport`.
- No ArduPilot files were edited.
- No MuJoCo physics coefficient, actuator remap, or PWM correction was changed.

## Validation

- `python3 -m py_compile` passed for the new receiver, `sim/transport` exports,
  and `bridge/sitl_transport.py`.
- UDP loopback smoke decoded a synthetic 16-channel ArduPilot JSON servo packet
  through `JsonServoReceiver`.
- Source contract audit was rerun after the split: 10 PASS, 5 WARN, 0 FAIL.
- Plant-input gate checks were rerun after the split:
  - missing CSV failed with `csv_data_rows_lt_1`;
  - known non-neutral `full_mujoco_rcout.csv` passed with 42 data rows.
- Refactor inventory now reports `bridge/sitl_transport.py` at 3131 LOC and
  536 branches.

## Open contract work

Continue the transport split in small pieces:

1. Extract passive MAVLink `SERVO_OUTPUT_RAW` telemetry observation.
2. Extract MAVLink command link for arm/mode/RC override/message intervals.
3. Extract controller-parity sensor replay clock and immediate reply policy.
4. Keep all extracted objects behind the current `SitlTransport` constructor
   until the GUI/runtime call sites are stable.
