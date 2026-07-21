# 2026-06-04 MAVLink Command Link Split

Goal: split low-level MAVLink command-link send primitives out of
`bridge/sitl_transport.py` without changing arm, mode, RC override, telemetry,
or plant-input semantics.

## Actions

- Added `sim/transport/mavlink_command_link.py`.
- Moved disabled command endpoint detection into `command_endpoint_disabled`.
- Moved command-link connection creation into `MavlinkCommandLink.connect`.
- Moved GCS heartbeat send throttling into `MavlinkCommandLink`.
- Moved low-level `RC_CHANNELS_OVERRIDE` packet send into
  `MavlinkCommandLink.send_rc_channels_override`.
- Moved low-level arm/disarm `MAV_CMD_COMPONENT_ARM_DISARM` send into
  `MavlinkCommandLink.send_arm_disarm`.
- Updated `bridge/sitl_transport.py` to keep the existing public methods and
  command policy, while delegating low-level sends to command-link helpers.

## What did not change

- RC override normalization and timing policy stayed in `SitlTransport`.
- Arm/disarm retry, force-arm fallback, pending-arm service, and confirmation
  behavior stayed in `SitlTransport`.
- Mode map and mode command policy stayed in `SitlTransport`.
- `SERVO_OUTPUT_RAW` remains telemetry for controller parity.
- Closed-loop plant input still uses raw ArduSub JSON servo packets.
- No ArduPilot files were edited.
- No MuJoCo physics coefficient, actuator remap, or PWM correction was changed.

## Validation

- `python3 -m py_compile` passed for `mavlink_command_link.py`,
  `sim/transport/__init__.py`, and `bridge/sitl_transport.py`.
- Pure command-link smoke verified:
  - disabled endpoint detection;
  - GCS heartbeat throttling and force send;
  - MAVLink2 18-channel RC override send;
  - MAVLink1 8-channel RC override fallback;
  - arm/disarm command payload and force magic.
- Source contract audit was rerun after the split: 10 PASS, 5 WARN, 0 FAIL.
- Plant-input gate checks were rerun after the split:
  - missing CSV failed with `csv_data_rows_lt_1`;
  - known non-neutral `full_mujoco_rcout.csv` passed with 42 data rows.
- Refactor inventory now reports `bridge/sitl_transport.py` at 3000 LOC and
  498 branches.

## Open contract work

Continue reducing `SitlTransport` by extracting policy objects that do not
change runtime behavior:

1. Sensor replay clock and immediate JSON reply scheduler.
2. MAVLink message-interval request scheduler.
3. ExternalNav/VPD output adapter.
4. RC override stream policy backed by `sim/contracts/rc.py`.
