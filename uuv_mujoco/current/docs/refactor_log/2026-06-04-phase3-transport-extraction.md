# 2026-06-04 Phase 3 Transport Extraction

Goal: start splitting `bridge/sitl_transport.py` without changing its public API
or the plant-input/controller-parity observation contract.

## Actions

- Added `sim/transport/json_servo.py` for ArduPilot JSON-SITL binary servo
  packet decoding.
- Added `sim/transport/mavlink_telemetry.py` for MAVLink telemetry type lists
  and `SERVO_OUTPUT_RAW` PWM extraction.
- Added `sim/transport/plant_command.py` for pure PWM activity classification.
- Updated `bridge/sitl_transport.py` to use the new transport helpers.
- Removed direct JSON servo `struct.unpack_from` use from `bridge/sitl_transport.py`.
- Removed direct `SERVO_OUTPUT_RAW -> servo{i}_raw` extraction from
  `bridge/sitl_transport.py`.
- Replaced inline active/non-neutral/all-min PWM tests with shared helpers.

## What did not change

- `SitlTransport` public methods and constructor stayed in place.
- Closed-loop plant input remains raw ArduSub JSON servo.
- MAVLink `SERVO_OUTPUT_RAW` remains telemetry/observation unless JSON servo
  fallback is explicitly disabled.
- No ArduPilot files were edited.
- No physics coefficient or thruster mapping was changed.

## Validation

- `python3 -m py_compile` passed for `sim/transport/*.py` and
  `bridge/sitl_transport.py`.
- Pure helper smoke decoded a synthetic 16-channel JSON servo packet.
- Pure helper smoke extracted 8 channels from a synthetic
  `SERVO_OUTPUT_RAW`-like message.
- Plant command helper smoke matched expected active/non-neutral/all-min
  classifications.
- Source contract audit still reports 10 PASS, 5 WARN, 0 FAIL.
- Plant-input gate still fails a known header-only CSV and passes a known
  non-neutral CSV.
- Inventory changed from `bridge/sitl_transport.py` 3199 LOC / 554 branches to
  3157 LOC / 544 branches.

## Open contract work

Split `SitlTransport` behind compatibility fields:

1. `JsonServoReceiver`: UDP socket ownership and packet receive loop.
2. `MavlinkTelemetryReceiver`: passive heartbeat and telemetry observation.
3. `MavlinkCommandLink`: arm/mode/RC override/message-interval commands.
4. `SensorReplayClock`: controller-parity replay time and immediate reply.

Do this one adapter at a time with compile and gate checks after each adapter.
