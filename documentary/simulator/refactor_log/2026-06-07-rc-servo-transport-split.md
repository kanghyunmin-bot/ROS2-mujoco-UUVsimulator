# RC and Servo Transport Split

Date: 2026-06-07

Scope: active runtime under `sim/current`, backed by
`uuv_mujoco/v2.2`.

## Changes

- Split SITL/plant-replay PWM servo runtime ownership:
  - `sim/runtime/sitl_servo_pwm.py`
  - `sim/runtime/sitl_servo_state.py`
  - `sim/runtime/sitl_servo_binding.py`
- Split JSON-SITL servo UDP receiver ownership:
  - `sim/transport/json_servo_endpoint.py`
  - `sim/transport/json_servo_receiver_config.py`
  - `sim/transport/json_servo_receiver_io.py`
- Split MAVLink servo telemetry handlers:
  - `bridge/sitl_mavlink_servo_callback.py`
  - `bridge/sitl_mavlink_servo_heartbeat.py`
  - `bridge/sitl_mavlink_servo_output.py`
- Split RC override forwarding ownership:
  - `bridge/sitl_rc_override_core.py`
  - `bridge/sitl_rc_override_send.py`
  - `bridge/sitl_rc_override_keepalive.py`
  - `bridge/sitl_rc_override_warn.py`

## Verification

Additional smoke checks passed:

- SITL servo runtime preserves PWM-to-normalized conversion, channel sign
  mapping, scale clamp, stale target clearing, mapping labels, and bridge
  handler binding.
- JSON servo receiver preserves UDP bind/reuse, invalid packet drop, valid
  JSON-SITL servo packet decoding, client/default send target selection,
  send telemetry, and close behavior.
- MAVLink servo handlers preserve HEARTBEAT target filtering, stream request
  triggers, `SERVO_OUTPUT_RAW` PWM extraction, callback invocation, wait warning,
  and callback error throttle.
- RC override forwarding preserves peer wait failure behavior, heartbeat before
  override send, 18-channel value storage, sensor-replay frame reset, and
  neutral keepalive holdoff after external RC override.

## Notes

- Controller parity observation remains telemetry-to-telemetry:
  real `/mavros/rc/out` versus SITL MAVLink `SERVO_OUTPUT_RAW`.
- Plant input remains raw ArduSub JSON servo or explicit plant-replay RCOU,
  not low-rate telemetry reshaped to hide mismatch.
- This pass does not change RC mapping, PWM normalization, neutral keepalive
  intervals, command retry timing, or ArduPilot source.
