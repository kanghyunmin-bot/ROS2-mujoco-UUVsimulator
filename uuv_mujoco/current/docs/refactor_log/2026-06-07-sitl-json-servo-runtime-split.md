# SITL JSON Servo Runtime Split

Date: 2026-06-07

## Scope

Split `bridge/sitl_json_servo_runtime.py` into focused JSON servo polling
helpers without changing `SitlTransport` method bindings or the JSON
servo/MAVLink/replay plant-input contract.

## Files

- `bridge/sitl_json_servo_runtime.py`
  - Compatibility export surface.
- `bridge/sitl_json_servo_endpoint.py`
  - JSON servo endpoint polling and latest-PWM dispatch.
- `bridge/sitl_json_servo_packet_state.py`
  - Frame-rate/frame-count bookkeeping, endpoint discovery/change tracking, and
    JSON-fallback versus MAVLink-active plant-source policy.
- `bridge/sitl_json_servo_warnings.py`
  - Missing and stale JSON servo endpoint warnings.
- `bridge/sitl_json_servo_timeout.py`
  - Plant replay external-servo timeout release and neutral replay timeout
    frame.
- `bridge/sitl_json_servo_poll_loop.py`
  - Top-level command MAVLink, servo MAVLink, JSON servo, auto-ready, replay
    timeout, and neutral keepalive polling sequence.

## Contract Notes

- JSON servo packets still update frame-rate/frame-count and client state even
  when MAVLink servo is the active plant source.
- When MAVLink servo is active and JSON fallback is disabled, JSON servo packets
  are observed but do not drive the plant.
- In JSON fallback mode, all received JSON servo packets can trigger immediate
  sensor-replay replies, but only the latest PWM frame from the poll batch is
  sent to `_handle_pwm_values(..., source="json")`.
- Plant replay timeout still releases `_sitl_external_servo_override_until_wall`
  and sends neutral `replay_rcout_timeout` PWM.

## Verification

```text
sitl json servo runtime split smoke: PASS
python3 -m compileall -q uuv_mujoco/current uuv_control_gui.py: PASS
audit_code_contract_sources.py: fail=0 pass=11 warn=5
check_runtime_readiness_policy.py: PASS
verify_ardusub_thruster_contract.py --quiet: PASS
physics_contract_audit.py --simulate-s 0.05: PASS
git diff --check: PASS
refactor_inventory.py: bridge/sitl_json_servo_runtime.py removed from top 45
```
