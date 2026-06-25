# SITL MAVLink Request Policy Split

Date: 2026-06-07

## Scope

Split the SITL MAVLink stream-request policy out of
`bridge/sitl_mavlink_requests.py` without changing the public method names used
by `SitlTransport`.

## Files

- `bridge/sitl_mavlink_requests.py`
  - Compatibility facade that keeps `_request_sitl_mavlink_servo_stream`,
    `_request_command_servo_telemetry_stream`,
    `_request_sitl_mavlink_ap_telemetry_stream`, and
    `_request_command_ap_telemetry_stream`.
- `bridge/sitl_mavlink_request_targets.py`
  - Shared target resolution and stale/fresh request-period selection.
- `bridge/sitl_mavlink_request_servo.py`
  - `SERVO_OUTPUT_RAW` request policy for servo and command MAVLink links.
- `bridge/sitl_mavlink_request_ap.py`
  - AP sensor/attitude telemetry request policy for servo and command MAVLink
    links.

## Contract Notes

- Servo-link `SERVO_OUTPUT_RAW` requests preserve the direct SITL heartbeat
  target rule and only use explicit system/component IDs when both are positive.
- Servo-link AP telemetry requests preserve the previous
  `_resolve_mav_target(_sitl_mav)` behavior.
- Command-link requests preserve `_resolve_mav_target(_sitl_cmd_mav)`.
- PWM frame acceptance and plant input neutralization remain in
  `bridge/sitl_pwm_runtime.py`.

## Verification

```text
sitl mavlink request split smoke: PASS
python3 -m compileall -q uuv_mujoco/current uuv_control_gui.py: PASS
audit_code_contract_sources.py: fail=0 pass=11 warn=5
check_runtime_readiness_policy.py: PASS
verify_ardusub_thruster_contract.py --quiet: PASS
physics_contract_audit.py --simulate-s 0.05: PASS
git diff --check: PASS
refactor_inventory.py: bridge/sitl_mavlink_requests.py removed from top 45
```
