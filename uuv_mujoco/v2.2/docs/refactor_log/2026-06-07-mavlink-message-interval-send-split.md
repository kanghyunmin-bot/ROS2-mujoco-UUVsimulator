# MAVLink Message Interval Send Split

Date: 2026-06-07

Scope: `uuv_mujoco/current/sim/transport`

## Change

`MavlinkMessageIntervalRequester` now keeps per-stream throttle state and the
existing public request methods, while low-level MAVLink send mechanics are
split out:

- `mavlink_message_constants.py`: AP telemetry message constant names requested
  from ArduPilot.
- `mavlink_message_interval_send.py`: MAVLink definition lookup, message-id
  resolution, `requested_hz` to microsecond interval conversion, and
  `MAV_CMD_SET_MESSAGE_INTERVAL` `command_long_send` payload construction.
- `mavlink_message_interval.py`: compatibility API and per-key request
  throttling.

## Contract

This is behavior-neutral.  It preserves:

- `MavlinkMessageIntervalRequester`
- `DEFAULT_AP_TELEMETRY_MESSAGE_CONSTANTS`
- `request_servo_output_raw(...)`
- `request_named_messages(...)`
- private `_send_message_interval(...)` compatibility wrapper
- request throttle update only after at least one successful send
- `requested_hz` lower clamp at `0.5 Hz` and interval lower bound at `1 us`

It does not change ArduPilot, telemetry message IDs, SITL command links, RC
override behavior, plant input, or physics coefficients.

## Verification

```text
PYTHONPATH=/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current python3 <fake MAVLink interval smoke>
python3 -m compileall -q /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current /Users/kanghyunmin/Desktop/uuv_sim/uuv_control_gui.py
PYTHONPATH=/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools python3 /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools/audit_code_contract_sources.py --out-dir /private/tmp/uuv_source_audit_spaghetti_20260607_mavlink_interval_split
python3 /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools/check_runtime_readiness_policy.py
python3 /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools/verify_ardusub_thruster_contract.py --quiet
/Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools/physics_contract_audit.py --output-dir /private/tmp/uuv_physics_audit_spaghetti_20260607_mavlink_interval_split --simulate-s 0.05
git -C /Users/kanghyunmin/Desktop/uuv_sim diff --check
```

Result: fake MAVLink interval smoke `PASS`, source audit
`fail=0 pass=11 warn=5`, readiness `PASS`, thruster contract `OK`, physics
contract audit completed, and `mavlink_message_interval.py` is no longer in
the top 40 refactor inventory hotspot list.
