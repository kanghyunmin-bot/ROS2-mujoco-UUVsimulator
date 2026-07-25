# SITL MAVLink polling split

Date: 2026-06-07

## Scope

- Added focused MAVLink polling modules:
  - `bridge/sitl_mavlink_servo_handlers.py`
  - `bridge/sitl_mavlink_servo_polling.py`
  - `bridge/sitl_mavlink_command_polling.py`
- Kept `bridge/sitl_mavlink_polling.py` as the compatibility export surface
  consumed by `bridge/sitl_mavlink_runtime.py`.

## Contract

The split preserves `_poll_servo_mavlink()`, `_poll_command_mavlink()`, and
`_handle_servo_link_heartbeat()` bindings.  `SERVO_OUTPUT_RAW` telemetry
callback behavior and JSON-servo fallback gating are unchanged.

## Verification

```text
python3 -m compileall -q sim/current/bridge/sitl_mavlink_polling.py sim/current/bridge/sitl_mavlink_servo_handlers.py sim/current/bridge/sitl_mavlink_servo_polling.py sim/current/bridge/sitl_mavlink_command_polling.py
PYTHONPATH=sim/current python3 - <<'PY'
from bridge import sitl_mavlink_polling, sitl_mavlink_runtime
required = ['_handle_servo_link_heartbeat', '_poll_command_mavlink', '_poll_servo_mavlink']
missing = [name for name in required if not hasattr(sitl_mavlink_polling, name) or not hasattr(sitl_mavlink_runtime, name)]
print({'missing': missing})
PY
```

Smoke result: `missing=[]`.
