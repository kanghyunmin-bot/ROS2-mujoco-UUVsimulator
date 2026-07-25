# MAVLink Command Link Send Split

Date: 2026-06-07

Scope: active runtime under `sim/current`, backed by
`uuv_mujoco/v2.2`.

## Changes

- Split command endpoint disable policy into
  `sim/transport/mavlink_command_endpoint.py`.
- Split low-level heartbeat, RC override, and arm/disarm send primitives into
  `sim/transport/mavlink_command_senders.py`.
- Kept `sim/transport/mavlink_command_link.py` as the public stateful command
  link owner.

## Verification

```bash
python3 -m compileall -q \
  sim/current/sim/transport/mavlink_command_link.py \
  sim/current/sim/transport/mavlink_command_endpoint.py \
  sim/current/sim/transport/mavlink_command_senders.py \
  sim/current/sim/transport/__init__.py

PYTHONPATH=sim/current python3 - <<'PY'
# Fake MAVLink smoke:
# - disabled endpoints still normalize as before
# - heartbeat still throttles at 1.0 s
# - RC override still prefers 18 channels and falls back to 8 on TypeError
# - arm/disarm force magic remains 21196.0
PY
```

## Notes

- This pass does not change RC override timing, target-system selection, MAVLink
  endpoint defaults, or retry policy.  It only makes the command-link send
  path easier to instrument when debugging command latency.
