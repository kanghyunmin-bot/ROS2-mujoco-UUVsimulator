# SITL RC Manual Setpoint Split

Date: 2026-06-07

## Scope

Behavior-neutral split of SITL pilot command helpers inside the active runtime
`uuv_mujoco/current`.  This does not change the ArduSub source, submodule
pointer, RC channel mapping, RC override normalization, MANUAL_CONTROL priming,
or GUIDED/raw setpoint payload values.

## Changed

- Added `bridge/sitl_rc_override_runtime.py` for MAVROS-style RC override
  forwarding, neutral RC keepalive, and RC override warning throttling.
- Added `bridge/sitl_manual_control_runtime.py` for MANUAL_CONTROL priming and
  send logic.
- Added `bridge/sitl_guided_setpoint_runtime.py` for body-velocity GUIDED
  setpoints and raw `SET_POSITION_TARGET_LOCAL_NED` forwarding.
- Reduced `bridge/sitl_rc_manual_runtime.py` to a compatibility facade consumed
  by `bridge/sitl_commanding.py` and `bridge/sitl_transport.py`.

## Contract

- RC override still normalizes through `sim.contracts.normalize_ardusub_rc_override`.
- Neutral RC keepalive still observes the external override holdoff.
- MANUAL_CONTROL still sends an initial neutral frame before held non-neutral
  joystick input when ArduSub has not been primed.
- GUIDED/raw setpoint values are forwarded with the same frame/sign conversion
  as before the split.

## Validation

```text
python3 -m py_compile \
  uuv_mujoco/current/bridge/sitl_rc_manual_runtime.py \
  uuv_mujoco/current/bridge/sitl_rc_override_runtime.py \
  uuv_mujoco/current/bridge/sitl_manual_control_runtime.py \
  uuv_mujoco/current/bridge/sitl_guided_setpoint_runtime.py \
  uuv_mujoco/current/bridge/sitl_commanding.py \
  uuv_mujoco/current/bridge/sitl_transport.py

PYTHONPATH=uuv_mujoco/current python3 - <<'PY'
from bridge import sitl_rc_manual_runtime as m
required = [
    "_normalize_rc_override_values",
    "_send_rc_channels_override",
    "send_rc_override",
    "_send_neutral_rc_keepalive",
    "send_manual_control",
    "_warn_rc_override_not_forwarded",
    "send_body_velocity_setpoint",
    "send_position_target_local_ned",
]
print({name: callable(getattr(m, name, None)) for name in required})
PY
```

Observed:

- Compile and compatibility import-surface checks passed.
- `bridge/sitl_rc_manual_runtime.py`: `291 LOC / 32` branches ->
  `24 LOC / 0` branches.
