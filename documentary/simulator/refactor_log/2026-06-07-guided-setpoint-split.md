# GUIDED Setpoint Split

Date: 2026-06-07

Scope: `sim/current/bridge`

## Change

`bridge/sitl_guided_setpoint_runtime.py` now preserves compatibility exports
only.  Setpoint forwarding responsibilities are split into focused modules:

- `sitl_body_velocity_setpoint.py`: body FLU velocity commands converted to
  MAVLink BODY_NED setpoints.
- `sitl_local_ned_setpoint.py`: raw local-NED position-target forwarding.

## Contract

This preserves the existing ArduSub/MAVLink sign contract:

- forward maps to BODY_NED `vx`.
- left maps to `-vy`.
- up maps to `-vz`.
- yaw-rate maps to negative BODY_NED yaw-rate.
- raw LOCAL_NED payload forwarding remains unchanged.

## Verification

```text
/Users/kanghyunmin/.venvs/mujoco311/bin/python <focused GUIDED setpoint smoke>
python3 -m compileall -q sim/current uuv_control_gui.py
git diff --check
```

Result: GUIDED setpoint smoke `PASS`, compileall `PASS`, diff check `PASS`.
