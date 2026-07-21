# SITL Replay Row Parser Split

Date: 2026-06-07

Scope: `uuv_mujoco/current/bridge`

## Change

`bridge/sitl_replay_row_parsers.py` now preserves the compatibility import
surface only.  Row parsing responsibilities are split into focused modules:

- `sitl_replay_row_vectors.py`: tolerant CSV row vector extraction.
- `sitl_replay_sensor_row.py`: sensor replay frame rows.
- `sitl_replay_vpd_row.py`: native VPD event rows.

## Contract

This is a behavior-preserving refactor.  It does not change zero-order-hold
policy, sensor replay start policy, native VPD ownership, or real/sim parity
observation surfaces.

## Verification

```text
/Users/kanghyunmin/.venvs/mujoco311/bin/python <focused replay row parser smoke>
python3 -m compileall -q uuv_mujoco/current uuv_control_gui.py
git diff --check
```

Result: row parser split smoke `PASS`, compileall `PASS`, diff check `PASS`.
