# GUI Replay Timeline Split

Date: 2026-06-07

Scope: active runtime only, under `uuv_mujoco/current`.

## Why

`gui/replay_timeline.py` mixed four responsibilities:

- binary-searching replay samples by time,
- formatting and clamping replay position/rate,
- handling Tk slider mouse events,
- holding thread-safe seek requests for the replay worker.

That made GUI replay latency/status bugs harder to isolate because UI event
handling and replay timeline math lived in the same module.

## Changed

- Added `gui/replay_time_math.py` for pure replay sample index, label, event
  time, and rate normalization helpers.
- Added `gui/replay_slider_events.py` for Tk slider press/motion/release and
  value-change handlers.
- Added `gui/replay_seek_state.py` for seek request/consume state.
- Kept `gui/replay_timeline.py` as the compatibility import surface used by
  `gui/replay_mixin.py`.

No replay API names changed.  The mixin still exposes:

- `_rc_replay_sample_index_for_time`
- `_set_rc_replay_position`
- `_update_rc_replay_time_label`
- `_event_to_rc_replay_time`
- `_set_replay_slider_from_event`
- `_on_rc_replay_slider_changed`
- `_on_rc_replay_slider_press`
- `_on_rc_replay_slider_motion`
- `_on_rc_replay_slider_release`
- `_request_rc_replay_seek`
- `_consume_rc_replay_seek`
- `_rc_replay_rate`

## Validation

Focused smoke:

```text
replay_timeline_split_smoke PASS
```

Full gates:

```text
python3 -m compileall -q /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current /Users/kanghyunmin/Desktop/uuv_sim/uuv_control_gui.py
PYTHONPATH=/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools python3 /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools/audit_code_contract_sources.py --out-dir /private/tmp/uuv_source_audit_spaghetti_20260607_replay_timeline_split
python3 /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools/check_runtime_readiness_policy.py && python3 /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools/verify_ardusub_thruster_contract.py --quiet
/Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools/physics_contract_audit.py --output-dir /private/tmp/uuv_physics_audit_spaghetti_20260607_replay_timeline_split --simulate-s 0.05
```

Results:

```text
compileall PASS
source contract audit: {"fail": 0, "pass": 11, "warn": 5}
runtime_readiness_policy=PASS
[thruster-contract] OK
physics contract audit PASS, static force balance required_scale=1.000000
```

## Contract Notes

This is a structural refactor only.  It does not change RC override timing,
MAVLink telemetry observation points, plant input, thruster coefficients,
Bar30 pressure handling, or MuJoCo physics parameters.
