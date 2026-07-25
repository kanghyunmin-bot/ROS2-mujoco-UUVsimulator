# RC Replay GUI Mixin Split

Date: 2026-06-07

Scope: active GUI runtime under `sim/current/gui`.

## Change

- `gui/replay_mixin.py` is now a compatibility facade.
- Thread-safe replay status updates live in `gui/replay_status.py`.
- Timeline, slider, seek, and replay-rate logic live in `gui/replay_timeline.py`.
- Browse/load/start/pause/stop user actions live in `gui/replay_controls.py`.
- The playback worker loop lives in `gui/replay_worker.py`.

## Preserved Contract

- `RcReplayMixin` keeps every historical method name consumed by layout,
  control, auto-tune, and simulator-stack mixins.
- RC replay still publishes the loaded bag's RC override channel frames through
  `node.publish_rc_channels()`.
- Stop/failure/finish paths still call `node.publish_rc_release()`.
- No ArduSub, RC mapping, MAVLink telemetry, JSON servo, or plant-input contract
  behavior was changed.

## Validation

- Replay modules and `gui/app.py` pass `py_compile`.
- GUI replay import-surface smoke passed under the same ROS Python environment
  used by the GUI launcher.
- Full runtime compile passed.
- Source contract audit after the split reports `fail=0`, `pass=11`, `warn=5`.
- Runtime readiness and thruster contract gates pass.
- Ubuntu compatibility gate reports `fail=0`, `pass=16`, `warn=2`; the warnings
  are Docker daemon unavailable and ROS2 not sourced in the current shell.
- Refactor inventory shows `gui/replay_mixin.py` reduced from `322 LOC / 52`
  branches to `60 LOC / 0` branches and removed from the top 35 hotspot list.
