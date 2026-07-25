# 2026-06-07 GUI Physics And Replay Controls Split

## Scope

- Split GUI physics parameter application into loading, persistence, and
  error-log helpers while preserving the existing compatibility entry points.
- Split GUI RC replay controls into bag browsing, load/clear controls, playback
  controls, pure replay formatting, pure RC padding, and replay worker helper
  modules.
- Keep controller/plant contract behavior unchanged: this pass only changes
  ownership boundaries, not RC override timing, PWM interpretation, SITL
  telemetry comparison, or MuJoCo force coefficients.

## Contract Notes

- `gui/physics_param_apply.py` remains the apply orchestration surface.
- `gui/physics_param_io.py` remains a compatibility import surface until
  callers move to the focused parse/status/load/persist modules.
- `gui/replay_controls.py` remains the GUI mixin import surface.
- `gui/rc_replay_loader.py` now lazy-loads ROS bag runtime dependencies so
  importing replay UI helpers does not require `rclpy`.
- `gui/replay_format.py` and `gui/gui_rc_padding.py` are pure helpers and can be
  used by replay code without importing GUI or ROS runtime modules.

## Deletion Candidates

These are not immediate delete targets:

- `run_urdf_full.py`: still referenced by GUI status detection, debug runners,
  logs, and docs.  Remove only after launch/debug references use
  `run_uuv_mujoco.py`.
- `gui/helpers.py`, `gui/config.py`, `gui/uuv_control_gui.py`: compatibility
  facades/wildcard import surfaces.  Remove only after callers import focused
  modules directly.
- `gui/physics_param_io.py` and `gui/replay_controls.py`: compatibility
  surfaces for active GUI mixins.
- Generated logs and debug outputs: cleanup/archive candidates, not code
  refactor deletes.

## Verification

- `python3 -m compileall -q sim/current uuv_control_gui.py`
- `git diff --check`
- `python3 sim/current/tools/audit_code_contract_sources.py`
- `python3 sim/current/tools/check_runtime_readiness_policy.py`
- `python3 sim/current/tools/verify_ardusub_thruster_contract.py --quiet`
- `/Users/kanghyunmin/.venvs/mujoco311/bin/python sim/current/tools/physics_contract_audit.py --simulate-s 0.05`
