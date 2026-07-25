# Runtime Start-Depth, Telemetry, and GUI Start Split

Date: 2026-06-07

Scope:

- `sim/runtime/initial_depth_runtime.py`
- `sim/runtime/initial_depth_release.py`
- `sim/runtime/initial_depth_service.py`
- `sim/transport/mavlink_telemetry_dispatch.py`
- `sim/transport/mavlink_telemetry_handlers.py`
- `gui/sim_stack_launch_runtime.py`
- `gui/sim_stack_restart_runtime.py`
- `gui/sim_stack_start_guards.py`
- `gui/sim_stack_start_process.py`
- `gui/sim_stack_start_state.py`
- `gui/sim_stack_start_runtime.py`

Intent:

- Keep GUI Start, initial-depth release, and passive MAVLink telemetry dispatch
  small enough to audit during RC override/ready-state debugging.
- Preserve behavior and public entry points:
  `InitialDepthHoldRuntime.release()`,
  `install_initial_depth_hold_service()`,
  `observe_mavlink_telemetry()`, and `_start_sim_stack()`.
- Avoid any ArduPilot, PWM remap, controller shim, or physics coefficient
  change.

Contract notes:

- Initial-depth release order is preserved:
  mark released, reset release state, apply captured release velocity,
  `mj_forward`, publish one fresh sensor snapshot, and report the release.
- MAVLink HEARTBEAT is still observed before the target-source filter.  ATTITUDE,
  LOCAL_POSITION_NED, SCALED_PRESSURE*, RC_CHANNELS, EKF_STATUS_REPORT,
  VFR_HUD, RAW_IMU, and SCALED_IMU storage still requires the target-source
  predicate.
- GUI Start still blocks tracked/external running stacks before opening logs or
  spawning the process.  It still stops RC replay, disables RC override, and
  publishes RC release before launching.

Verification:

- `initial depth release split smoke: PASS`
- `mavlink telemetry dispatch split smoke: PASS`
- `sim stack start split smoke: PASS`
- `python3 -m compileall -q sim/current uuv_control_gui.py`
- `git diff --check`
- `python3 sim/current/tools/audit_code_contract_sources.py`
  -> `{"fail": 0, "pass": 11, "warn": 5}`
- `python3 sim/current/tools/check_runtime_readiness_policy.py`
  -> `runtime_readiness_policy=PASS`
- `python3 sim/current/tools/verify_ardusub_thruster_contract.py --quiet`
  -> `[thruster-contract] OK`
- `/Users/kanghyunmin/.venvs/mujoco311/bin/python
  sim/current/tools/physics_contract_audit.py --simulate-s 0.05`

Inventory effect:

- `sim/runtime/initial_depth_runtime.py` no longer appears in the top hotspot
  list.
- `sim/transport/mavlink_telemetry_dispatch.py` no longer appears in the top
  hotspot list.
- `gui/sim_stack_launch_runtime.py` no longer appears in the top hotspot list.
