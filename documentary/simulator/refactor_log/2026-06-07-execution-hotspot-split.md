# Execution Hotspot Split

Date: 2026-06-07

Scope:

- `gui/ros_logged_process_mixin.py`
- `gui/ros_logged_process_files.py`
- `gui/ros_logged_process_launcher.py`
- `gui/ros_logged_process_start.py`
- `gui/ros_logged_process_threads.py`
- `gui/ros_logged_process_tail.py`
- `gui/ros_logged_process_finish.py`
- `gui/ros_logged_process_watch.py`
- `gui/node_rc_publishers.py`
- `gui/node_rc_override_publishers.py`
- `gui/node_manual_control_publishers.py`
- `gui/node_ping360_publishers.py`
- `sim/runtime/simulation_step_runtime.py`
- `sim/runtime/simulation_step_direct.py`
- `sim/runtime/simulation_step_raw_pwm.py`

Intent:

- Reduce branch-heavy code on paths that matter for GUI Start, ROS process
  status, RC override publication, and one-step MuJoCo/SITL execution.
- Preserve public method names consumed by the GUI and runner.
- Keep behavior-neutral splits only: no RC remapping, PWM correction, ArduSub
  parameter changes, or physics coefficient changes.

Verification:

- Logged process watcher fake-process smoke:
  `ros logged process split smoke: PASS`
- One-step runtime fake-runtime smoke:
  `simulation step split smoke: PASS`
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

- `gui/ros_logged_process_mixin.py` no longer appears in the top hotspot list.
- `gui/node_rc_publishers.py` no longer appears in the top hotspot list.
- `sim/runtime/simulation_step_runtime.py` no longer appears in the top
  hotspot list.
- Remaining large entries are now dominated by config/data modules, diagnostics
  tools, and physics setup helpers rather than the GUI process/RC/step loop
  methods touched in this pass.
