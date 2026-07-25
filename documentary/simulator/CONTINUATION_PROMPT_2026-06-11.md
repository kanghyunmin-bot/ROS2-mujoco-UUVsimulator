# Continuation Prompt

Use this prompt in a new Codex thread if the current context is too large.

```text
You are working in:
  /Users/kanghyunmin/Desktop/uuv_sim

Main runtime:
  /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2

Goal:
  Continue making uuv_mujoco/v2.2 maintainable and normally runnable, but do not
  force-refactor complex physics/communication code that is already functioning.
  Refactoring here means contract hardening: keep the final plant-replay target
  intact, where real RCOU/PWM history fed to the MuJoCo plant should reproduce
  real rosbag sensor/state trends as closely as possible.

Hard constraints:
  - Do not modify /Users/kanghyunmin/Desktop/uuv_sim/ardupilot.
  - Do not change the ArduPilot submodule pointer.
  - Keep edits inside uuv_mujoco/v2.2 unless explicitly told otherwise.
  - Do not hide mismatch with ALT_HOLD shims, PWM remaps, output offsets, or
    observation-point mixing.
  - Controller parity is real /mavros/rc/out vs SITL MAVLink SERVO_OUTPUT_RAW.
  - Plant replay is real RCOU/PWM -> MuJoCo plant -> simulated sensors.

First command to run:
  cd /Users/kanghyunmin/Desktop/uuv_sim
  env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 \
    uuv_mujoco/v2.2/tools/run_fast_contract_sanity.py \
    --include-host \
    --out-dir uuv_mujoco/v2.2/research_workspace/00_current_contract/fast_contract_sanity_next

Current green reference:
  uuv_mujoco/v2.2/research_workspace/00_current_contract/fast_contract_sanity_host_20260611

Read before editing:
  uuv_mujoco/v2.2/docs/HANDOFF_2026-06-11.md
  uuv_mujoco/v2.2/docs/refactor_log/2026-06-11-time-sensor-rc-thruster-fluid-contract-tightening.md

Preserve unless a focused test fails:
  sim/physics/fossen_residual_runtime_builder.py
  bridge/sitl_math.py
  sim/runtime/sitl_servo_binding.py
  bridge/ros2_state_sitl_velocity.py
  sim/transport/mavlink_message_interval_send.py
  sim/contracts/rc_value_rules.py
  bridge/sitl_rc_override_keepalive.py
  sim/physics/dynamic_fluidcoef_pattern_prepare.py
  sim/runtime/underwater_hydrostatic_runtime.py
  bridge/sitl_transport_model_vertical.py
  bridge/sitl_rc_override_send.py
  bridge/sitl_external_nav_vpd_pose.py

Next useful work:
  1. Keep fast contract sanity green.
  2. Live GUI/MuJoCo/SITL smoke: GUI start, MuJoCo ready, arm/mode ACK, MANUAL
     RC override movement latency, RCOU telemetry logging.
  3. Run real RCOU plant replay on the 90s bag baseline.
  4. Compare simulated sensors to real sensors by correlation/phase first and
     RMSE second.
  5. Tune dynamics only after input/output contracts are proven.
```
