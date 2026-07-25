# GUI Launch, Plant Gate, and Step Runtime Split

Date: 2026-06-07

Scope: active runtime under `sim/current`, backed by
`uuv_mujoco/v2.2`.

## Changes

- Split GUI simulator-stack launch ownership:
  - `gui/sim_stack_launch_command.py`
  - `gui/sim_stack_launch_logs.py`
  - `gui/sim_stack_launch_process.py`
  - `gui/sim_stack_launch_target.py`
- Removed unnecessary ROS2/rclpy coupling from the new launch target/process
  helpers by using standard-library `os`, `subprocess`, and `threading`.
- Split plant-input validation gate ownership:
  - `sim/validation/plant_input_gate_types.py`
  - `sim/validation/plant_input_gate_csv.py`
  - `sim/validation/plant_input_gate_logs.py`
  - `sim/validation/plant_input_gate_eval.py`
- Split per-step simulation runtime common physics and initial-depth hold
  release policy:
  - `sim/runtime/simulation_step_physics.py`
  - `sim/runtime/simulation_step_hold_release.py`

## Verification

```bash
python3 -m compileall -q \
  sim/current/gui/sim_stack_launch_runtime.py \
  sim/current/gui/sim_stack_launch_command.py \
  sim/current/gui/sim_stack_launch_logs.py \
  sim/current/gui/sim_stack_launch_process.py \
  sim/current/gui/sim_stack_launch_target.py \
  sim/current/sim/validation/plant_input_gate.py \
  sim/current/sim/validation/plant_input_gate_*.py \
  sim/current/sim/runtime/simulation_step_runtime.py \
  sim/current/sim/runtime/simulation_step_physics.py \
  sim/current/sim/runtime/simulation_step_hold_release.py
```

Additional smoke checks passed:

- GUI launch command helper preserves MAVROS surface args, direct-MAVLink arg,
  no-rebuild arg, initial-depth args, and normalized extra-arg ordering.
- Plant input gate still fails header-only CSVs, neutral-only plant input when
  non-neutral rows are required, disarmed JSON-servo log signatures, and neutral
  JSON-servo log signatures.
- Simulation step runtime preserves direct-command path order, raw-PWM SITL
  servo path order, paused-step behavior, ROS publish gating, QGC video publish,
  ALT_HOLD auto-release, and non-neutral servo auto-release.

## Notes

- This pass does not change RC override timing, command retry policy, plant
  input semantics, physics coefficients, or ArduPilot source.
- The goal is traceability: GUI Start failures, command launch arguments,
  plant-input gate failures, and one-step physics order are now separated
  enough to instrument independently.
