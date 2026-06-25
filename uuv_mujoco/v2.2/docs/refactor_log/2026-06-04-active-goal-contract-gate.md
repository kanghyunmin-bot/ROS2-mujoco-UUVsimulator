# 2026-06-04 Active Goal and Plant-Input Gate Correction

Goal: stop treating refactor steps as the objective.  The objective is the
full MuJoCo closed-loop contract:

```text
controller parity:
  real /mavros/rc/out
  vs SITL MAVLink SERVO_OUTPUT_RAW telemetry

plant input:
  raw ArduSub JSON servo packet
  to MuJoCo thrusters
```

## Actions

- Updated `docs/architecture/REFACTOR_MASTER_PLAN.md` so contract gates are the
  acceptance criteria.
- Updated `docs/architecture/ACTIVE_CONTRACT_WORKLIST.md` into an active
  contract worklist ordered by runtime contract risk.
- Updated `docs/contracts/CURRENT_RUNTIME_CONTRACT.md` to require the
  plant-input gate before overlay generation.
- Integrated `sim.validation.plant_input_gate.evaluate_plant_input_gate()` into
  `debug/controller_parity_412/run_full_mujoco_controller_parity.py`.
- The full MuJoCo controller-parity runner now writes:
  - `plant_input_gate.json`
  - `plant_input_gate.md`
- The runner now records the same gate verdict in
  `full_mujoco_controller_parity_manifest.json` as
  `plant_input_gate_result`.
- The runner raises before raw/auto overlay generation when the plant-input
  evidence is missing, header-only, neutral-only, or disarmed.
- `debug/controller_parity_412/run_full_v22_parity_after_conflicts.sh` now
  treats missing/failed plant-input gate artifacts as wrapper failures after a
  successful runner invocation.
- GUI command readiness now subscribes to
  `/uuv_mujoco/sitl/mavlink_telemetry_status`, maps it through
  `sim.runtime.readiness.RuntimeReadiness`, and blocks `READY`/arm-mode send
  attempts when the internal sim bridge lacks a fresh SITL MAVLink heartbeat.
- Added `tools/check_runtime_readiness_policy.py` so the GUI READY policy can
  be verified without launching ROS or Tk.

## What did not change

- ArduPilot source and submodule pointer were not edited.
- Closed-loop plant input remains raw ArduSub JSON servo.
- Controller parity observation remains real `/mavros/rc/out` vs SITL MAVLink
  `SERVO_OUTPUT_RAW`.
- No MuJoCo physics coefficient, actuator remap, or PWM correction was changed.

## Validation

- `python3 -m py_compile` passed for the runner, gate CLI, and validation
  module.
- Runner dry-run manifest includes plant-input gate output paths.
- Plant-input gate report smoke writes JSON and Markdown.
- Existing non-neutral full MuJoCo RCOU CSV plus runtime/probe logs passes the
  gate with 42 rows, 42 non-neutral rows, and zero disarmed/neutral signatures.
- Source contract audit was rerun after the correction: 10 PASS, 5 WARN, 0 FAIL.
