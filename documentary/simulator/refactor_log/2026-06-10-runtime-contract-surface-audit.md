# Runtime Contract Surface Audit

Date: 2026-06-10

## Scope

This pass does not change plant coefficients or controller behavior.  It adds
source-contract coverage for runtime surfaces that can invalidate controller
parity or plant replay even when the static plant contract looks reasonable.

Covered surfaces:

- sim-time sensor publish cadence versus wall-time transport polling
- MAVLink `SERVO_OUTPUT_RAW` request cadence
- raw 18-channel `RC_CHANNELS_OVERRIDE` forwarding
- `/mavros/rc/in` raw-frame mirror behavior
- plant-input ownership between SITL JSON SERVO and replay RCOU/PWM
- thruster force-conversion entry points after raw PWM reaches the plant
- dynamic MuJoCo ellipsoid `fluidcoef` setup and update path

## Files

- `tools/audit_code_contract_paths.py`
- `tools/audit_code_contract_checks.py`
- `tools/audit_code_contract_runtime_surface_ext.py`
- `sim/runtime/real_start_publisher.py`
- `sim/runtime/real_start_status_logging.py`
- `sim/runtime/real_start_status_publish.py`
- `gui/node_initial_depth_commands.py`
- `gui/node_initial_depth_state.py`
- `gui/readiness_arm_mode_gate.py`
- `gui/readiness_feedback_gate.py`
- `sim/runtime/descent_contract_guard.py`
- `sim/runtime/descent_contract_logic.py`
- `docs/architecture/ACTIVE_CONTRACT_WORKLIST.md`
- `docs/architecture/SPAGHETTI_AUDIT.md`

## Contract Result

The extended source audit produced:

```text
{"fail": 0, "pass": 15, "warn": 5}
```

New checks:

- `active_runtime_time_contract_sim_publish_wall_transport`
- `active_runtime_rc_override_forward_mirror_contract`
- `active_runtime_plant_input_raw_pwm_contract`
- `active_runtime_dynamic_fluidcoef_contract`

These gates make contract drift visible before running expensive plant replay,
HAN, or CFD calibration.

## Hotspot Splits

- `sim/runtime/real_start_publisher.py` now delegates JSON publish and one-shot
  OK/released logs to focused helpers.
- `gui/node_initial_depth_commands.py` now delegates pending/in-flight/release
  state transitions to `node_initial_depth_state.py`.
- `gui/readiness_arm_mode_gate.py` now delegates fresh vehicle/Bar30/IMU
  feedback reasons to `readiness_feedback_gate.py`.
- `sim/runtime/descent_contract_guard.py` now delegates descent cause
  classification and report formatting to `descent_contract_logic.py`.

These splits remove the real-start, initial-depth, ARM/MODE readiness, and
descent-contract files from the top structural-complexity inventory without
changing command, sensor, RC, or plant-input behavior.
