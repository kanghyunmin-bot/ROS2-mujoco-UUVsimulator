# Contract Matrix And Transport Split

Date: 2026-06-11

## Scope

- Kept edits inside `uuv_mujoco/v2.2`.
- Did not modify ArduPilot source or intentionally change the submodule pointer.
- Preserved the observation boundary:
  - controller parity: real `/mavros/rc/out` vs SITL MAVLink `SERVO_OUTPUT_RAW`
  - closed-loop plant input: raw SITL JSON servo PWM
  - plant replay input: recorded RCOU/PWM

## Changes

- Split JSON servo UDP receiver runtime into:
  - `sim/transport/json_servo_receiver_state.py`
  - `sim/transport/json_servo_receiver_socket.py`
  - compatibility facade `sim/transport/json_servo_receiver.py`
- Split JSON servo receiver smoke into:
  - `tools/json_servo_receiver_smoke_fixture.py`
  - `tools/json_servo_receiver_smoke_cases.py`
  - runner `tools/check_json_servo_receiver.py`
- Split SITL JSON sender diagnostics into:
  - `bridge/sitl_json_sender_sample_log.py`
  - `bridge/sitl_json_sender_status_log.py`
  - compatibility facade `bridge/sitl_json_sender_diagnostics.py`
- Split MAVLink command send primitives into:
  - `sim/transport/mavlink_heartbeat_sender.py`
  - `sim/transport/mavlink_rc_override_sender.py`
  - `sim/transport/mavlink_arm_sender.py`
  - compatibility facade `sim/transport/mavlink_command_senders.py`
- Split SITL arm/mode send helpers into:
  - `bridge/sitl_arm_mode_arm_send.py`
  - `bridge/sitl_arm_mode_resolve.py`
  - `bridge/sitl_arm_mode_mode_send.py`
  - compatibility facade `bridge/sitl_arm_mode_send.py`
- Added `tools/audit_code_contract_matrix.py` and wired it into
  `tools/audit_code_contract_checks.py`.
- Split matrix data/evaluation helpers into:
  - `tools/audit_code_contract_matrix_domains.py`
  - `tools/audit_code_contract_matrix_eval.py`
  - `tools/audit_code_contract_matrix_issue.py`
- Split RC override frame value handling into:
  - `sim/contracts/rc_value_rules.py`
  - `sim/contracts/rc_frame_fill.py`
  - compatibility APIs `sanitize_primary_rc` and `normalize_ardusub_rc_override`
- Split replay RCOU plant-input callback into:
  - `bridge/ros2_replay_rcout_channels.py`
  - `bridge/ros2_replay_rcout_inject.py`
  - `bridge/ros2_replay_rcout_log.py`
  - compatibility facade `bridge/ros2_replay_rcout.py`
- Added `tools/check_ros2_replay_rcout.py` to verify replay RCOU injection
  ownership directly.

## Contract Matrix

`source_contract_matrix_gate` requires these source-audit domains to be
present before controller parity or plant replay tuning is treated as valid:

- `source_identity`
- `time_phase`
- `sensor_input_output`
- `rc_input_output`
- `controller_output_and_plant_input`
- `thruster_contract`
- `dynamic_ellipsoid_fluid`

This gate does not claim that the physical coefficients are tuned. It only
prevents refactors from silently dropping a required contract check.

## Validation

```text
compileall tools/bridge/sim/physics/gui: PASS
json_servo_receiver=PASS
ros2_sitl_command_override=PASS
runtime_readiness_policy=PASS
rc_frame_contract=PASS
ros2_replay_rcout=PASS
dynamic_fluidcoef_contract=PASS
ardusub_thruster_contract=OK
contract_source_audit_matrix: fail=0 pass=17 warn=6
closed_loop_contract: real_vs_sitl_mismatches={} missing_sitl_params=[]
closed_loop_contract: thruster_voltage=20.0 dynamic_fluidcoef.active=false
refactor_inventory: no new matrix or transport split file remains above score 110
```

## Known WARN Items

- `top_level_ardupilot_gitlink`: local ArduPilot checkout does not match the
  top-level recorded gitlink. Do not commit a submodule pointer change unless
  that is an intentional project decision.
- `rc_override_local_16_channel_limit`: this local ArduSub 4.1.2 code consumes
  override channels through `chan16_raw`; the current vehicle uses C1-C8, but
  the old "preserve 1..18" assumption is not true for this firmware.
- `active_runtime_json_altitude_field_is_compat_only`: JSON `altitude` is
  compatibility/debug data for this firmware, not the controller input contract.
- `active_runtime_atm_pressure_excluded_output_surface`: real bag
  `/mavros/imu/atm_pressure` semantics are not Pa-scale Bar30 pressure, so it is
  excluded from parity and plant replay fitting targets.
- `plant_replay_gate_safe_targets`: RCOU, Bar30, IMU, DVL x/y, and gyro are
  safe targets; DVL z and local-position estimator surfaces are not primary
  fitting targets yet.
