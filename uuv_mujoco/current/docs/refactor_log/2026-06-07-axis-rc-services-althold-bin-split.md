# Axis RC Services And ALT_HOLD BIN Split

Date: 2026-06-07

Scope:

- `tools/axis_rc_services.py`
- `tools/althold_contract_bin.py`

What changed:

- Split axis RC neutral publish/spin helpers into
  `tools/axis_rc_service_spin.py`.
- Split MAVROS-like stack readiness into `tools/axis_rc_stack_wait.py`.
- Split SetMode service retry into `tools/axis_rc_mode_service.py`.
- Split arming service retry into `tools/axis_rc_arm_service.py`.
- Split trigger service calls into `tools/axis_rc_trigger_service.py`.
- Split ALT_HOLD BIN stream schema, path selection, message collection, BIN
  array conversion, and signal math into focused modules.
- Kept `tools/axis_rc_services.py` and `tools/althold_contract_bin.py` as
  compatibility facades for existing CLI imports.

Contract notes:

- Axis RC service calls still publish neutral control while waiting for service
  completion and state confirmation.
- `wait_for_stack(..., require_manual_input=True)` still fails with the RC
  override command-link readiness message when manual input is absent.
- `call_set_mode()` still treats service acceptance and reported vehicle mode
  as separate conditions.
- `call_arm()` still treats service acceptance and reported armed state as
  separate conditions.
- DataFlash BIN `TimeUS` values are still normalized against the earliest
  stream or mode timestamp.
- Vertical plant command from RCOU still uses down-positive MuJoCo convention
  with C5/C8 inverted and C6/C7 positive.
- ArduPilot source and submodule pointer were not modified.

Focused verification:

```text
python3 -m compileall -q \
  uuv_mujoco/current/tools/axis_rc_services.py \
  uuv_mujoco/current/tools/axis_rc_service_spin.py \
  uuv_mujoco/current/tools/axis_rc_stack_wait.py \
  uuv_mujoco/current/tools/axis_rc_mode_service.py \
  uuv_mujoco/current/tools/axis_rc_arm_service.py \
  uuv_mujoco/current/tools/axis_rc_trigger_service.py \
  uuv_mujoco/current/tools/axis_rc_node_services.py

axis_rc_services_split_smoke PASS

python3 -m compileall -q \
  uuv_mujoco/current/tools/althold_contract_bin.py \
  uuv_mujoco/current/tools/althold_contract_bin_schema.py \
  uuv_mujoco/current/tools/althold_contract_bin_paths.py \
  uuv_mujoco/current/tools/althold_contract_bin_streams.py \
  uuv_mujoco/current/tools/althold_contract_bin_reader.py \
  uuv_mujoco/current/tools/althold_contract_signal_math.py \
  uuv_mujoco/current/tools/analyze_althold_contract.py \
  uuv_mujoco/current/tools/althold_contract_signals.py

althold_contract_bin_split_smoke PASS
```

Full verification:

```text
python3 -m compileall -q uuv_mujoco/current uuv_control_gui.py
audit_code_contract_sources.py -> {"fail": 0, "pass": 11, "warn": 5}
check_runtime_readiness_policy.py -> runtime_readiness_policy=PASS
verify_ardusub_thruster_contract.py --quiet -> OK
physics_contract_audit.py --simulate-s 0.05 -> required_scale=1.000000
```

Inventory result:

- `tools/axis_rc_services.py` is no longer in the top hotspot list.
- `tools/althold_contract_bin.py` is no longer in the top hotspot list.
