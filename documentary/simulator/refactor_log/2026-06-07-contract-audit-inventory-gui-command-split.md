# Contract Audit, Inventory, And GUI Command Split

Date: 2026-06-07

Scope: active runtime `sim/current`, backed by `uuv_mujoco/v2.2`.

## Change

Source-contract audit builders were split by contract surface:

- `tools/audit_code_contract_firmware_json_servo.py`
- `tools/audit_code_contract_firmware_json_sensor.py`
- `tools/audit_code_contract_firmware_baro.py`
- `tools/audit_code_contract_firmware_servo_output.py`
- `tools/audit_code_contract_runtime_baro.py`
- `tools/audit_code_contract_runtime_json_altitude.py`
- `tools/audit_code_contract_runtime_static_pressure.py`
- `tools/audit_code_contract_runtime_atm_pressure.py`

`refactor_inventory.py` now ranks by a structural complexity score instead of
raw LOC.  Data-only modules no longer dominate the hotspot table.

GUI process and command helpers were split while preserving public method names:

- `gui/process_env.py`
- `gui/process_scan.py`
- `gui/process_termination.py`
- `gui/node_mode_request_steps.py`
- `gui/node_arm_request_steps.py`

## Verification

Commands run:

```text
python3 -m compileall -q sim/current uuv_control_gui.py
git diff --check
python3 sim/current/tools/audit_code_contract_sources.py
python3 sim/current/tools/check_runtime_readiness_policy.py
python3 sim/current/tools/verify_ardusub_thruster_contract.py --quiet
/Users/kanghyunmin/.venvs/mujoco311/bin/python sim/current/tools/physics_contract_audit.py --simulate-s 0.05
python3 sim/current/tools/refactor_inventory.py --root sim/current --format markdown --limit 20
```

Results:

- Python compile: PASS.
- `git diff --check`: PASS.
- Source contract audit: `fail=0`, `pass=11`, `warn=5`.
- Runtime readiness policy: PASS.
- ArduSub thruster contract: OK.
- Static physics audit remains neutral: `net_down=+0.000N`,
  `required_scale=1.000000`, neutral drift about `+0.00001m`.
- `process_common_mixin.py`, `node_mode_commands.py`,
  `node_arm_commands.py`, and the two large source-contract builders dropped
  out of the top hotspot list.

## Contract Notes

No ArduPilot source, submodule pointer, PWM mapping, RCOU observation point, or
plant physics coefficient was changed.  This is ownership cleanup around
contract checks, GUI command plumbing, and the refactor prioritization metric.
