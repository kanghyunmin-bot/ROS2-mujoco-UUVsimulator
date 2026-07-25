# Code Contract Active Runtime Split

Date: 2026-06-07

Scope: `sim/current/tools/audit_code_contract_sources.py` and support
modules.

## Change

- Replaced the old monolithic source-contract audit entrypoint with a thin CLI.
- Added shared audit types, path/evidence helpers, and report writers.
- Split source checks by responsibility:
  - ArduPilot source identity and gitlink evidence.
  - JSON SITL servo/sensor, Bar30, and SERVO_OUTPUT_RAW contracts.
  - RC override channel/timeout/joystick mapping contracts.
  - Active runtime Bar30/static-pressure/unsafe-output-surface contracts.
  - Thruster final-PWM and plant replay gate checks.
- Changed generated report metadata from `v22_root` to
  `active_runtime_root`, while preserving `compat_v22_root` as compatibility
  backing-path metadata.

## Validation

```text
python3 -m py_compile \
  sim/current/tools/audit_code_contract_sources.py \
  sim/current/tools/audit_code_contract_types.py \
  sim/current/tools/audit_code_contract_common.py \
  sim/current/tools/audit_code_contract_checks.py \
  sim/current/tools/audit_code_contract_report.py

PYTHONPATH=sim/current/tools \
python3 sim/current/tools/audit_code_contract_sources.py \
  --out-dir /private/tmp/uuv_code_contract_group_split
```

Result:

```text
{"fail": 0, "pass": 10, "warn": 5}
```

The report now states:

```text
active_runtime_root: /Users/kanghyunmin/Desktop/uuv_sim/sim/current
compat_v22_root: /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2
```
