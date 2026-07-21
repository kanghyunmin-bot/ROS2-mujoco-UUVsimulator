# Audit Closed-Loop Contract Split

Date: 2026-06-07

Scope: active validation tooling under `uuv_mujoco/current/tools`.

## Changes

- Split `tools/audit_closed_loop_contract.py` into focused modules:
  - `tools/audit_closed_loop_params.py`
  - `tools/audit_closed_loop_profile.py`
  - `tools/audit_closed_loop_payload.py`
- Kept `tools/audit_closed_loop_contract.py` as the CLI facade and preserved
  existing helper import names where practical.
- Removed no contract data from the emitted JSON payload.

## Contract Notes

- Active runtime resolution still prefers `uuv_mujoco/current` and only reports
  `uuv_mujoco/v2.2` as a compatibility fallback.
- Watched real/SITL parameter comparison still covers the same ArduSub motor,
  RC, joystick, failsafe, ALT_HOLD, Bar30/EKF, INS, and VISO parameters.
- The tool still reports `real_vs_sitl_mismatches`, `missing_sitl_params`,
  active/inactive MuJoCo profile keys, and selected T200 thruster curve data.
- No controller parity observation point, JSON servo plant input contract,
  ArduPilot source, submodule pointer, PWM remap, or ALT_HOLD shim changed.

## Verification

```bash
python3 -m py_compile \
  uuv_mujoco/current/tools/audit_closed_loop_contract.py \
  uuv_mujoco/current/tools/audit_closed_loop_params.py \
  uuv_mujoco/current/tools/audit_closed_loop_profile.py \
  uuv_mujoco/current/tools/audit_closed_loop_payload.py
python3 uuv_mujoco/current/tools/audit_closed_loop_contract.py --help
python3 uuv_mujoco/current/tools/audit_closed_loop_contract.py \
  --json-out /private/tmp/uuv_audit_closed_loop_contract_after_split.json
python3 uuv_mujoco/current/tools/refactor_inventory.py --root uuv_mujoco/current --limit 35
```

Results:

- Parsed JSON payload before/after split was identical.
- The active runtime remained `uuv_mujoco/current`.
- `real_vs_sitl_mismatches` remained `1`; `missing_sitl_params` remained `0`.
- `tools/audit_closed_loop_contract.py` dropped from `337 LOC / 48` branches to
  `76 LOC / 2` branches and no longer appears in the top 35 hotspot inventory.
