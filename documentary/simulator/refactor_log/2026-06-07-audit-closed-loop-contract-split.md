# Audit Closed-Loop Contract Split

Date: 2026-06-07

Scope: active validation tooling under `sim/current/tools`.

## Changes

- Split `tools/audit_closed_loop_contract.py` into focused modules:
  - `tools/audit_closed_loop_params.py`
  - `tools/audit_closed_loop_profile.py`
  - `tools/audit_closed_loop_payload.py`
- Kept `tools/audit_closed_loop_contract.py` as the CLI facade and preserved
  existing helper import names where practical.
- Removed no contract data from the emitted JSON payload.

## Contract Notes

- Active runtime resolution still prefers `sim/current` and only reports
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
  sim/current/tools/audit_closed_loop_contract.py \
  sim/current/tools/audit_closed_loop_params.py \
  sim/current/tools/audit_closed_loop_profile.py \
  sim/current/tools/audit_closed_loop_payload.py
python3 sim/current/tools/audit_closed_loop_contract.py --help
python3 sim/current/tools/audit_closed_loop_contract.py \
  --json-out /private/tmp/uuv_audit_closed_loop_contract_after_split.json
python3 sim/current/tools/refactor_inventory.py --root sim/current --limit 35
```

Results:

- Parsed JSON payload before/after split was identical.
- The active runtime remained `sim/current`.
- `real_vs_sitl_mismatches` remained `1`; `missing_sitl_params` remained `0`.
- `tools/audit_closed_loop_contract.py` dropped from `337 LOC / 48` branches to
  `76 LOC / 2` branches and no longer appears in the top 35 hotspot inventory.
