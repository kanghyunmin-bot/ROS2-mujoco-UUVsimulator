# ArduPilot Preflight Split

Date: 2026-06-07

Scope: `uuv_mujoco/current/tools`

## Change

`tools/preflight_ardupilot_integrity.py` now owns CLI orchestration only.
Focused helpers own the rest:

- `preflight_ardupilot_git.py`: read-only git status and watched diff
  collection.
- `preflight_ardupilot_classify.py`: P0/P1 issue classification.
- `preflight_ardupilot_output.py`: JSON report writing, terminal output, and
  final exit-code policy.

## Contract

This is a behavior-preserving refactor.  It does not modify ArduPilot, does not
change the ArduPilot submodule pointer, and preserves the watched-file policy
for `ArduSub/control_althold.cpp` and `libraries/AP_Common/missing/fenv.h`.

## Verification

```text
PYTHONPATH=uuv_mujoco/current/tools python3 <preflight classify smoke>
python3 uuv_mujoco/current/tools/preflight_ardupilot_integrity.py --help
python3 uuv_mujoco/current/tools/preflight_ardupilot_integrity.py --workspace . --allow-dirty --json-out /tmp/uuv_ardupilot_preflight_smoke.json
python3 -m compileall -q uuv_mujoco/current uuv_control_gui.py
git diff --check
python3 uuv_mujoco/current/tools/audit_code_contract_sources.py
python3 uuv_mujoco/current/tools/check_runtime_readiness_policy.py
python3 uuv_mujoco/current/tools/verify_ardusub_thruster_contract.py --quiet
```

Result: classify smoke `PASS`; CLI help `PASS`; read-only preflight reported
watched files clean; compileall `PASS`; diff check `PASS`; source audit
`fail=0 pass=11 warn=5`; readiness `PASS`; thruster contract `OK`.
