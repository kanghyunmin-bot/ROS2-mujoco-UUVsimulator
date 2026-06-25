# Runtime Logging Section Split

Date: 2026-06-07

Scope: `uuv_mujoco/current/sim/runtime`

## Change

Hydrodynamics and thruster diagnostic logging now separate orchestration from
individual log sections:

- `hydrodynamics_runtime_logging.py`: top-level hydrodynamics setup summary.
- `hydrodynamics_runtime_log_sections.py`: thruster-loop, buoyancy, immersion,
  yaw-torque, custom-hydrodynamics, and MuJoCo-hydrodynamics log sections.
- `thruster_param_runtime_summary.py`: top-level thruster parameter summary.
- `thruster_param_runtime_log_sections.py`: base dynamics, direct gain
  overrides, yaw reverse-asymmetry overrides, and yaw dynamics override log
  sections.

## Contract

This is a behavior-preserving refactor.  It does not change hydrodynamic
coefficients, hydrostatic setup, thruster parameters, PWM curves, actuator
contracts, or controller-parity observation surfaces.

## Verification

```text
/Users/kanghyunmin/.venvs/mujoco311/bin/python <focused runtime logging smoke>
python3 -m compileall -q uuv_mujoco/current uuv_control_gui.py
python3 uuv_mujoco/current/tools/audit_code_contract_sources.py
python3 uuv_mujoco/current/tools/check_runtime_readiness_policy.py
python3 uuv_mujoco/current/tools/verify_ardusub_thruster_contract.py --quiet
/Users/kanghyunmin/.venvs/mujoco311/bin/python uuv_mujoco/current/tools/physics_contract_audit.py --simulate-s 0.05
git diff --check
```

Result: runtime logging smoke `PASS`; source audit `fail=0 pass=11 warn=5`;
readiness `PASS`; thruster contract `OK`; physics contract audit completed;
compileall and diff check `PASS`.
