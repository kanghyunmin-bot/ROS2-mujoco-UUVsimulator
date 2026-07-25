# Runtime Startup And Physics Reporting Split

Date: 2026-06-07

Scope: `sim/current/sim/runtime`, `sim/current/sim/physics`

## Change

Execution-path hotspots were split by contract owner:

- `runner_initial_setup.py` now orchestrates model loading and returned setup
  assembly only.  Initial-depth runtime construction, startup thruster immersion
  env parsing, and the returned dataclass live in focused helper modules.
- `control_bridge_setup.py` now orchestrates command/ROS bridge setup only.
  Command timeout/direct-command policy, initial-depth release service wiring,
  real-start status construction, and the returned dataclass live in focused
  helper modules.
- `cfd_dynamic_wrench_profile.py` now delegates axis force-table validation and
  parsing to `cfd_dynamic_wrench_axis_parser.py`.
- `hydrostatic_runtime_reporting.py` now delegates CoB site alignment/override
  logging and hydrostatic application/restoring log sections to focused modules.
- `initial_hold.py` now keeps the public `InitialDepthHoldState` fields and
  factory while dict-style compatibility and pose/release actions live in
  mixins.

## Contract

This is a behavior-preserving refactor.  It does not change ArduPilot,
submodule provenance, controller-parity observation surfaces, RC override
semantics, initial-depth hold semantics, Bar30/depth semantics, hydrostatic
coefficients, CFD dynamic-wrench tables, or thruster/PWM contracts.

## Verification

```text
/Users/kanghyunmin/.venvs/mujoco311/bin/python <runner initial/control setup split smoke>
/Users/kanghyunmin/.venvs/mujoco311/bin/python <cfd/hydrostatic reporting split smoke>
/Users/kanghyunmin/.venvs/mujoco311/bin/python <initial hold state split smoke>
python3 -m compileall -q sim/current uuv_control_gui.py
git diff --check
python3 sim/current/tools/audit_code_contract_sources.py
python3 sim/current/tools/check_runtime_readiness_policy.py
python3 sim/current/tools/verify_ardusub_thruster_contract.py --quiet
/Users/kanghyunmin/.venvs/mujoco311/bin/python sim/current/tools/physics_contract_audit.py --simulate-s 0.05
```

Result: all focused smokes `PASS`; compileall `PASS`; diff check `PASS`;
source audit `fail=0 pass=11 warn=5`; readiness `PASS`; thruster contract
`OK`; static physics contract audit completed.
