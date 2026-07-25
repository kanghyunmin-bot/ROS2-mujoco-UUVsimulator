# SITL Initialization Split

Date: 2026-06-07

Scope: `sim/current/bridge`

## Change

`bridge/sitl_initialization.py` now stays as the public initialization entry
point while the responsibilities are split into focused modules:

- `sitl_initialization_config.py`: sensor replay env parsing and clock/start
  policy normalization.
- `sitl_initialization_loaders.py`: replay CSV and native VPD frame loading.
- `sitl_initialization_state.py`: mutable transport replay state reset.
- `sitl_initialization_logging.py`: replay/VPD startup evidence logs.

## Contract

This is a behavior-preserving refactor.  It does not change ArduPilot, the
submodule pointer, output remapping, PWM shims, or the controller parity
comparison surface.

## Verification

```text
PYTHONPATH=/Users/kanghyunmin/Desktop/uuv_sim/sim/current python3 <focused fake-transport smoke>
python3 -m compileall -q /Users/kanghyunmin/Desktop/uuv_sim/sim/current /Users/kanghyunmin/Desktop/uuv_sim/uuv_control_gui.py
PYTHONPATH=/Users/kanghyunmin/Desktop/uuv_sim/sim/current/tools python3 /Users/kanghyunmin/Desktop/uuv_sim/sim/current/tools/audit_code_contract_sources.py --out-dir /private/tmp/uuv_source_audit_spaghetti_20260607_sitl_init_split
python3 /Users/kanghyunmin/Desktop/uuv_sim/sim/current/tools/check_runtime_readiness_policy.py
python3 /Users/kanghyunmin/Desktop/uuv_sim/sim/current/tools/verify_ardusub_thruster_contract.py --quiet
/Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python /Users/kanghyunmin/Desktop/uuv_sim/sim/current/tools/physics_contract_audit.py --output-dir /private/tmp/uuv_physics_audit_spaghetti_20260607_sitl_init_split --simulate-s 0.05
git -C /Users/kanghyunmin/Desktop/uuv_sim diff --check
```

Result: source audit `fail=0 pass=11 warn=5`, readiness `PASS`, thruster
contract `OK`, physics contract audit completed.
