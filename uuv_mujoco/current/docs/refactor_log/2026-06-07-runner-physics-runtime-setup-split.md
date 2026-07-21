# Runner Physics Runtime Setup Split

Date: 2026-06-07

Scope: active runtime under `uuv_mujoco/current`.

## Change

- Moved the `run_uuv_mujoco.py` physics wiring block into
  `sim/runtime/physics_runtime_setup.py`.
- Split step-time physics callbacks into
  `sim/runtime/physics_step_callbacks.py`.
- Kept the same runtime contracts:
  - plant input remains raw ArduSub JSON/SERVO PWM.
  - controller parity observation remains MAVLink `SERVO_OUTPUT_RAW`.
  - no ArduPilot source or submodule pointer changes.
  - no PWM remap, shim, or physics coefficient retune.

## Inventory Effect

Before this split:

```text
run_uuv_mujoco.py: 612 LOC, 15 branches, main() 541 lines
```

After this split:

```text
run_uuv_mujoco.py: 335 LOC, 7 branches, main() 281 lines
sim/runtime/physics_runtime_setup.py: 417 LOC, 6 branches, create_runtime_physics_setup() 224 lines
```

## Validation

```text
python3 -m compileall -q uuv_mujoco/current uuv_control_gui.py
source ./.uuv_mujoco_env.sh && "$MJ311_PYTHON" uuv_mujoco/current/run_uuv_mujoco.py --help
python3 uuv_mujoco/current/tools/check_runtime_readiness_policy.py
python3 uuv_mujoco/current/tools/verify_ardusub_thruster_contract.py --quiet
PYTHONPATH=uuv_mujoco/current/tools python3 uuv_mujoco/current/tools/audit_code_contract_sources.py --out-dir /private/tmp/uuv_code_contract_after_physics_runtime_setup_split
python3 uuv_mujoco/current/tools/check_dev_os_compat.py --headless --target-os ubuntu
source ./.uuv_mujoco_env.sh && PYTHONPATH=uuv_mujoco/current/tools "$MJ311_PYTHON" uuv_mujoco/current/tools/physics_contract_audit.py --output-dir /private/tmp/uuv_physics_contract_after_physics_runtime_setup_split --simulate-s 0
```

Results:

```text
runtime_readiness_policy=PASS
[thruster-contract] OK
code contract audit: {"fail": 0, "pass": 10, "warn": 5}
dev OS compat: {"fail": 0, "pass": 16, "warn": 2}
physics static force balance: net_down=+0.000N, required_scale=1.000000
```
