# Dynamic Fluidcoef Split

Date: 2026-06-07

Scope: active runtime under `uuv_mujoco/current`.

## Change

- Replaced `sim/physics/dynamic_fluidcoef.py` with a compatibility facade.
- Split responsibilities into:
  - `dynamic_fluidcoef_types.py`: default weights and setup dataclass.
  - `dynamic_fluidcoef_loads.py`: velocity-to-load math.
  - `dynamic_fluidcoef_runtime.py`: runtime MuJoCo `geom_fluid` updater.
  - `dynamic_fluidcoef_setup.py`: profile pattern matching and setup arrays.
- Preserved the existing import surface:
  `from sim.physics.dynamic_fluidcoef import DynamicFluidcoefRuntime,
  build_dynamic_fluidcoef_setup`.

## Inventory Effect

Before:

```text
sim/physics/dynamic_fluidcoef.py: 567 LOC, 37 branches
```

After:

```text
sim/physics/dynamic_fluidcoef.py: 25 LOC facade
dynamic_fluidcoef_runtime.py: 297 LOC
dynamic_fluidcoef_setup.py: 187 LOC
dynamic_fluidcoef_loads.py: 61 LOC
dynamic_fluidcoef_types.py: 45 LOC
```

The dynamic fluidcoef file is no longer in the top 30 hotspot inventory.

## Validation

```text
python3 -m compileall -q uuv_mujoco/current uuv_control_gui.py
source ./.uuv_mujoco_env.sh && "$MJ311_PYTHON" uuv_mujoco/current/run_uuv_mujoco.py --help
python3 uuv_mujoco/current/tools/check_runtime_readiness_policy.py
python3 uuv_mujoco/current/tools/verify_ardusub_thruster_contract.py --quiet
PYTHONPATH=uuv_mujoco/current/tools python3 uuv_mujoco/current/tools/audit_code_contract_sources.py --out-dir /private/tmp/uuv_code_contract_after_dynamic_fluidcoef_split
python3 uuv_mujoco/current/tools/check_dev_os_compat.py --headless --target-os ubuntu
source ./.uuv_mujoco_env.sh && PYTHONPATH=uuv_mujoco/current/tools "$MJ311_PYTHON" uuv_mujoco/current/tools/physics_contract_audit.py --output-dir /private/tmp/uuv_physics_contract_after_dynamic_fluidcoef_split --simulate-s 0
```

Results:

```text
runtime_readiness_policy=PASS
[thruster-contract] OK
code contract audit: {"fail": 0, "pass": 10, "warn": 5}
dev OS compat: {"fail": 0, "pass": 16, "warn": 2}
physics static force balance: net_down=+0.000N, required_scale=1.000000
```
