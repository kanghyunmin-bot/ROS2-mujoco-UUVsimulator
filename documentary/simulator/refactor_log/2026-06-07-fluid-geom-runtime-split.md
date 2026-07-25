# Fluid Geom Runtime Split

Date: 2026-06-07

## Scope

Split the MuJoCo fluid geom runtime scaling hotspot without changing the active
hydrodynamic coefficient contract.

## Changed Files

- `sim/physics/fluid_geom_common.py`: geom name lookup, fluid geom mask,
  wildcard matching, and scalar/vector parsing helpers.
- `sim/physics/fluid_geom_size_runtime.py`: profile and environment geom-size
  scale application.
- `sim/physics/fluidcoef_scale_runtime.py`: global, per-geom, and environment
  MuJoCo `fluidcoef` scale application for the five MuJoCo coefficients
  `(blunt, slender, angular, Kutta, Magnus)`.
- `sim/physics/fluid_geom_apply.py`: ordered top-level application and derived
  geom metadata return.
- `sim/physics/fluid_geom_runtime.py`: compatibility export surface.

## Contract Notes

- No ArduPilot source or submodule pointer changes.
- No controller-parity shim, PWM remap, or output correction was introduced.
- MuJoCo `geom_fluid[:, 1:6]` semantics remain the same; this split only makes
  size scaling and coefficient scaling separately inspectable.
- The active runtime remains `sim/current`; the backing directory name
  `v2.2` is compatibility metadata only.

## Validation

```text
python3 -m compileall -q sim/current uuv_control_gui.py
PYTHONPATH=sim/current/tools python3 sim/current/tools/audit_code_contract_sources.py --out-dir /private/tmp/uuv_code_contract_after_fluid_geom_split
python3 sim/current/tools/check_runtime_readiness_policy.py
python3 sim/current/tools/verify_ardusub_thruster_contract.py --quiet
python3 sim/current/tools/check_dev_os_compat.py --headless --target-os ubuntu
git diff --check
```

Observed status after the split:

```text
source audit: fail=0 pass=11 warn=5
runtime readiness: PASS
thruster contract: OK
dev OS compatibility: fail=0 pass=16 warn=2
diff whitespace: clean
```

The two dev-OS warnings are environment state only: Docker daemon is not
running and ROS2 is not sourced in the current shell.
