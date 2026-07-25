# Dev-OS, Thruster Contract, Param Audit, and Inventory Split

Date: 2026-06-07

## Scope

Continue reducing branch-heavy active-runtime hotspots under
`sim/current` while preserving simulator launch freshness, dev-OS
checks, ArduSub thruster-contract validation, closed-loop parameter audit
payloads, and the refactor inventory tool output.

No ArduPilot source, submodule pointer, controller-parity observation point,
plant input contract, JSON servo semantics, physics coefficients, or runtime
hydrodynamics behavior were changed.

## Changed Files

Dev-OS compatibility:

- `tools/dev_os_compat_docker.py`: Docker CLI/daemon and host-gateway checks.
- `tools/dev_os_compat_ros.py`: ROS2 shell environment check.
- `tools/dev_os_compat_sitl_paths.py`: active runtime alias, launcher,
  runner, real-param, and ArduPilot tree path checks.
- `tools/dev_os_compat_ubuntu.py`: Ubuntu migration markers.
- `tools/dev_os_compat_system.py`: compatibility facade.

ArduSub thruster contract:

- `tools/ardusub_thruster_contract_constants.py`: axis and expected-sign
  constants.
- `tools/ardusub_thruster_contract_scene.py`: MuJoCo scene wrench extraction.
- `tools/ardusub_thruster_contract_response.py`: VECTORED_6DOF axis response.
- `tools/verify_ardusub_thruster_contract.py`: CLI facade.

Closed-loop parameter audit:

- `tools/audit_closed_loop_param_watchlist.py`: watched parameter list.
- `tools/audit_closed_loop_param_io.py`: param/log parsing helpers.
- `tools/audit_closed_loop_param_compare.py`: numeric/string comparison and
  watched-param report.
- `tools/audit_closed_loop_params.py`: compatibility facade.

Refactor inventory:

- `tools/refactor_inventory_types.py`: dataclasses, excludes, and branch-node
  constants.
- `tools/refactor_inventory_analysis.py`: AST file analysis.
- `tools/refactor_inventory_render.py`: markdown renderer.
- `tools/refactor_inventory.py`: CLI facade and JSON/markdown output handling.

## Contract Notes

- `check_dev_os_compat.py` still imports from `dev_os_compat_system`, so caller
  surface is unchanged.
- `verify_ardusub_thruster_contract.py --quiet` still prints
  `[thruster-contract] OK` on success.
- `verify_ardusub_thruster_contract.py --json` still emits the same top-level
  `ok`, `failed`, and `axes` payload shape.
- `audit_closed_loop_contract.py` still loads `WATCH_PARAMS`,
  `parse_param_file`, and `same_param_value` from `audit_closed_loop_params`.
- `refactor_inventory.py --format json` and markdown output are preserved.

## Validation

```text
python3 -m compileall -q sim/current uuv_control_gui.py

PYTHONPATH=sim/current/tools python3 \
  sim/current/tools/audit_code_contract_sources.py \
  --out-dir /private/tmp/uuv_source_audit_spaghetti_20260607_devos_thruster_params_inventory_split

PYTHONPATH=sim/current/tools python3 \
  sim/current/tools/check_runtime_freshness.py \
  --workspace /Users/kanghyunmin/Desktop/uuv_sim \
  --runtime-dir /Users/kanghyunmin/Desktop/uuv_sim/sim/current \
  --fetch --refresh-version

python3 sim/current/tools/check_runtime_readiness_policy.py
python3 sim/current/tools/verify_ardusub_thruster_contract.py --quiet
python3 sim/current/tools/check_dev_os_compat.py --headless --target-os ubuntu

/Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python \
  sim/current/tools/physics_contract_audit.py \
  --output-dir /private/tmp/uuv_physics_audit_spaghetti_20260607_devos_thruster_params_inventory_split \
  --simulate-s 0

python3 sim/current/tools/refactor_inventory.py --limit 30
```

Observed status:

```text
compileall: PASS
source audit: fail=0 pass=11 warn=5
runtime freshness: PASS
runtime_readiness_policy=PASS
thruster-contract: OK
dev-os compatibility: fail=0 pass=16 warn=2
physics static balance: net_down=+0.000 N
```

## Inventory Effect

Removed from the top 30 hotspot list:

- `tools/dev_os_compat_system.py`
- `tools/verify_ardusub_thruster_contract.py`
- `tools/audit_closed_loop_params.py`
- `tools/refactor_inventory.py`

Current top branch-heavy targets after this pass:

```text
tools/althold_diagnostics_node.py          170 LOC / 14 branches
gui/node_motion_callbacks.py               169 LOC / 16 branches
tools/audit_code_contract_runtime_identity.py 161 LOC / 17 branches
sim/runtime/initial_hold.py                161 LOC / 15 branches
bridge/qgc_video_ffmpeg.py                 160 LOC / 13 branches
sim/physics/cfd_dynamic_wrench.py          157 LOC / 20 branches
bridge/ros2_bridge_runtime.py              156 LOC / 24 branches
tools/roll_stability_metrics.py            156 LOC / 15 branches
```
