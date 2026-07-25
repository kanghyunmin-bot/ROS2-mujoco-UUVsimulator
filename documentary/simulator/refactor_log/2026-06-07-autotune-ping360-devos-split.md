# AutoTune, Ping360, and Dev OS Checker Split

Date: 2026-06-07

Scope: active runtime under `sim/current`.

## Change

- Split auto-tune monitor responsibilities out of `gui/autotune_mixin.py`.
  - `gui/autotune_monitor.py` now owns monitor window construction, live log
    updates, candidate parsing, progress updates, and score chart drawing.
  - `gui/autotune_mixin.py` keeps workflow state, bag selection, process
    start/stop, output reader, and compatibility method names.
- Split Ping360 data contracts and firmware-style setting calculations out of
  `bridge/ping360_sim.py`.
  - `bridge/ping360_types.py` owns constants, config, effective settings, and
    sample dataclasses.
  - `bridge/ping360_settings.py` owns range/sample/transmit-duration and scan
    timing calculations.
  - `bridge/ping360_sim.py` now focuses on MuJoCo raycast profile generation
    and active scan state.
- Split the development OS compatibility checker.
  - `tools/dev_os_compat_common.py` owns shared paths, result records, command
    execution, and Python probe helpers.
  - `tools/dev_os_compat_runtime.py` owns Python, MuJoCo, viewer, mjpython, and
    display checks.
  - `tools/dev_os_compat_system.py` owns Docker, SITL path, Ubuntu migration,
    and ROS2 environment checks.
  - `tools/check_dev_os_compat.py` is now CLI parsing, aggregation, and output.

## Inventory Effect

Before:

```text
gui/autotune_mixin.py: 548 LOC, 80 branches
bridge/ping360_sim.py: 544 LOC, 56 branches
tools/check_dev_os_compat.py: 527 LOC, 79 branches
```

After:

```text
gui/autotune_mixin.py: 259 LOC
gui/autotune_monitor.py: 389 LOC
bridge/ping360_sim.py: 286 LOC
bridge/ping360_types.py: 152 LOC
bridge/ping360_settings.py: 146 LOC
tools/check_dev_os_compat.py: 111 LOC
tools/dev_os_compat_common.py: 142 LOC
tools/dev_os_compat_runtime.py: 167 LOC
tools/dev_os_compat_system.py: 176 LOC
```

The old auto-tune, Ping360, and dev-OS checker hotspots are no longer in the
top 14 hotspot inventory.  The current largest hotspots are `gui/node.py`,
`bridge/ros2_bridge.py`, `gui/layout_mixin.py`, and `gui/node_commanding.py`.

## Validation

```text
python3 -m py_compile \
  uuv_mujoco/v2.2/gui/autotune_mixin.py \
  uuv_mujoco/v2.2/gui/autotune_monitor.py \
  uuv_mujoco/v2.2/bridge/ping360_types.py \
  uuv_mujoco/v2.2/bridge/ping360_settings.py \
  uuv_mujoco/v2.2/bridge/ping360_sim.py \
  uuv_mujoco/v2.2/tools/check_dev_os_compat.py \
  uuv_mujoco/v2.2/tools/dev_os_compat_common.py \
  uuv_mujoco/v2.2/tools/dev_os_compat_runtime.py \
  uuv_mujoco/v2.2/tools/dev_os_compat_system.py
```

```text
source ./.uuv_mujoco_env.sh
"$MJ311_PYTHON" - <<'PY'
from uuv_mujoco.current.bridge.ping360_sim import PING360_GRADS_PER_REV, Ping360Config, Ping360Simulator
from uuv_mujoco.current.bridge.ping360_settings import build_effective_settings
settings = build_effective_settings(Ping360Config(requested_range_m=2.0))
print(PING360_GRADS_PER_REV, settings.number_of_samples, round(settings.effective_range_m, 3), Ping360Simulator.__name__)
PY
```

```text
python3 -m compileall -q sim/current uuv_control_gui.py
source ./.uuv_mujoco_env.sh && "$MJ311_PYTHON" sim/current/run_uuv_mujoco.py --help
python3 sim/current/tools/check_runtime_readiness_policy.py
python3 sim/current/tools/verify_ardusub_thruster_contract.py --quiet
PYTHONPATH=sim/current/tools python3 sim/current/tools/audit_code_contract_sources.py --out-dir /private/tmp/uuv_code_contract_after_autotune_ping360_devos_split
source ./.uuv_mujoco_env.sh && PYTHONPATH=sim/current/tools "$MJ311_PYTHON" sim/current/tools/physics_contract_audit.py --output-dir /private/tmp/uuv_physics_contract_after_autotune_ping360_devos_split --simulate-s 0
python3 sim/current/tools/check_dev_os_compat.py --headless --target-os ubuntu
git diff --check -- uuv_mujoco/v2.2/gui/autotune_mixin.py uuv_mujoco/v2.2/gui/autotune_monitor.py uuv_mujoco/v2.2/bridge/ping360_sim.py uuv_mujoco/v2.2/bridge/ping360_types.py uuv_mujoco/v2.2/bridge/ping360_settings.py uuv_mujoco/v2.2/tools/check_dev_os_compat.py uuv_mujoco/v2.2/tools/dev_os_compat_common.py uuv_mujoco/v2.2/tools/dev_os_compat_runtime.py uuv_mujoco/v2.2/tools/dev_os_compat_system.py
```

Results:

```text
Ping360 import/settings check: 400 1200 1.98 Ping360Simulator
runtime_readiness_policy=PASS
[thruster-contract] OK
code contract audit: {"fail": 0, "pass": 10, "warn": 5}
physics static force balance: net_down=+0.000N, required_scale=1.000000
dev OS compat: {"fail": 0, "pass": 16, "warn": 2}
git diff --check: clean
```
