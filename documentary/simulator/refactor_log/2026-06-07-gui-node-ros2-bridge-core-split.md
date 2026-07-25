# GUI Node and ROS2 Bridge Core Split

Date: 2026-06-07

Scope: active runtime under `sim/current`.

## Change

- Split `gui/node.py`.
  - `gui/node_state_runtime.py` now owns backend graph probing, command
    readiness calculation, event insertion, telemetry snapshot copying, and
    age calculation.
  - `gui/node.py` keeps ROS subscriptions, publishers, clients, constructor
    setup, vehicle-info request wiring, and compatibility method exports for
    command/telemetry callbacks.
- Split `bridge/ros2_bridge.py`.
  - `bridge/ros2_bridge_runtime_methods.py` now owns rate helpers, safe
    publish, ROS spin thread loop, topic rate gating, MAVROS state message
    construction, robot-description loading, static-context publishing, and
    sensor slicing.
  - `bridge/ros2_ping360_config.py` owns the `/ping360/config` callback.
  - `bridge/ros2_bridge.py` now focuses on constructor wiring and compatibility
    method exports.

No controller-parity or plant-input contract was changed.  Real `/mavros/rc/out`
vs SITL `SERVO_OUTPUT_RAW` remains the controller-parity comparison surface,
and raw ArduSub JSON servo remains the plant input surface.

## Inventory Effect

Before:

```text
gui/node.py: 527 LOC, 67 branches
bridge/ros2_bridge.py: 518 LOC, 46 branches
```

After:

```text
gui/node.py: 341 LOC
gui/node_state_runtime.py: 276 LOC
bridge/ros2_bridge.py: 330 LOC
bridge/ros2_bridge_runtime_methods.py: 190 LOC
bridge/ros2_ping360_config.py: 44 LOC
```

Both `gui/node.py` and `bridge/ros2_bridge.py` are no longer in the top 15
hotspot inventory.  The next hotspot is `gui/layout_mixin.py`, which is mostly
single-function Tk layout construction rather than high-branch logic.

## Validation

```text
python3 -m py_compile \
  uuv_mujoco/v2.2/gui/node.py \
  uuv_mujoco/v2.2/gui/node_state_runtime.py \
  uuv_mujoco/v2.2/bridge/ros2_bridge.py \
  uuv_mujoco/v2.2/bridge/ros2_bridge_runtime_methods.py \
  uuv_mujoco/v2.2/bridge/ros2_ping360_config.py \
  uuv_control_gui.py
```

```text
python3 -m compileall -q sim/current uuv_control_gui.py
source ./.uuv_mujoco_env.sh && "$MJ311_PYTHON" sim/current/run_uuv_mujoco.py --help
python3 sim/current/tools/check_runtime_readiness_policy.py
python3 sim/current/tools/verify_ardusub_thruster_contract.py --quiet
PYTHONPATH=sim/current/tools python3 sim/current/tools/audit_code_contract_sources.py --out-dir /private/tmp/uuv_code_contract_after_node_bridge_split
source ./.uuv_mujoco_env.sh && PYTHONPATH=sim/current/tools "$MJ311_PYTHON" sim/current/tools/physics_contract_audit.py --output-dir /private/tmp/uuv_physics_contract_after_node_bridge_split --simulate-s 0
python3 sim/current/tools/check_dev_os_compat.py --headless --target-os ubuntu
git diff --check -- uuv_mujoco/v2.2/gui/node.py uuv_mujoco/v2.2/gui/node_state_runtime.py uuv_mujoco/v2.2/bridge/ros2_bridge.py uuv_mujoco/v2.2/bridge/ros2_bridge_runtime_methods.py uuv_mujoco/v2.2/bridge/ros2_ping360_config.py
```

Results:

```text
runtime_readiness_policy=PASS
[thruster-contract] OK
code contract audit: {"fail": 0, "pass": 10, "warn": 5}
physics static force balance: net_down=+0.000N, required_scale=1.000000
dev OS compat: {"fail": 0, "pass": 16, "warn": 2}
git diff --check: clean
```
