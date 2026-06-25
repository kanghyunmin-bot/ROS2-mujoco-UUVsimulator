# Profile, GUI Init, and ROS Message Facade Split

Date: 2026-06-07

## Problem

The current hotspot inventory still had multiple mixed-responsibility files:

- `physics/sim_profile_helpers.py` mixed built-in profile data with profile
  loading and runtime override logic.
- `gui/node_init.py` mixed GUI node state, real-start gate state,
  publisher/client wiring, and subscription wiring.
- `bridge/ros2_standard_messages.py` mixed pose, twist, IMU, pressure, status,
  battery, and DVL-adjacent builders.

## Change

- Added `physics/sim_profile_defaults.py` for profile aliases and built-in
  default profiles. `physics/sim_profile_helpers.py` remains the compatibility
  facade for existing imports.
- Split GUI node init into:
  - `gui/node_init_state.py`
  - `gui/node_init_publishers.py`
  - `gui/node_init_subscriptions.py`
  - `gui/node_init.py` facade
- Split standard ROS2 builders into:
  - `bridge/ros2_pose_messages.py`
  - `bridge/ros2_twist_messages.py`
  - `bridge/ros2_sensor_messages.py`
  - `bridge/ros2_status_messages.py`
  - `bridge/ros2_standard_messages.py` facade

## Contract

No topic names, callback names, message fields, profile values, or public import
names changed. Existing code can still import from:

```text
physics.sim_profile_helpers
gui.node_init
bridge.ros2_standard_messages
```

## Verification

```text
python3 -m compileall -q uuv_mujoco/current uuv_control_gui.py
/Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python - <<'PY'
import sys
sys.path.insert(0, 'uuv_mujoco/current')
from gui.node_init import initialize_uuv_gui_node, initialize_subscriptions
from bridge.ros2_standard_messages import build_imu_msg, build_pose_msg
print('ros2_python_imports_ok', bool(initialize_uuv_gui_node and initialize_subscriptions and build_imu_msg and build_pose_msg))
PY
PYTHONPATH=uuv_mujoco/current/tools python3 uuv_mujoco/current/tools/audit_code_contract_sources.py --out-dir /private/tmp/uuv_source_audit_spaghetti_split
python3 uuv_mujoco/current/tools/check_runtime_readiness_policy.py
python3 uuv_mujoco/current/tools/verify_ardusub_thruster_contract.py --quiet
```

Result:

```text
ros2_python_imports_ok True
contract source audit: fail=0 pass=11 warn=5
runtime_readiness_policy=PASS
thruster-contract OK
```

Inventory effect:

```text
physics/sim_profile_helpers.py removed from top 20
gui/node_init.py removed from top 20
bridge/ros2_standard_messages.py removed from top 20
```
