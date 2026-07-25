# ROS2 Bridge Public API Split

Date: 2026-06-07

## Problem

`bridge/ros2_bridge.py` still owned the public methods called by the simulator
loop (`publish`, `spin_once`, `shutdown`, SITL servo handler wiring, replay
handler wiring, and odometry reset).  These methods are high-branch runtime
orchestration and made the core bridge class harder to inspect while debugging
ROS topic and sensor contracts.

## Change

- Added `bridge/ros2_bridge_public_api.py`.
- Moved public simulator-loop methods there without changing call order:
  - `set_sitl_servo_handler`
  - `set_replay_rcout_handler`
  - `sitl_vehicle_armed`
  - `sitl_vehicle_mode`
  - `spin_once`
  - `publish`
  - `force_next_publish`
  - `reset_odometry`
  - `shutdown`
- `Ros2Bridge` now binds those methods as aliases, preserving the class API
  consumed by `run_uuv_mujoco.py` and runtime wrappers.

## Size Check

```text
bridge/ros2_bridge.py: LOC=518 branches=46 funcs=14 largest=__init__:128
bridge/ros2_bridge_public_api.py: LOC=170 branches=52 funcs=10 largest=shutdown:35
```

Previous measured state:

```text
bridge/ros2_bridge.py: LOC=645 branches=98
```

## Validation

```text
python3 -m py_compile \
  sim/current/bridge/ros2_bridge.py \
  sim/current/bridge/ros2_bridge_public_api.py
```

```text
PYTHONPATH=sim/current "$MJ311_PYTHON" - <<'PY'
from bridge.ros2_bridge import Ros2Bridge
# verified all public simulator-loop methods still exist on Ros2Bridge
PY
```

```text
python3 -m compileall -q sim/current uuv_control_gui.py
python3 sim/current/tools/check_runtime_readiness_policy.py
python3 sim/current/tools/verify_ardusub_thruster_contract.py --quiet
python3 sim/current/tools/audit_code_contract_sources.py \
  --out-dir /private/tmp/uuv_code_contract_after_ros2_public_api_split
python3 sim/current/tools/check_dev_os_compat.py --headless --target-os ubuntu
```

Results:

- `runtime_readiness_policy=PASS`
- `[thruster-contract] OK`
- `audit_code_contract_sources`: `fail=0`, `pass=10`, `warn=5`
- `check_dev_os_compat`: `fail=0`, `pass=16`, `warn=2`
