# Dev OS Gate and ExternalNav Split

Date: 2026-06-06

## Scope

This pass kept all edits inside `uuv_mujoco/v2.2` and did not touch the
ArduPilot source tree or submodule pointer.

## Changes

- Added `docs/contracts/DEV_OS_COMPATIBILITY.md`.
- Linked the development OS gate from `docs/contracts/CURRENT_RUNTIME_CONTRACT.md`.
- Extended `tools/check_dev_os_compat.py` so macOS viewer runs verify
  `mjpython` separately from headless Python/MuJoCo checks.
- Updated `tools/audit_code_contract_sources.py` so Bar30 JSON payload evidence
  follows the new `bridge/sitl_json_sensor_runtime.py` split.
- Extracted ExternalNav/VISION_POSITION_DELTA runtime helpers from
  `bridge/sitl_transport.py` into `bridge/sitl_external_nav_runtime.py`.
- Extracted the ROS2 SITL sensor-feed snapshot path from `bridge/ros2_bridge.py`
  into `bridge/ros2_sitl_sensor_feed.py`.
- Extracted ROS topic publish queue construction from `bridge/ros2_bridge.py`
  into `bridge/ros2_publish_runtime.py`.
- Extracted GUI telemetry subscription callbacks from `gui/node.py` into
  `gui/node_telemetry_callbacks.py`.

## Validation

Commands run:

```bash
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 -m compileall -q uuv_mujoco/v2.2
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 uuv_mujoco/v2.2/tools/check_dev_os_compat.py --headless --json
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 uuv_mujoco/v2.2/tools/check_dev_os_compat.py --require-viewer --json
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 uuv_mujoco/v2.2/tools/audit_code_contract_sources.py --out-dir /private/tmp/uuv_code_contract_after_dev_os_gate
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 uuv_mujoco/v2.2/tools/check_runtime_readiness_policy.py
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 uuv_mujoco/v2.2/tools/verify_ardusub_thruster_contract.py --quiet
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 uuv_mujoco/v2.2/tools/audit_closed_loop_contract.py --json-out /private/tmp/uuv_closed_loop_contract_after_dev_os_gate.json
env PYTHONPATH=uuv_mujoco/v2.2 PYTHONPYCACHEPREFIX=/private/tmp/pycache /Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python -c "from gui.node import UuvGuiNode; print(hasattr(UuvGuiNode, '_on_state'))"
```

Observed results:

- compileall: pass
- development OS gate, headless: `fail=0`, `warn=2`
- development OS gate, viewer: `fail=0`, `warn=2`
- source contract audit: `fail=0`, `warn=5`
- readiness policy: pass
- thruster contract: pass
- closed-loop contract audit: still reports the pre-existing watched-param
  mismatch count of `1`
- ROS2 bridge runtime import: pass
- GUI node callback binding import on the ROS2 conda Python: pass

## Remaining Structure Risk

The largest remaining runtime files are still:

- `run_urdf_full.py`
- `bridge/ros2_bridge.py`
- `gui/node.py`
- `gui/ros_process_mixin.py`

The next useful split should target ROS publish surfaces or GUI process control,
not physics coefficient tuning.
