# GUI ROS Panel Split

Date: 2026-06-10

## Change

- Split `gui/ros_panel_mixin.py` into focused ROS-panel helpers:
  - `gui/ros_panel_visibility.py`: optional ROS2 panel show/hide behavior.
  - `gui/ros_panel_process_state.py`: MAVROS build/runtime/RViz process
    running predicates and external-MAVROS control gate.
  - `gui/ros_panel_status.py`: thread-safe Tk status variable updates.
  - `gui/ros_panel_buttons.py`: MAVROS/RViz toggle button label refresh.
- Kept `gui/ros_panel_mixin.py` as the compatibility method-binding surface.
- Removed an unnecessary `.runtime` import from the status helper by using
  stdlib `threading` directly.  This lets the ROS panel mixin import in a
  non-ROS Python smoke without pulling `rclpy` through the GUI runtime module.

## Contract

- No RC input/output contract changed.
- No Bar30, SERVO_OUTPUT_RAW, thruster, SITL, or MuJoCo physics path changed.
- GUI method names consumed by package build, MAVROS stack toggle, RViz toggle,
  and sim-stack restart logic are preserved.

## Verification

```text
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 -m compileall -q uuv_mujoco/current/gui
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 - <<'PY'
import sys
from pathlib import Path
sys.path.insert(0, str(Path('uuv_mujoco/current').resolve()))
from gui.ros_panel_mixin import RosPanelMixin
required = [
    '_toggle_ros2_panel', '_ros_pkg_running', '_ros_build_running',
    '_rviz_running', '_gui_external_mavros_controls_enabled',
    '_set_ros_pkg_status', '_set_rviz_status', '_refresh_ros2_buttons',
]
missing = [name for name in required if not callable(getattr(RosPanelMixin, name, None))]
assert not missing, missing
print('ros_panel_mixin_bindings=PASS')
PY
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 uuv_mujoco/current/tools/check_rc_frame_contract.py
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 uuv_mujoco/current/tools/check_gui_readiness_contract.py
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 uuv_mujoco/current/tools/check_gui_backend_selection.py
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 uuv_mujoco/current/tools/check_runtime_readiness_policy.py
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 uuv_mujoco/current/tools/verify_ardusub_thruster_contract.py --quiet
env PYTHONPYCACHEPREFIX=/private/tmp/pycache PYTHONPATH=uuv_mujoco/current/tools python3 uuv_mujoco/current/tools/audit_code_contract_sources.py --out-dir /private/tmp/uuv_source_audit_ros_panel_split_20260610
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 uuv_mujoco/current/tools/refactor_inventory.py --root uuv_mujoco/current --format markdown --limit 12
```

Results:

- `ros_panel_mixin_bindings=PASS`
- `rc_frame_contract=PASS`
- `gui_readiness_contract=PASS`
- `gui_backend_selection=PASS`
- `runtime_readiness_policy=PASS`
- `[thruster-contract] OK`
- source contract audit: `{"fail": 0, "pass": 11, "warn": 5}`
- `gui/ros_panel_mixin.py` is no longer in the top 12 hotspot list.
