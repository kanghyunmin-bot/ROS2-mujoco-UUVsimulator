# GUI, Topic Registry, and Hydrodynamics Runtime Split

Date: 2026-06-07

## Problem

The next hotspot inventory still had several mixed-responsibility files:

- `gui/ping360_mixin.py` mixed window layout, sonar config publishing, and
  RViz/rqt process control.
- `gui/widgets.py` kept joystick state, drag math, geometry, and canvas drawing
  in one widget class.
- `bridge/ros2_topic_registry.py` mixed topic/service registry data with launch
  and runtime summary strings.
- `sim/runtime/underwater_hydrodynamics_runtime.py` mixed custom diagonal
  damping, residual/Fossen terms, CFD dynamic force, and empirical pitch/lift
  terms in one function.

## Change

- Split Ping360 GUI ownership into:
  - `gui/ping360_window_mixin.py`
  - `gui/ping360_view_mixin.py`
  - `gui/ping360_config_mixin.py`
  - `gui/ping360_mixin.py` facade
- Split the virtual joystick into:
  - `gui/joystick_math.py`
  - `gui/joystick_render.py`
  - `gui/widgets.py` public `VirtualJoystick`
- Split topic registry ownership into:
  - `bridge/ros2_topic_specs.py`
  - `bridge/ros2_topic_summaries.py`
  - `bridge/ros2_topic_registry.py` facade
- Split underwater hydrodynamic wrench application into:
  - `sim/runtime/underwater_hydrodynamics_custom.py`
  - `sim/runtime/underwater_hydrodynamics_residual.py`
  - `sim/runtime/underwater_hydrodynamics_extra.py`
  - `sim/runtime/underwater_hydrodynamics_runtime.py` orchestration facade

## Contract

No Ping360 topic names, GUI callback names, joystick axis convention, ROS2 topic
summary text, or hydrodynamic equations were intentionally changed. Public
imports remain:

```text
gui.ping360_mixin.Ping360ControlMixin
gui.widgets.VirtualJoystick
bridge.ros2_topic_registry
sim.runtime.underwater_hydrodynamics_runtime.apply_hydrodynamic_wrenches
```

## Verification

```text
python3 -m compileall -q ...
PYTHONPATH=sim/current python3 - <<'PY'
from gui.widgets import VirtualJoystick
from bridge.ros2_topic_registry import build_bridge_topic_summary
from sim.runtime.underwater_hydrodynamics_runtime import apply_hydrodynamic_wrenches
print(bool(VirtualJoystick and build_bridge_topic_summary and apply_hydrodynamic_wrenches))
PY
/Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python sim/current/tools/physics_contract_audit.py --output-dir /private/tmp/uuv_physics_audit_hydro_split --simulate-s 0
```

Result:

```text
ping360 imports ok
widget math import ok
topic registry summary import ok
hydrodynamics import ok
physics static balance PASS
```

Inventory effect:

```text
gui/ping360_mixin.py removed from top hotspot list
gui/widgets.py removed from top hotspot list
bridge/ros2_topic_registry.py removed from top hotspot list
sim/runtime/underwater_hydrodynamics_runtime.py removed from top hotspot list
```
