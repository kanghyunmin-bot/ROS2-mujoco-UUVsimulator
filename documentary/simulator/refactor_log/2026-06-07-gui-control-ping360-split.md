# GUI Control And Ping360 Split

Date: 2026-06-07

Scope: active runtime only, under `sim/current`.

## Why

`gui/layout_control_core.py` mixed the telemetry toggle, SITL/MuJoCo stack
buttons, ROS2/MAVROS utility controls, and arm/mode command buttons.  The GUI
Ping360 window also mixed window lifecycle and widget construction in one
mixin.  Both made command-path review harder because UI layout and lifecycle
logic were coupled.

## Changed

- Added `gui/layout_control_stack.py` for telemetry toggle and simulation stack
  buttons.
- Added `gui/layout_control_ros2.py` for ROS2/MAVROS utility panel widgets.
- Added `gui/layout_control_modes.py` for Arm/Disarm and mode buttons.
- Kept `gui/layout_control_core.py` as the compatibility builder exporting
  `build_control_panel(owner, right)`.
- Added `gui/ping360_window_lifecycle.py` for Ping360 window show/toggle/close.
- Added `gui/ping360_window_panels.py` for Ping360 status, power, viewer,
  parameter, and footer panels.
- Kept `gui/ping360_window_mixin.py` as the compatibility mixin surface.

## Validation

Focused smoke:

```text
layout_control_core_split_smoke PASS
ping360_window_split_smoke PASS
```

Full gates:

```text
python3 -m compileall -q /Users/kanghyunmin/Desktop/uuv_sim/sim/current /Users/kanghyunmin/Desktop/uuv_sim/uuv_control_gui.py
PYTHONPATH=/Users/kanghyunmin/Desktop/uuv_sim/sim/current/tools python3 /Users/kanghyunmin/Desktop/uuv_sim/sim/current/tools/audit_code_contract_sources.py --out-dir /private/tmp/uuv_source_audit_spaghetti_20260607_gui_control_ping_split
python3 /Users/kanghyunmin/Desktop/uuv_sim/sim/current/tools/check_runtime_readiness_policy.py && python3 /Users/kanghyunmin/Desktop/uuv_sim/sim/current/tools/verify_ardusub_thruster_contract.py --quiet
/Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python /Users/kanghyunmin/Desktop/uuv_sim/sim/current/tools/physics_contract_audit.py --output-dir /private/tmp/uuv_physics_audit_spaghetti_20260607_gui_control_ping_split --simulate-s 0.05
```

Results:

```text
compileall PASS
source contract audit: {"fail": 0, "pass": 11, "warn": 5}
runtime_readiness_policy=PASS
[thruster-contract] OK
physics contract audit PASS, static force balance required_scale=1.000000
```

## Contract Notes

This pass changes GUI structure only.  It does not change MAVLink command
routing, RC override timing, arm/mode service semantics, Ping360 ROS message
contents, bridge subscriptions, sensor contracts, actuator contracts, or
MuJoCo physics parameters.
