# GUI Telemetry Layout Split

Date: 2026-06-07

Scope: active runtime only, under `sim/current`.

## Why

`gui/layout_telemetry.py` mixed the vehicle summary, hidden detail rows,
attitude/depth canvases, RC feedback bars, and event log in one module.  That
made telemetry display changes harder to review because sensor-state widgets
and RC feedback widgets were edited in the same file.

## Changed

- Added `gui/layout_vehicle_summary.py` for the summary header and hidden detail
  rows.
- Added `gui/layout_vehicle_visuals.py` for attitude canvas, depth canvas, and
  RC feedback bars.
- Added `gui/layout_event_log.py` for the event listbox.
- Kept `gui/layout_telemetry.py` as the compatibility facade exporting
  `build_telemetry_panel(owner, left)`.

## Validation

Focused smoke:

```text
layout_telemetry_split_smoke PASS
```

Full gates:

```text
python3 -m compileall -q /Users/kanghyunmin/Desktop/uuv_sim/sim/current /Users/kanghyunmin/Desktop/uuv_sim/uuv_control_gui.py
PYTHONPATH=/Users/kanghyunmin/Desktop/uuv_sim/sim/current/tools python3 /Users/kanghyunmin/Desktop/uuv_sim/sim/current/tools/audit_code_contract_sources.py --out-dir /private/tmp/uuv_source_audit_spaghetti_20260607_telemetry_layout_split
python3 /Users/kanghyunmin/Desktop/uuv_sim/sim/current/tools/check_runtime_readiness_policy.py && python3 /Users/kanghyunmin/Desktop/uuv_sim/sim/current/tools/verify_ardusub_thruster_contract.py --quiet
/Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python /Users/kanghyunmin/Desktop/uuv_sim/sim/current/tools/physics_contract_audit.py --output-dir /private/tmp/uuv_physics_audit_spaghetti_20260607_telemetry_layout_split --simulate-s 0.05
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

This is a GUI structure refactor only.  It does not change MAVLink, ROS topic,
sensor, RC override, actuator, or MuJoCo physics contracts.
