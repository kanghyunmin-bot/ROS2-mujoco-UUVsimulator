# SITL Status Payload Split

Date: 2026-06-07

## Scope

Split `bridge/sitl_status.py` into focused wall-clock age, sensor replay status,
MAVLink core status, and ExternalNav readiness helpers without changing the GUI
status payload keys.

## Files

- `bridge/sitl_status.py`
  - Compatibility export surface.
- `bridge/sitl_status_age.py`
  - Wall-clock age helper.
- `bridge/sitl_status_sensor_replay.py`
  - Sensor replay status payload builder.
- `bridge/sitl_status_mavlink.py`
  - Top-level MAVLink telemetry status builder.
- `bridge/sitl_status_mavlink_core.py`
  - Command heartbeat, servo heartbeat, RC override, vehicle mode, and
    auto-ready fields.
- `bridge/sitl_status_mavlink_extnav.py`
  - ExternalNav freshness/rate/grace readiness fields.

## Contract Notes

- `current_real_t_s`, frame depth/velocity/pressure, frame RPY, and frame gyro
  keys are preserved for sensor replay status.
- Command/servo heartbeat age, `rc_override_ready`, vehicle arm/mode,
  auto-ready, and ExternalNav readiness keys are preserved for MAVLink telemetry
  status.
- `extnav_ready` still requires both freshness and rate checks unless ExternalNav
  is not required.
- `auto_ready_done` still requires positive done time, armed vehicle, and mode
  matching `auto_ready_mode`.

## Verification

```text
sitl status split smoke: PASS
sitl status mavlink core split smoke: PASS
python3 -m compileall -q sim/current uuv_control_gui.py: PASS
audit_code_contract_sources.py: fail=0 pass=11 warn=5
check_runtime_readiness_policy.py: PASS
verify_ardusub_thruster_contract.py --quiet: PASS
physics_contract_audit.py --simulate-s 0.05: PASS
git diff --check: PASS
check_runtime_freshness.py --warn-only: WARN current-dirty, dirty_paths=684, active_runtime_dirty_paths=666
refactor_inventory.py: bridge/sitl_status.py and bridge/sitl_status_mavlink.py removed from top 45
```
