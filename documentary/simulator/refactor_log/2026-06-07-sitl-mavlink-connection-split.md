# SITL MAVLink Connection Split

Date: 2026-06-07

## Scope

Split `bridge/sitl_mavlink_connection.py` into focused endpoint, dependency,
servo connection, command connection, reconnect, and heartbeat helpers without
changing the SITL MAVLink connection contract.

## Files

- `bridge/sitl_mavlink_connection.py`
  - Compatibility export surface.
- `bridge/sitl_mavlink_endpoint.py`
  - Disabled endpoint detection and default servo endpoint construction.
- `bridge/sitl_mavlink_imports.py`
  - `pymavlink.mavutil` dependency loading.
- `bridge/sitl_mavlink_servo_connection.py`
  - Servo/telemetry MAVLink connection setup and JSON-servo fallback log.
- `bridge/sitl_mavlink_command_connection.py`
  - Dedicated command-link setup, unavailable-link handling, and reconnect
    cadence.
- `bridge/sitl_mavlink_heartbeat.py`
  - GCS heartbeat send and heartbeat timestamp synchronization.

## Contract Notes

- Disabled servo endpoint values still disable MAVLink servo input and keep JSON
  UDP servo packets as the only servo source.
- Missing servo endpoint still defaults to
  `udpin:0.0.0.0:${ROS2_UUV_SITL_MAV_PORT:-14660}`.
- Servo link still configures both telemetry observer and servo command-link
  endpoint before connecting.
- Dedicated command output still falls back to the servo link when disabled.
- Command-link reconnect still uses the 2 second minimum interval.
- Heartbeat sends still sync both command-link and servo-link heartbeat
  timestamps.

## Verification

```text
sitl mavlink connection split smoke: PASS
python3 -m compileall -q sim/current uuv_control_gui.py: PASS
git diff --check: PASS
check_runtime_freshness.py --warn-only: WARN current-dirty, dirty_paths=679, active_runtime_dirty_paths=661
refactor_inventory.py: bridge/sitl_mavlink_connection.py removed from top 60
```
