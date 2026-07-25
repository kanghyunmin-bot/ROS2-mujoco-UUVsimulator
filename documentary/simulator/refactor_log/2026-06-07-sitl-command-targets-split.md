# SITL command target split

Date: 2026-06-07

## Scope

- Added focused MAVLink command target modules:
  - `bridge/sitl_command_links.py`
  - `bridge/sitl_vehicle_heartbeat.py`
  - `bridge/sitl_mavlink_peer.py`
- Kept `bridge/sitl_command_targets.py` as the compatibility export surface
  consumed by `bridge/sitl_commanding.py` and `SitlTransport`.

## Contract

RC override readiness, command-link selection, MAVLink target resolution,
vehicle HEARTBEAT state tracking, COMMAND_ACK logging, and UDP peer discovery
retain the existing method names bound on `SitlTransport`.

## Verification

```text
python3 -m compileall -q sim/current/bridge/sitl_command_targets.py sim/current/bridge/sitl_command_links.py sim/current/bridge/sitl_vehicle_heartbeat.py sim/current/bridge/sitl_mavlink_peer.py sim/current/bridge/sitl_commanding.py
PYTHONPATH=sim/current python3 - <<'PY'
from bridge import sitl_commanding, sitl_command_targets
required = ['mavlink_connected', 'rc_override_ready', '_mav_for_commands', '_resolve_mav_target', '_heartbeat_is_vehicle', '_update_vehicle_heartbeat', '_handle_command_ack', '_ensure_mavlink_peer']
missing = [name for name in required if not hasattr(sitl_commanding, name) or not hasattr(sitl_command_targets, name)]
print({'missing': missing})
PY
```

Smoke result: `missing=[]`.  The former `bridge/sitl_command_targets.py`
hotspot no longer appears in the top 30 inventory.
