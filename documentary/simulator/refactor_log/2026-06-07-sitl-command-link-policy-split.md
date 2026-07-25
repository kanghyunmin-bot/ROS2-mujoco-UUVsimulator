# SITL Command-Link Policy Split

Date: 2026-06-07

## Scope

Split `bridge/sitl_command_links.py` into focused command-link helper modules
without changing the public method names bound onto `SitlTransport`.

## Files

- `bridge/sitl_command_links.py`
  - Compatibility export surface for existing imports.
- `bridge/sitl_command_link_select.py`
  - Command MAVLink object selection, arm/mode command pending state, and
    command-link object mapping.
- `bridge/sitl_command_link_readiness.py`
  - `mavlink_connected` and `rc_override_ready` property gates.
- `bridge/sitl_command_target_resolution.py`
  - MAVLink system/component target resolution and assignment.

## Contract Notes

- `_mav_for_commands()` and `_mav_for_external_nav()` still prefer the dedicated
  command link when present, then fall back to the servo MAVLink link.
- `_mavs_for_arm_mode_commands()` still returns de-duplicated command/servo
  MAVLink objects in the same order.
- `_resolve_mav_target()` still prefers explicit positive
  `_sitl_mavlink_target_sysid/_compid`, then falls back to the matching link
  heartbeat and finally servo heartbeat fallback.
- `rc_override_ready` still requires a resolvable target plus a fresh heartbeat
  on the selected command path.

## Verification

```text
sitl command link split smoke: PASS
python3 -m compileall -q sim/current uuv_control_gui.py: PASS
audit_code_contract_sources.py: fail=0 pass=11 warn=5
check_runtime_readiness_policy.py: PASS
verify_ardusub_thruster_contract.py --quiet: PASS
physics_contract_audit.py --simulate-s 0.05: PASS
git diff --check: PASS
refactor_inventory.py: bridge/sitl_command_links.py removed from top 60
```
