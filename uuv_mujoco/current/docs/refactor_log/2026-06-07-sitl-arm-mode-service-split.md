# SITL Arm/Mode Pending Service Split

Date: 2026-06-07

## Scope

Split `bridge/sitl_arm_mode_service.py` into focused pending-state predicates,
MAVLink broadcast helpers, and retry-loop policy without changing pending
arm/mode retry cadence or neutral-RC priming.

## Files

- `bridge/sitl_arm_mode_service.py`
  - Compatibility export surface.
- `bridge/sitl_arm_mode_service_state.py`
  - Pending target extraction, target-reached checks, timeout checks, and resend
    cadence checks.
- `bridge/sitl_arm_mode_service_send.py`
  - Broadcast pending arm/mode requests to all eligible command links and send
    neutral RC after arm requests.
- `bridge/sitl_arm_mode_service_pending.py`
  - Top-level pending arm/mode service loops.

## Contract Notes

- Pending arm target timeout remains 90 seconds.
- Pending arm resend period remains 1 second.
- Pending mode timeout remains 30 seconds.
- Pending mode resend period remains 0.5 seconds.
- Arm resend still sends neutral RC after an arm request.
- Pending targets are still cleared only when reached or timed out.

## Verification

```text
sitl pending arm/mode service split smoke: PASS
python3 -m compileall -q uuv_mujoco/current uuv_control_gui.py: PASS
git diff --check: PASS
check_runtime_freshness.py --warn-only: WARN current-dirty, dirty_paths=657, active_runtime_dirty_paths=639
refactor_inventory.py: bridge/sitl_arm_mode_service.py removed from top 45
```
