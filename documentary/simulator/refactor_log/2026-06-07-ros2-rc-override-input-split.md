# ROS2 RC Override Input Split

Date: 2026-06-07

## Scope

Split `bridge/ros2_rc_override_input.py` into focused MAVROS RC override
helpers without changing the `/mavros/rc/override` forwarding, local fallback,
or `/mavros/rc/in` mirror contract.

## Files

- `bridge/ros2_rc_override_input.py`
  - Compatibility export surface.
- `bridge/ros2_rc_override_frame.py`
  - Channel extraction, 18-channel forwarding frame construction, and
    normalized forward/sway/yaw/heave axis math.
- `bridge/ros2_rc_override_forwarding.py`
  - Locked SITL transport forwarding through `send_rc_override(...)`.
- `bridge/ros2_rc_override_mirror.py`
  - Immediate `/mavros/rc/in` mirror publication for GUI and diagnostics.
- `bridge/ros2_rc_override_warning.py`
  - Throttled forwarding-not-ready warning.
- `bridge/ros2_rc_override_callback.py`
  - Top-level callback policy that connects forwarding, local fallback, warning,
    and mirror behavior.

## Contract Notes

- Incoming `/mavros/rc/override` still forwards at most 18 integer channels to
  `SitlTransport.send_rc_override(...)`.
- When SITL transport is absent or local fallback is explicitly enabled, the
  callback still calls `_handle_normalized_cmd(...)` and mirrors the RC input.
- When SITL transport exists but forwarding fails and local fallback is disabled,
  the callback still warns and returns without publishing an `/mavros/rc/in`
  mirror.
- The split does not change ArduSub RC mapping, PWM span, inversion, fallback
  policy, or command timing.

## Verification

```text
ros2 rc override split smoke: PASS
python3 -m compileall -q sim/current uuv_control_gui.py: PASS
audit_code_contract_sources.py: fail=0 pass=11 warn=5
check_runtime_readiness_policy.py: PASS
verify_ardusub_thruster_contract.py --quiet: PASS
physics_contract_audit.py --simulate-s 0.05: PASS
git diff --check: PASS
check_runtime_freshness.py --warn-only: WARN current-dirty, dirty_paths=650, active_runtime_dirty_paths=632
refactor_inventory.py: bridge/ros2_rc_override_input.py removed from top 45
```
