# ROS2 MAVROS Arm/Mode Service Split

Date: 2026-06-07

## Scope

Split `bridge/ros2_mavros_arm_mode_services.py` into focused guard, transport,
forwarding, and ROS callback helpers without changing the MAVROS service
response or SITL arm/mode forwarding contract.

## Files

- `bridge/ros2_mavros_arm_mode_services.py`
  - Compatibility export surface.
- `bridge/ros2_mavros_arm_mode_guard.py`
  - Boot-guard and `ROS2_UUV_MAVROS_FORWARD_ARM_MODE` rejection logic.
- `bridge/ros2_mavros_arm_mode_transport.py`
  - Locked `SitlTransport.send_arm_command(...)` and `send_set_mode(...)`
    calls.
- `bridge/ros2_mavros_arm_mode_forwarding.py`
  - Bridge-level arm/mode forwarding policy and MAVROS state updates.
- `bridge/ros2_mavros_arm_mode_callbacks.py`
  - `/mavros/cmd/arming` and `/mavros/set_mode` response population.

## Contract Notes

- Arming requests during boot guard are still rejected only when requesting
  `armed=True`.
- Non-`MANUAL` mode requests during boot guard are still rejected.
- `ROS2_UUV_MAVROS_FORWARD_ARM_MODE=0` still rejects both arm and mode
  forwarding.
- Missing SITL transport still counts as success, matching the previous bridge
  compatibility behavior.
- Service responses still set `success`, `result`, and `mode_sent` with the
  same rules as before the split.

## Verification

```text
ros2 mavros arm/mode split smoke: PASS
python3 -m compileall -q sim/current uuv_control_gui.py: PASS
audit_code_contract_sources.py: fail=0 pass=11 warn=5
check_runtime_readiness_policy.py: PASS
verify_ardusub_thruster_contract.py --quiet: PASS
physics_contract_audit.py --simulate-s 0.05: PASS
git diff --check: PASS
check_runtime_freshness.py --warn-only: WARN current-dirty, dirty_paths=654, active_runtime_dirty_paths=636
refactor_inventory.py: bridge/ros2_mavros_arm_mode_services.py removed from top 45
```
