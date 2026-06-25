# ROS2 Command Shaping Split

Date: 2026-06-07

## Scope

Split `bridge/ros2_command_shaping.py` into focused input-path modules while
preserving the command callback symbols exported through
`bridge/ros2_bridge_commands.py`.

## Files

- `bridge/ros2_command_shaping.py`
  - Compatibility export surface.
- `bridge/ros2_direct_command_filter.py`
  - Deadband, slew limiting, direct command callback dispatch, and direct command
    clear behavior.
- `bridge/ros2_cmd_vel_input.py`
  - `/cmd_vel` parsing and SITL guided body-velocity setpoint forwarding.
- `bridge/ros2_manual_control_input.py`
  - MAVROS manual-control parsing and forwarding.

## Contract Notes

- SITL closed-loop mode still blocks direct MuJoCo command callbacks unless
  direct commands are explicitly enabled.
- `/cmd_vel` still stays blocked in SITL closed-loop mode unless
  `_sitl_cmd_vel_setpoint_enabled` is true.
- Local `/cmd_vel` fallback keeps the original sign mapping:
  `(fwd, left, -yaw, -up)`.
- MAVROS manual control still forwards to SITL when transport exists and falls
  back to normalized local commands otherwise.

## Verification

```text
ros2 command shaping split smoke: PASS
python3 -m compileall -q uuv_mujoco/current uuv_control_gui.py: PASS
audit_code_contract_sources.py: fail=0 pass=11 warn=5
check_runtime_readiness_policy.py: PASS
verify_ardusub_thruster_contract.py --quiet: PASS
physics_contract_audit.py --simulate-s 0.05: PASS
git diff --check: PASS
refactor_inventory.py: bridge/ros2_command_shaping.py removed from top 60
```
