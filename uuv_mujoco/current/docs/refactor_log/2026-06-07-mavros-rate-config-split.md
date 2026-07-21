# MAVROS Rate And Policy Config Split

Date: 2026-06-07

## Scope

Split `bridge/ros2_bridge_config_mavros_rates.py` into focused state, sensor
rate, RCOUT policy, battery, and replay bookkeeping modules while preserving
the compatibility import surface used by `bridge/ros2_bridge_config_mavros.py`.

## Files

- `bridge/ros2_bridge_config_mavros_rates.py`
  - Compatibility export surface and top-level assembly.
- `bridge/ros2_bridge_config_mavros_state.py`
  - MAVROS mode/armed state defaults and state-publish cadence.
- `bridge/ros2_bridge_config_mavros_sensor_rates.py`
  - Real-robot ROS/MAVROS surface rate defaults and logging.
- `bridge/ros2_bridge_config_mavros_rcout.py`
  - RCOUT header-stamp and publish-mode policy.
- `bridge/ros2_bridge_config_mavros_battery.py`
  - MAVROS battery defaults.
- `bridge/ros2_bridge_config_mavros_replay.py`
  - RC override / RCOU replay bookkeeping defaults.

## Contract Notes

- No MAVROS topic, RCOUT publish mode fallback, header stamp source default,
  sensor-rate environment variable name, real-robot rate default, battery
  default, or replay handler field name was changed.
- `configure_mavros_state_and_rates()` still applies state defaults, rates,
  RCOUT policy, rate log, battery defaults, and replay state in the same order.

## Verification

```text
mavros rates config split smoke: PASS
python3 -m compileall -q uuv_mujoco/current uuv_control_gui.py: PASS
git diff --check: PASS
check_runtime_freshness.py --fetch --refresh-version --warn-only: WARN current-dirty, dirty_paths=704, active_runtime_dirty_paths=686
refactor_inventory.py: bridge/ros2_bridge_config_mavros_rates.py removed from top 35
```
