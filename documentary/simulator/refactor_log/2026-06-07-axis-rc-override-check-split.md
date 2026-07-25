# Axis RC Override Check Split

Date: 2026-06-07

Scope: active runtime tooling under `sim/current/tools`.

## Change

- Split `tools/axis_rc_override_check.py` into:
  - `axis_rc_contract.py`
  - `axis_rc_metrics.py`
  - `axis_rc_plotting.py`
  - `axis_rc_node.py`
  - `axis_rc_override_check.py`
- Kept the original executable name as the CLI entry point.
- Moved ROS2/rclpy imports into the actual run path.
- Moved matplotlib plotting imports into output generation.

## Contract Notes

- RC mapping remains `RC1=pitch`, `RC2=roll`, `RC3=heave`, `RC4=yaw`,
  `RC5=forward`, `RC6=lateral`.
- The diagnostic still validates `/mavros/rc/out`, DVL, IMU, depth, local
  odometry, arm state, and mode state.
- No SITL parameter, PWM remap, controller shim, or plant input path changed.

## Validation

```text
python3 -m py_compile sim/current/tools/axis_rc_override_check.py \
  sim/current/tools/axis_rc_contract.py \
  sim/current/tools/axis_rc_metrics.py \
  sim/current/tools/axis_rc_plotting.py \
  sim/current/tools/axis_rc_node.py
PYTHONPATH=sim/current/tools python3 sim/current/tools/axis_rc_override_check.py --help
PYTHONPATH=sim/current/tools python3 <axis non-ROS helper smoke>
python3 sim/current/tools/refactor_inventory.py --limit 30 --format markdown
```

Results:

- Compile passed.
- Non-ROS helper smoke passed.
- CLI help passed without importing ROS2/rclpy or matplotlib.
- `tools/axis_rc_override_check.py` dropped out of the top 30 hotspot list.
- The remaining ROS node module is `tools/axis_rc_node.py` at
  `330 LOC / 63` branches.
