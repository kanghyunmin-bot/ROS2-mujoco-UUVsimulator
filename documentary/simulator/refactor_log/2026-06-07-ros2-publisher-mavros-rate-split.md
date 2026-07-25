# ROS2 Publisher And MAVROS Rate Split

Date: 2026-06-07

Scope: active runtime `sim/current`.

Changed ownership:

- `bridge/ros2_endpoint_publishers.py` remains the public publisher facade.
- `bridge/ros2_endpoint_core_publishers.py` owns simulator sensor and SITL status publishers.
- `bridge/ros2_endpoint_ping360_publishers.py` owns Ping360 image/scan/echo/status publishers.
- `bridge/ros2_endpoint_mavros_publishers.py` owns MAVROS-compatible telemetry publishers and disabled-surface reset state.
- `bridge/ros2_endpoint_misc_publishers.py` owns DVL compatibility and TF publishers.
- `bridge/ros2_bridge_config_mavros.py` now owns RC mapping and setpoint configuration.
- `bridge/ros2_bridge_config_mavros_rates.py` owns sensor-rate defaults, RCOUT publish policy, battery defaults, and replay state.

Validation:

- Focused compileall for the split endpoint/config modules passed.
- `ros2_endpoint_publishers_smoke PASS`
- `mavros_config_rates_smoke PASS`
- Refactor inventory no longer lists `bridge/ros2_endpoint_publishers.py` or `bridge/ros2_bridge_config_mavros.py` as top hotspots.

Contract note:

- This is behavior-neutral. Topic names, optional message handling, MAVROS disabled-state `None` assignments, real-robot sensor-rate defaults, and RCOUT publish-mode fallback are preserved.
