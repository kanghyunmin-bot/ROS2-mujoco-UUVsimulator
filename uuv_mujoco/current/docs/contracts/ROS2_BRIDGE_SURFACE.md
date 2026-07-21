# ROS2 Bridge Surface Contract

Scope: active runtime bridge class `bridge/ros2_bridge.py`.

This bridge intentionally publishes only the ROS2 topics needed for the real
robot compatibility path and simulator diagnostics.  It does not emulate every
MAVROS plugin topic.

## Design Goals

1. Publish only the ROS2 topics that matter for the real robot bringup.
2. Preserve the public class/signature expected by the simulator.
3. Keep MAVROS-facing topic names compatible with the real robot stack.
4. Avoid heavyweight image/rendering/registry logic unless it is explicitly
   added back later.

## Core Simulator Topics

- `/imu/data`
- `/depth`
- `/bar30/pressure_pa`
- `/dvl/velocity`
- `/dvl/altitude`
- `/dvl/odometry`
- `/dvl/data`
- `/dvl/position`
- `/rovio/odometry`
- `/odometry/filtered`
- `/sim/odom`
- `/mujoco/sim_time`
- `/mujoco/ground_truth/pose`
- `/mujoco/course_buoys/status`
- `/mujoco/hydrophone/status`
- `/mujoco/hydrophone/direction`
- `/audio`
- `/audio_info`
- `/ping360/scan_image`
- `/ping360/scan_echo`
- `/ping360/echo`
- `/tf`
- `/tf_static`
- `/robot_description`

## Real Robot Compatibility / MAVROS Surface

Full surface mode publishes:

- `/mavros/state`
- `/mavros/imu/data`
- `/mavros/imu/data_raw`
- `/mavros/imu/static_pressure`
- `/mavros/imu/atm_pressure`
- `/mavros/vfr_hud`
- `/mavros/local_position/pose`
- `/mavros/local_position/odom`
- `/mavros/local_position/velocity_local`
- `/mavros/local_position/velocity_body`
- `/mavros/local_position/velocity_body_cov`
- `/mavros/vision_pose/pose`
- `/mavros/battery`
- `/mavros/rc/in`
- `/mavros/rc/out`

When `enable_mavros_surface=False`, `/mavros/vfr_hud` remains as the
compatibility-only MAVROS output.

## Subscriptions And Services

- `/cmd_vel` (`TwistStamped`)
- `/mavros/rc/override`
- `/mavros/setpoint_raw/local`
- `/mavros/cmd/arming`
- `/mavros/set_mode`

## Sensor Notes

- Stereo camera image publishing is removed for performance.
- Ping360 publishes a lightweight on-demand sonar image only when a subscriber
  is present.
- The synthetic hydrophone uses the course pinger buoy site as a 22.53 kHz
  acoustic source by default and mixes configurable near-frequency interferers
  into `/audio`. Ten interferers are enabled by default in the 22-23 kHz band:
  eight attached to thruster sites and the remainder at deterministic random
  pool positions. `/mujoco/hydrophone/status` reports the active source list.
- `/mavros/imu/static_pressure` defaults to external Bar30-derived pressure.
- `/mavros/imu/atm_pressure` is kept as the observed real-robot MAVROS
  auxiliary field value; in the April 1 bag it is about `0.24` and is not a
  Pa-scale surface pressure reference.
- `/dvl/data` and `/dvl/position` are published best-effort using installed
  `dvl_msgs` definitions when present. Unknown fields are ignored safely.
