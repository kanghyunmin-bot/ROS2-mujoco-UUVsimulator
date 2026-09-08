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

`/sim/odom` is a simulation-only ground-truth diagnostic/oracle. Despite its
`nav_msgs/Odometry` type, it is forbidden as an estimator, SLAM, MAVROS, or
controller input and must not be scored as an estimate. Quantitative evaluation
should normally use the explicitly evaluation-only
`/mujoco/ground_truth/pose` topic.

The bridge does not publish `/odometry/filtered` by default, including in plain
`--ros2` mode. That standard localization topic belongs to an external estimator.
For an old integration that cannot yet migrate, the exact-state alias can be
temporarily restored with the deliberately alarming opt-in below:

```bash
./launch_uuv_sim.sh --ros2 \
  --unsafe-legacy-ground-truth-odometry-filtered
# Equivalent environment opt-in:
ROS2_UUV_UNSAFE_LEGACY_GROUND_TRUTH_ODOMETRY_FILTERED=1 \
  ./launch_uuv_sim.sh --ros2
```

This prints a startup error-level warning because the resulting
`/odometry/filtered` is exact MuJoCo state, not an estimate. It is forbidden for
SLAM, state estimation, control, and research metrics. The opt-in is rejected
when `--ros2-real-pkg-compat` is active so the external estimator remains the
only possible owner.

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

When `enable_mavros_surface=False` by itself, the bridge publishes no MAVROS
surface. In strict ROS+SITL real-package compatibility mode, the additional
strict sensor transport creates only `/mavros/imu/data_raw` and
`/mavros/imu/static_pressure`. External ArduSub-to-MAVROS owns the fused-AHRS
`/mavros/imu/data` topic as well as the remaining command, state, and
local-position topics. This keeps one public publisher per sensor topic while
preserving the real downstream names.

## Subscriptions And Services

- `/cmd_vel` (`TwistStamped`)
- `/mavros/rc/override`
- `/mavros/setpoint_raw/local`
- `/mavros/cmd/arming`
- `/mavros/set_mode`

## Sensor Notes

- Stereo camera image publishing remains opt-in and on-demand for performance.
  `ROS2_UUV_CAMERA_SENSOR_MODEL_ENABLE=1` inserts the deterministic underwater
  optical/electronic/timing model at the renderer-to-ROS boundary; it remains
  disabled by default for legacy parity. See
  `UNDERWATER_CAMERA_SENSOR_MODEL.md` for calibration status and limitations.
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
- IMU and Bar30 captures are modeled independently. The IMU includes
  cross-axis/scale coupling, turn-on bias, Gauss-Markov bias instability,
  random walk, bandwidth-scaled white noise, saturation, and quantization.
  Bar30 includes gauge-pressure scale/offset, drift, random walk, thermal
  response, white noise, range saturation, and quantization.
- A captured IMU/Bar30 sample reaches the FCU-facing SITL JSON plant feed
  immediately. That mandatory 400 Hz JSON frame is sample-and-hold input for
  the ArduSub controller only; it is not the SLAM raw-sensor transport and its
  frame timestamp must not be reinterpreted as a new hardware capture.
- In strict SITL real-package mode, external ArduSub-to-MAVROS owns fused AHRS
  `/mavros/imu/data`. The bridge owns only `/mavros/imu/data_raw` and
  `/mavros/imu/static_pressure` with sensor-data best-effort QoS. The sim
  MAVROS node remains connected for FCU commands, state, local position, and
  fused attitude; only its JSON-backend raw IMU and pressure copies are
  quarantined under `/uuv_mujoco/mavros_fcu_passthrough/` to guarantee single
  publisher ownership.
- The strict raw topics are delivery-driven: independent device clock,
  processing latency, transport latency, dropout, and bounded queues decide
  whether a capture is published. Messages retain capture-time headers,
  `fcu_link`, and modeled covariance. `/mavros/imu/data_raw` carries the
  modeled accelerometer and gyroscope only: its quaternion is identity and
  `orientation_covariance[0]` is `-1` (all other orientation covariance terms
  are zero), matching the physical bag's "orientation unavailable" contract.
  Every newly arrived capture is published once, including a batch after a
  delayed bridge update; an empty delivery batch never republishes the
  previous sample.
- The default profile is
  `config/sensor_models/imu_bar30_uncalibrated_prior.json`. The FCU IMU part
  number remains unresolved, so all error values are explicitly unvalidated
  priors pending Allan-deviation, fixture, pressure-gauge, and clock tests.
- The real-robot DVL contract is `auv_dvl_a50_msg`, not the older local
  `dvl_msgs` package. Legacy fallback is available only outside strict mode.
- A50 velocity remains FRD in `dvl_link`. The `base_link -> dvl_link` X-pi TF
  performs the single FRD/FLU conversion; the twist bridge does not flip Y/Z.
- The A50 model raycasts four 22.5-degree beams independently, so slopes,
  steps, obstacles, range loss, and incidence loss affect each beam. Sensor
  velocity includes the DVL lever arm (`omega x r`).
- Sensor timing separates capture, device transmission, and host arrival.
  Noise, clock drift, jitter, beam dropout, packet dropout, bounded queues,
  and 5 Hz dead-reckoning reports are deterministic from the configured seed.
  Velocity and position use independent schedules, queues, dropout streams,
  and cross-stream arrival ordering. Losing a velocity report therefore does
  not delay or burst the 5 Hz position stream.
- Bottom-lock loss freezes measured position but continues growing the
  dead-reckoning uncertainty. A reset removes in-flight old-generation
  position reports while preserving the capture phase and random streams.
- Direct diagnostic mode matches the physical driver's raw-topic boundary:
  `/dvl/data` and `/dvl/position` use sensor-data best-effort QoS and host
  arrival-time headers. The embedded validity/transmission fields retain the
  modeled device times, and `DVL.time` includes packet-loss gaps.
- The shipped noise values are explicitly unvalidated priors. Fit them from a
  synchronized real bag before claiming quantitative sim-to-real accuracy.

## Physical A50 Driver Boundary

Direct ROS mode remains available for diagnostics. For sim-to-real work, use
the TCP device mode so the unmodified physical driver performs connection,
JSON parsing, QoS, and ROS publication:

```bash
export ROS2_UUV_DVL_DEVICE_EMULATOR_ENABLE=1
cd uuv_mujoco/current
./start_sitl_mujoco_mj311.sh --ros2-real-pkg-compat
```

In the same ROS environment, start the real driver in one terminal:

```bash
ros2 launch hit25_auv_ros2 dvl_a50_driver.launch.py \
  ip_address:=127.0.0.1 \
  velocity_frame_id:=dvl_link \
  position_frame_id:=dvl_link \
  use_sim_time:=true
```

Then start the shared converter in a second terminal after sourcing the same
ROS workspace. The launch command above remains in the foreground by design:

```bash
ros2 run hit25_auv_ros2 dvl_to_twist_bridge \
  --ros-args -p output_frame_id:=dvl_link
```

The wrapper runs the unmodified real driver but explicitly puts its
host-receipt headers on `/clock` in simulation. It defaults to
`use_sim_time:=false` on the physical vehicle. A50 measurement time is kept
separately in `time_of_validity`, and transmission time in
`time_of_transmission`; SLAM synchronization must use those device fields only
after estimating the device-to-host clock relationship.

The emulated device advances before the ROS publication guard. An unrelated
ROS publisher failure therefore cannot freeze a connected TCP DVL. The public
and private authoritative clocks also keep advancing after such a failure, so
the external driver's host-receipt headers do not become stale. Constructor
rollback closes the A50 listener, SITL transport/thread, and any partially
created ROS node, executor, and context.

Build and run the offline physical-driver integration gate with:

```bash
./docker/ubuntu-dev/dev.sh build-ros
docker compose -f docker/ubuntu-dev/docker-compose.yml run --rm uuv-dev \
  bash -lc 'source /opt/ros/humble/setup.bash && \
    source /workspace/rospkg/install/setup.bash && \
    /workspace/.venv/bin/python \
      /workspace/uuv_mujoco/current/tools/check_dvl_real_driver_e2e.py'
```
