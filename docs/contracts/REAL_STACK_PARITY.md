# Real Stack Parity Contract

`--ros2-real-pkg-compat` is the strict integration mode. It uses the real
`mavros_node`, the `hit25_auv_ros2` sensor bridges, and `robot_localization`.
The simulator only owns raw sensors, simulation diagnostics, and ground truth.

## Topic ownership

| Surface | Owner |
| --- | --- |
| `/dvl/data`, `/dvl/position` | physical `auv_dvl_a50`; A50 TCP emulator in simulation |
| camera, `/battery`, Ping360 | `uuv_mujoco_bridge` |
| `/collector/state` | physical collector driver; MuJoCo physical-event adapter in simulation |
| `/mavros/imu/data` | physical and strict-simulation `mavros_node`, fused ArduSub AHRS |
| `/mavros/imu/data_raw`, `/mavros/imu/static_pressure` | physical `mavros_node`; delivery-driven `uuv_mujoco_bridge` in strict simulation |
| remaining `/mavros/*` | external `mavros_node` |
| `/dvl/twist`, `/depth/pose` | `hit25_auv_ros2` bridges |
| `/odometry/filtered`, `odom -> base_link` | `robot_localization` |
| `/mujoco/ground_truth/*`, `/sim/*` | `uuv_mujoco_bridge`, evaluation only |
| `/mavros/rc/override` | active controller using the physical launch policy |

The strict command path is `/mavros/rc/override -> mavros_node -> ArduSub ->
UDP 9002 -> MuJoCo`. `/cmd_vel` and `/uuv_mujoco/sitl/command_override` do not
control the strict-mode plant.

The physical default is direct `/mavros/rc/override` output. A mux is optional
for explicitly configured multi-controller experiments; it is not a required
physical-robot interface. This document covers the base ROV transport rather
than synchronization of competition algorithms.

## Upstream repository audit

The reference organization is [2026-kmu-underwater-robot](https://github.com/2026-kmu-underwater-robot).
The base ROV comparison was refreshed on 2026-09-08 against `auv`
(`756d1a412c9a4804ae9384b6ded18245969bddec`) and `auv_mavros`
(`d717e98c4d6b83db0d44b21880d230ef37bd29f9`). Local directory names
such as `kmu26_auv` are retained for compatibility and are not current upstream
repository names. The local ROS implementation package remains `hit25_auv_ros2`;
`auv` provides a compatible `rov_start.launch.py` entry point.

| Repository | Base ROV interface |
| --- | --- |
| `auv` | MAVROS bringup, DVL/depth bridges, EKF and sensor rate configuration |
| `auv_dvl_a50` | A50 TCP driver for `/dvl/data` and `/dvl/position` |
| `auv_dvl_a50_msg` | DVL message schemas |
| `auv_mavros` | MAVROS transport, including custom `vision_position_delta` |

The stock MAVROS installation needs the bundled `auv_mavros_dvl_plugin`
for the custom DVL message path. When using the full organization MAVROS fork,
exclude this supplemental package to avoid duplicate plugin exports.
Competition controllers and mission handshakes are outside this baseline's
compatibility claim. This is not a claim that every upstream executable or
launch file is available through the local `auv` adapter.

## Canonical launch

Run each foreground command from the repository root in its own terminal.

```bash
export ROS2_UUV_DVL_DEVICE_EMULATOR_ENABLE=1
export ROS2_UUV_CAMERA_SENSOR_MODEL_ENABLE=1
./uuv_mujoco/start_sitl_mujoco.sh --ros2-real-pkg-compat -- \
  --scene scenes/research_pool_slam_scene.xml \
  --profile research_pool --fluid-model current \
  --ros2-images --ros2-image-width 1280 --ros2-image-height 720 \
  --ros2-image-hz 30
```

```bash
source /opt/ros/humble/setup.bash
source rospkg/install/setup.bash
ros2 launch auv rov_start.launch.py \
  use_sim_time:=true \
  fcu_url:=udp://0.0.0.0:14551@ \
  use_dvl:=true dvl_ip:=127.0.0.1 \
  use_joy2mavros:=false use_battery_bridge:=false \
  use_odom2mavros:=false publish_static_tf:=false \
  use_web_gui:=false use_rviz:=false use_mission_rviz_visualizer:=false
```

```bash
source .uuv_mujoco_env.sh
"$MJ311_PYTHON" uuv_mujoco/current/tools/check_strict_real_pkg_surface.py
"$MJ311_PYTHON" uuv_mujoco/current/tools/check_real_sim_sensor_contract.py
```

`/mujoco/ground_truth/*` and `/sim/odom` are simulation-only ground-truth test or
visualization oracles. An estimator, SLAM node, MAVROS component, or mission
controller subscribing to them is a parity failure. The MuJoCo bridge leaves
the standard `/odometry/filtered` topic unowned by default; in strict mode only
the external estimator may publish it.

## FCU IMU and Bar30 boundary

The IMU and Bar30 measurement profile is
`uuv_mujoco/current/config/sensor_models/imu_bar30_uncalibrated_prior.json`.
Each sensor has an independent fixed-rate capture clock, device clock,
processing/transport latency, packet-drop stream, and bounded queue. Captured
measurements are applied immediately to the mandatory 400 Hz ArduSub JSON/SITL
plant frame as sample-and-hold FCU inputs. That plant cadence is not a new
sensor capture. In strict simulation, external ArduSub-to-MAVROS owns the
fused-AHRS `/mavros/imu/data` topic. `uuv_mujoco_bridge` is the sole public
publisher of `/mavros/imu/data_raw` and `/mavros/imu/static_pressure`; it
publishes every delivered packet once after the modeled host arrival, with
capture-time headers and sensor-data QoS. Raw IMU orientation is identity with
`orientation_covariance[0] = -1` and every other orientation covariance term
zero; its modeled accelerometer and gyroscope values are preserved. The
external sim MAVROS node remains the command/state/local-position and AHRS
owner; only its FCU-derived raw IMU and pressure copies are quarantined below
`/uuv_mujoco/mavros_fcu_passthrough/`. On the vehicle, physical MAVROS remains
the owner of all `/mavros/*` topics.

IMU errors include cross-axis/scale coupling, turn-on bias, first-order
Gauss-Markov bias instability, random walk, bandwidth-scaled white noise,
saturation, and quantization. Bar30 errors include gauge-pressure scale,
turn-on/constant offset, bias instability, random walk, white noise, thermal
offset response, the 30 bar limit, and pressure quantization. ROS packet loss
does not remove a measurement already consumed inside the FCU.

The flight-controller IMU part number is not pinned in the real repository.
The Bar30 identity is inferred from the real stack name and must still be
checked against installed inventory. Consequently every stochastic value is
marked `unvalidated_prior`; do not present it as measured hardware behavior
until Allan-deviation, fixture, reference-gauge, clock, and packet tests have
been fitted and held-out bags validate the result.

For the DVL hardware-in-the-loop boundary, enable the localhost A50 protocol
emulator and run the unmodified `auv_dvl_a50` driver. The simulator suppresses
its duplicate raw ROS publishers in this mode:

```bash
export ROS2_UUV_DVL_DEVICE_EMULATOR_ENABLE=1
export ROS2_UUV_DVL_DEVICE_EMULATOR_HOST=127.0.0.1
export ROS2_UUV_DVL_DEVICE_EMULATOR_PORT=16171
```

The modeled velocity stream is 10 Hz and the modeled local-position stream is
independently 5 Hz. Each has its own transport/dropout state; reset commands
atomically invalidate queued pre-reset positions. The driver remains the ROS
topic owner and publishes both raw topics with sensor-data best-effort QoS.
Launch it through `hit25_auv_ros2/dvl_a50_driver.launch.py`: `rov_start` passes
`use_sim_time` into that wrapper, so device-mode headers share `/clock` with
the rest of the simulated stack while the physical default remains wall time.
The authoritative clock is isolated from optional ROS publisher health so a
failed visualization/diagnostic topic cannot freeze external-driver stamps.

## Physical IMX219 calibration and extrinsics

The physical dual-IMX219 launch publishes image frame IDs, but it cannot infer
either camera's installed pose. `rov_start.launch.py` therefore defaults
`publish_imx219_static_tf:=false` and prints a warning while physical cameras
are active without TF. Do not feed those images to localization or SLAM in that
state. The packaged example calibration is also only a placeholder: calibrate
each installed camera in water and pass both measured YAML files through
`rov_start.launch.py` before using either stream for SLAM.

Measure both transforms from `base_link` to the installed optical frames, in
metres and radians, then supply all twelve values and explicitly enable them:

```bash
ros2 launch auv rov_start.launch.py \
  camera0_calibration_file:="$CAM0_CALIBRATION_YAML" \
  camera1_calibration_file:="$CAM1_CALIBRATION_YAML" \
  publish_imx219_static_tf:=true \
  imx219_camera0_x:="$CAM0_X" imx219_camera0_y:="$CAM0_Y" \
  imx219_camera0_z:="$CAM0_Z" imx219_camera0_roll:="$CAM0_ROLL" \
  imx219_camera0_pitch:="$CAM0_PITCH" imx219_camera0_yaw:="$CAM0_YAW" \
  imx219_camera1_x:="$CAM1_X" imx219_camera1_y:="$CAM1_Y" \
  imx219_camera1_z:="$CAM1_Z" imx219_camera1_roll:="$CAM1_ROLL" \
  imx219_camera1_pitch:="$CAM1_PITCH" imx219_camera1_yaw:="$CAM1_YAW"
```

The zero-valued launch defaults are inactive placeholders, not measured
extrinsics. The new TF path is physical-only; simulation continues to use the
MuJoCo camera sites and the bridge-owned camera TF chain.
