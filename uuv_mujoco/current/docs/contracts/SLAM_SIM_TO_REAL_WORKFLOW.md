# SLAM Sim-to-Real Workflow

This workflow keeps the estimator identical between MuJoCo, another simulator,
and the physical AUV. Only the raw-sensor or device-protocol provider changes.

## Runtime Boundary

1. A simulator or physical driver owns raw sensor/device data.
2. The same ROS message types, frames, QoS, clocks, converters, and SLAM launch
   files run downstream.
3. `/mujoco/ground_truth/pose` is recorded by an evaluation process only. It is
   forbidden as an input to SLAM, state estimation, MAVROS, or control.

In strict SITL real-package mode, ArduSub still receives the mandatory 400 Hz
JSON plant frame so its scheduler and controller remain unchanged. The IMU and
pressure values in that frame are sample-and-hold FCU inputs, not 400 Hz SLAM
captures. External ArduSub-to-MAVROS owns `/mavros/imu/data`, so its quaternion
comes from the FCU AHRS rather than MuJoCo noisy ground truth. MuJoCo owns only
`/mavros/imu/data_raw` and `/mavros/imu/static_pressure` and publishes them
when modeled packets arrive, with capture-time headers and sensor-data
best-effort QoS. Raw IMU orientation is explicitly unavailable: the quaternion
is identity, `orientation_covariance[0]` is `-1`, and the remaining orientation
covariance terms are zero; modeled accelerometer and gyroscope values remain
unchanged. Only the sim MAVROS raw-IMU and pressure copies are available for
diagnosis under `/uuv_mujoco/mavros_fcu_passthrough/`; never bag those private
topics as SLAM inputs.

The Water Linked A50 path should use the TCP device emulator plus the physical
`auv_dvl_a50` driver. The driver is launched through
`ros2 launch hit25_auv_ros2 dvl_a50_driver.launch.py use_sim_time:=true` in
simulation and the physical default `false` on the vehicle.

Physical IMX219 images are not SLAM-ready merely because their headers contain
optical frame IDs. Before a physical run, measure both camera extrinsics,
supply every `imx219_camera{0,1}_{x,y,z,roll,pitch,yaw}` argument to
`rov_start.launch.py`, and set `publish_imx219_static_tf:=true`. Its default is
false, and the launch warning means the camera frames remain deliberately
disconnected from `base_link`. Also pass the two measured in-water CameraInfo
YAMLs through `camera0_calibration_file` and `camera1_calibration_file`; the
default example file has zero intrinsics and is not a SLAM calibration.
Simulation retains the bridge-owned MuJoCo camera TF path.

## Are Bags Required?

A bag is not required to execute the simulator or sensor models. It is required
for quantitative claims:

- identify bias, random walk, latency, packet loss, exposure, and distortion
  distributions from the exact hardware;
- estimate device-clock to ROS-clock offset and drift;
- replay identical inputs while comparing model versions;
- keep auditable evidence for sim-to-real results.

Until fitted from synchronized real data, every shipped noise profile remains
an `unvalidated_prior`, not a calibrated sensor specification.

The current IMU/Bar30 prior lives at
`config/sensor_models/imu_bar30_uncalibrated_prior.json`. Record its hash and
the full-profile override `ROS2_UUV_IMU_BAR30_SENSOR_CONFIG_PATH`, plus all
per-sensor overrides (`ROS2_UUV_IMU_SENSOR_*` and
`ROS2_UUV_BAR30_SENSOR_*`), with every run. The FCU IMU model is unresolved in
the real repository, so first pin the flight-controller/IMU part number,
full-scale, filter, and sampling configuration. Fit IMU Allan deviation from a
long stationary capture, cross-axis/scale terms from controlled orientations
and rates, and Bar30 offset/thermal/drift terms against a reference pressure
or depth measurement. Motion-bag high-pass variation is not a substitute for
a stationary sensor-noise estimate.

## Record One Evaluation Run

Use the same topic set in simulation and on the vehicle. The real run naturally
omits `/clock` and MuJoCo ground truth.

The MuJoCo bridge does not publish `/odometry/filtered` by default, in either
plain `--ros2` or `--ros2-real-pkg-compat` mode. In simulation, record an
estimate only in `--ros2-real-pkg-compat` mode and only after starting
`robot_localization` or the actual SLAM estimator. That external estimator must
be the sole publisher of `/odometry/filtered`. Verify topic ownership before
every run:

```bash
ros2 topic info /odometry/filtered --verbose
```

The reported publisher must be the external estimator, not
`uuv_mujoco_bridge`. The option
`--unsafe-legacy-ground-truth-odometry-filtered` (or its
`ROS2_UUV_UNSAFE_LEGACY_GROUND_TRUTH_ODOMETRY_FILTERED=1` equivalent) exists
only for temporary legacy compatibility outside strict mode. It exposes exact
MuJoCo state under the estimator topic, prints a strong warning, and is invalid
for SLAM, control, or metrics. Strict mode rejects that opt-in. Do not bypass
the recorder's frame and publisher-ownership guards.

```bash
ros2 bag record \
  /clock /tf /tf_static \
  /mavros/imu/data /mavros/imu/data_raw /mavros/imu/static_pressure \
  /dvl/data /dvl/position /dvl/twist /dvl/odometry \
  /depth/pose \
  /imx219/camera0/image_raw /imx219/camera0/image_raw/compressed /imx219/camera0/camera_info \
  /imx219/camera1/image_raw /imx219/camera1/image_raw/compressed /imx219/camera1/camera_info \
  /odometry/filtered \
  /mujoco/ground_truth/pose
```

Also store the simulator commit, sensor-profile JSON hashes, random seeds,
vehicle configuration, SLAM parameters, and device serial/firmware versions.
Those values are part of the experiment, not incidental metadata.

For a live simulation run, the evaluation-only recorder writes estimate and
truth directly in TUM form while rejecting non-monotonic/invalid stamps:

```bash
python3 tools/record_slam_trajectories.py \
  --output-dir run/trajectory \
  --estimate-topic /odometry/filtered \
  --estimate-type odometry \
  --duration-s 120
```

It returns a non-zero status for missing data or timestamp faults. Do not run
the ground-truth subscription on the physical vehicle; record the estimate and
external reference system separately there.

The recorder also rejects frame-contract violations. Its defaults require the
estimate to be `odom -> base_link` and MuJoCo truth to use the simulator's
`world` frame. Override `--estimate-frame-id`, `--estimate-child-frame-id`, or
`--ground-truth-frame-id` only when the upstream system intentionally uses a
different fixed frame. The two global frames may have a fixed origin/yaw
offset, which the reported SE(3) alignment removes; a changing or mislabeled
frame is rejected instead of being silently compared as raw numbers.

## Synchronization Contract

- ROS headers share one reference clock within a run.
- A50 `time_of_validity` remains the device measurement time; its ROS header is
  host receipt time. Fit an affine device-to-host clock relation before using
  the embedded timestamp for fusion.
- Never replace a capture timestamp with arrival time merely to make streams
  appear synchronized.
- Reject or separately report non-monotonic timestamps, large interpolation
  gaps, stale `/clock`, and low match coverage.

Export the SLAM estimate and evaluation-only truth in TUM form:

```text
timestamp_s tx ty tz qx qy qz qw
```

Then evaluate them without feeding truth back into the estimator:

```bash
python3 tools/evaluate_slam_trajectory.py \
  --estimate run/trajectory/estimate.tum \
  --ground-truth run/trajectory/ground_truth.tum \
  --alignment se3 \
  --estimate-time-offset-s 0.0 \
  --max-interpolation-gap-s 0.05 \
  --rpe-delta-s 1.0 \
  --rpe-tolerance-s 0.05 \
  --json-out run/trajectory/metrics.json \
  --markdown-out run/trajectory/metrics.md
```

Use `se3` for metric DVL/IMU/depth SLAM. Use `sim3` only when evaluating an
intentionally scale-ambiguous estimator, such as unscaled monocular vision;
report the recovered scale explicitly.

## Experiment Matrix

At minimum, repeat each trajectory across:

- nominal parameters and multiple deterministic seeds;
- sensor bias/latency/dropout profile variants;
- clear, turbid, bright, and low-light camera conditions;
- still water and spatially varying currents;
- bottom-lock transitions and partial visual occlusion;
- MuJoCo, the second simulator, and the physical pool.

Compare timestamp health and match coverage before comparing ATE/RPE. Report
translation and rotation RMSE, median, P95, maximum, RPE pair count, alignment
mode/scale, seed, and every rejected run. Thresholds must be selected from the
mission accuracy requirement and physical repeatability; they must not be tuned
on the final evaluation trajectory.
