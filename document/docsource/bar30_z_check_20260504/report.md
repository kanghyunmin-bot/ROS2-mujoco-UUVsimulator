# 2026-04-01 Real Robot ROS Bag Analysis

## Bag inventory

| bag | duration s | messages | topics | IMU Hz | DVL Hz | depth Hz | odom filtered Hz | local odom Hz |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| bag_2026-04-01_20-08-11 | 471.56 | 286685 | 99 | 20 | 10.05 | 2 | 29.9 | 3.004 |

## Interface observations

- `bag_2026-04-01_20-08-11` frames: IMU `{'fcu_link': 9426}`, DVL `{'dvl': 4219}`, depth `{'odom': 943}`.
- `bag_2026-04-01_20-08-11` MAVROS modes: `{'ALT_HOLD': 386, 'MANUAL': 65, 'STABILIZE': 44}`.
- `bag_2026-04-01_20-08-11` top TF edges: `[('odom->base_link', 11129)]`.

## Sensor statistics

| bag | depth range m | DVL speed p95 m/s | DVL max gap s | IMU gyro HP std xyz rad/s | IMU accel HP std xyz m/s^2 |
|---|---:|---:|---:|---|---|
| bag_2026-04-01_20-08-11 | 0.7933 | 0.4861 | 2.379 | [0.02351, 0.0381, 0.1397] | [0.2263, 0.2017, 0.08216] |

## Cross-sensor checks

| bag | static pressure slope Pa/m | pressure-depth corr | static pressure fit residual std Pa | depth vs -filtered z residual std m | local odom vs filtered xyz residual norm p95 m |
|---|---:|---:|---:|---:|---:|
| bag_2026-04-01_20-08-11 | 9806.8 | 1 | 0.1732 | 0.0472 | 2.354 |

## Covariance values observed

### bag_2026-04-01_20-08-11
- `/mavros/imu/data:orientation_cov` mean diag: `[1.0, 1.0, 1.0]`, unique count: `1`
- `/mavros/imu/data:gyro_cov` mean diag: `[1.2184700254281686e-07, 1.2184700254281686e-07, 1.2184700254281686e-07]`, unique count: `1`
- `/mavros/imu/data:accel_cov` mean diag: `[9.000000000000614e-08, 9.000000000000614e-08, 9.000000000000614e-08]`, unique count: `1`
- `/dvl/twist:linear_cov` mean diag: `[8.49518670811179e-06, 2.461549917992729e-06, 4.197449937026559e-07]`, unique count: `4219`
- `/depth/pose:position_cov` mean diag: `[0.0, 0.0, 0.04999999999999943]`, unique count: `1`
- `/odometry/filtered:pose_position_cov` mean diag: `[110067.55037473992, 237379.4693454163, 0.04484462214329505]`, unique count: `11128`
- `/odometry/filtered:twist_linear_cov` mean diag: `[6.453945193904481, 47.910405497760024, 0.025756494964314033]`, unique count: `11128`

## Control / command observations

- `bag_2026-04-01_20-08-11` RC override median: `[1500.0, 1500.0, 1500.0, 1495.0, 1501.0, 1500.0, 1500.0, 1500.0]`, p05: `[1500.0, 1500.0, 1435.0, 1321.0, 1488.0, 1476.0, 1500.0, 1500.0]`, p95: `[1500.0, 1500.0, 1535.0, 1676.0, 1800.0, 1540.0, 1500.0, 1500.0]`, active fraction: `0.7807`.
- `bag_2026-04-01_20-08-11` joy axes p95 abs: `[0.23184596002101898, 0.9999999403953552, 0.9030014276504517, 0.6685123443603516, 0.9999999403953552, 0.9999999403953552, 0.0, 0.0]`.

## Simulator implications

- The April 1 data confirms the bridge should publish `/mavros/imu/data` at about 20 Hz with `frame_id=fcu_link`, `/depth/pose` at 2 Hz with `frame_id=odom`, and `/dvl/twist` in `frame_id=dvl` when DVL bottom lock is available.
- Actual water pressure variation is on `/mavros/imu/static_pressure` at about 2 Hz. `/mavros/imu/atm_pressure` is present at about 10 Hz but is constant near `0.24` in these bags, so it should not be treated as the depth pressure stream.
- DVL is not guaranteed to be present: the 20:06 bag has zero `/dvl/twist` messages while `/dvl/position` exists. The simulator should support DVL dropout/bottom-lock failure, not only Gaussian noise.
- `/mavros/local_position/*` is much slower than raw sensors, around 2.6-2.8 Hz in the runs where it exists. For EKF/controller testing, raw sensor rates and estimator output rates should remain separate.
- The 20:20 bag contains RealSense depth/IR/color and camera IMU streams. For SLAM tests, the sim will eventually need camera topic compatibility or a camera bridge path, but the control-critical AUV dynamics still depend first on IMU, DVL, depth, RC/joy, and odometry.
- Covariance values are mostly fixed constants. The measured high-pass residuals should be treated as operational noise-plus-motion, so use them as upper bounds unless you isolate a stationary segment.

## Generated plots

- `document/docsource/bar30_z_check_20260504/bag_2026-04-01_20-08-11/bag_2026-04-01_20-08-11_depth_dvl_rc.png`
- `document/docsource/bar30_z_check_20260504/bag_2026-04-01_20-08-11/bag_2026-04-01_20-08-11_trajectory.png`
- `document/docsource/bar30_z_check_20260504/bag_2026-04-01_20-08-11/bag_2026-04-01_20-08-11_imu.png`
