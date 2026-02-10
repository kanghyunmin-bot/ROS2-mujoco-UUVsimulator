# ROS2 Bridge Quick Start

This simulator can publish sensor topics to ROS2 and receive control via `/cmd_vel`.

## 1) Run Simulator + ROS2 Bridge

IMU + DVL only:

```bash
python uuv_mujoco/run_urdf_full.py --scene uuv_mujoco/urdf_full_scene.xml --ros2
```

IMU + DVL + stereo images:

```bash
python uuv_mujoco/run_urdf_full.py \
  --scene uuv_mujoco/urdf_full_scene.xml \
  --ros2 --ros2-images --ros2-image-width 640 --ros2-image-height 360 --ros2-image-hz 10
```

## 2) Published Topics

- `/imu/data` (`sensor_msgs/msg/Imu`)
- `/dvl/velocity` (`geometry_msgs/msg/TwistStamped`)
- `/dvl/altitude` (`sensor_msgs/msg/Range`)
- `/stereo/left/image_raw` (`sensor_msgs/msg/Image`) when `--ros2-images`
- `/stereo/right/image_raw` (`sensor_msgs/msg/Image`) when `--ros2-images`
- `/stereo/left/camera_info` (`sensor_msgs/msg/CameraInfo`) when `--ros2-images`
- `/stereo/right/camera_info` (`sensor_msgs/msg/CameraInfo`) when `--ros2-images`

## 3) Control Input

Subscribe input:

- `/cmd_vel` (`geometry_msgs/msg/Twist`)

Mapping:

- `linear.x` -> forward
- `linear.y` -> sway
- `linear.z` -> heave
- `angular.z` -> yaw

Expected command range is roughly `[-1, 1]` for each field. Internally it scales to the simulator control limit.

## 4) Quick Test Commands

List topics:

```bash
ros2 topic list
```

Echo IMU:

```bash
ros2 topic echo /imu/data
```

Send forward command:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.4, y: 0.0, z: 0.0}, angular: {z: 0.0}}" -r 20
```

Stop:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}" -1
```

