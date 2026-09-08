# auv_imx219_camera

ROS 2 Humble driver for two IMX219 CSI cameras connected to an NVIDIA Jetson.
It captures through `nvarguscamerasrc` and publishes standard ROS image-transport topics.

## Build

```bash
cd /home/auv/catkin_ws
colcon build --packages-select auv_imx219_camera
source install/setup.bash
```

## Run both cameras

```bash
ros2 launch auv_imx219_camera dual_imx219.launch.py
```

Default topics:

- `/imx219/camera0/image_raw`
- `/imx219/camera0/image_raw/compressed`
- `/imx219/camera0/camera_info`
- `/imx219/camera1/image_raw`
- `/imx219/camera1/image_raw/compressed`
- `/imx219/camera1/camera_info`

The compressed topics are supplied by `compressed_image_transport` and start encoding
when a compressed subscriber connects.

Image and CameraInfo headers use the GStreamer buffer PTS mapped into the ROS
clock by default (`timestamp_source:=gstreamer_pts`). If a source provides no
valid PTS, the node falls back to ROS publish time and emits a throttled
warning. `timestamp_source:=ros_now` restores the original behavior;
`max_capture_age_ms` bounds accepted PTS age.

Examples:

```bash
ros2 topic info /imx219/camera0/image_raw
ros2 topic hz /imx219/camera1/image_raw
```

Change the shared capture mode or rotate a sensor using launch arguments:

```bash
ros2 launch auv_imx219_camera dual_imx219.launch.py \
  width:=1920 height:=1080 framerate:=30 camera1_flip_method:=2
```

`flip_method` follows `nvvidconv`: `0` none, `1` 90 degrees counterclockwise,
`2` 180 degrees, `3` 90 degrees clockwise, `4` horizontal flip, `5` upper-right
diagonal, `6` vertical flip, and `7` upper-left diagonal.

If the physical left/right cameras are reversed, swap `camera0_sensor_id` and
`camera1_sensor_id` in the launch command. Camera-info matrices default to
uncalibrated zeros. Copy `config/calibration.example.yaml` for each calibrated lens,
fill in the matrices, and load them independently:

```bash
ros2 launch auv_imx219_camera dual_imx219.launch.py \
  camera0_calibration_file:=/path/to/camera0.yaml \
  camera1_calibration_file:=/path/to/camera1.yaml
```
