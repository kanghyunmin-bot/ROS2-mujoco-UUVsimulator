# Copyright 2026 AUV Team
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _camera_node(index: int) -> Node:
    prefix = f"camera{index}"
    return Node(
        package="auv_imx219_camera",
        executable="imx219_camera_node",
        name="camera",
        namespace=LaunchConfiguration(f"{prefix}_namespace"),
        output="screen",
        parameters=[
            LaunchConfiguration(f"{prefix}_calibration_file"),
            {
                "sensor_id": ParameterValue(
                    LaunchConfiguration(f"{prefix}_sensor_id"), value_type=int
                ),
                "width": ParameterValue(LaunchConfiguration("width"), value_type=int),
                "height": ParameterValue(LaunchConfiguration("height"), value_type=int),
                "framerate": ParameterValue(
                    LaunchConfiguration("framerate"), value_type=int
                ),
                "flip_method": ParameterValue(
                    LaunchConfiguration(f"{prefix}_flip_method"), value_type=int
                ),
                "frame_id": LaunchConfiguration(f"{prefix}_frame_id"),
                "image_topic": "image_raw",
                "timestamp_source": LaunchConfiguration("timestamp_source"),
                "max_capture_age_ms": ParameterValue(
                    LaunchConfiguration("max_capture_age_ms"), value_type=int
                ),
            }
        ],
    )


def generate_launch_description() -> LaunchDescription:
    default_calibration = PathJoinSubstitution(
        [FindPackageShare("auv_imx219_camera"), "config", "calibration.example.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("width", default_value="1280"),
            DeclareLaunchArgument("height", default_value="720"),
            DeclareLaunchArgument("framerate", default_value="30"),
            DeclareLaunchArgument("timestamp_source", default_value="gstreamer_pts"),
            DeclareLaunchArgument("max_capture_age_ms", default_value="2000"),
            DeclareLaunchArgument(
                "camera0_namespace", default_value="imx219/camera0"
            ),
            DeclareLaunchArgument("camera0_sensor_id", default_value="0"),
            DeclareLaunchArgument("camera0_flip_method", default_value="0"),
            DeclareLaunchArgument(
                "camera0_calibration_file", default_value=default_calibration
            ),
            DeclareLaunchArgument(
                "camera0_frame_id", default_value="imx219_camera0_optical_frame"
            ),
            DeclareLaunchArgument(
                "camera1_namespace", default_value="imx219/camera1"
            ),
            DeclareLaunchArgument("camera1_sensor_id", default_value="1"),
            DeclareLaunchArgument("camera1_flip_method", default_value="0"),
            DeclareLaunchArgument(
                "camera1_calibration_file", default_value=default_calibration
            ),
            DeclareLaunchArgument(
                "camera1_frame_id", default_value="imx219_camera1_optical_frame"
            ),
            _camera_node(0),
            _camera_node(1),
        ]
    )
