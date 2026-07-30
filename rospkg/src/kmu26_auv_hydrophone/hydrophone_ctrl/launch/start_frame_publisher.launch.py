from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "odometry_topic", default_value="/odometry/filtered"
            ),
            DeclareLaunchArgument(
                "start_frame_topic", default_value="/start_frame"
            ),
            DeclareLaunchArgument("frame_id", default_value="odom"),
            DeclareLaunchArgument("publish_rate_hz", default_value="1.0"),
            Node(
                package="hydrophone_ctrl",
                executable="start_frame_publisher",
                name="start_frame_publisher",
                output="screen",
                parameters=[
                    {
                        "odometry_topic": LaunchConfiguration("odometry_topic"),
                        "start_frame_topic": LaunchConfiguration(
                            "start_frame_topic"
                        ),
                        "frame_id": LaunchConfiguration("frame_id"),
                        "publish_rate_hz": ParameterValue(
                            LaunchConfiguration("publish_rate_hz"),
                            value_type=float,
                        ),
                    }
                ],
            ),
        ]
    )
