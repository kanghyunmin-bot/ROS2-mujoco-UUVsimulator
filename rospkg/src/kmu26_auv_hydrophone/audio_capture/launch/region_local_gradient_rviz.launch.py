from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    marker_topic = LaunchConfiguration("marker_topic")
    rviz_config = os.path.join(
        get_package_share_directory("audio_capture"),
        "rviz",
        "region_local_gradient.rviz",
    )

    arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("odometry_topic", default_value="/odometry/filtered"),
        DeclareLaunchArgument(
            "snr_topic", default_value="/audio_frequency_detector/snr_db_stamped"
        ),
        DeclareLaunchArgument(
            "region_gradient_topic", default_value="/homing/region_gradient"
        ),
        DeclareLaunchArgument(
            "rolling_gradient_topic", default_value="/homing/rolling_gradient"
        ),
        DeclareLaunchArgument(
            "homing_direction_topic", default_value="/homing/homing_direction"
        ),
        DeclareLaunchArgument(
            "waypoint_topic", default_value="/waypoint"
        ),
        DeclareLaunchArgument("scan_center_topic", default_value="/homing/scan_center"),
        DeclareLaunchArgument("marker_topic", default_value="/homing/rviz/markers"),
        DeclareLaunchArgument("arena_length_m", default_value="15.0"),
        DeclareLaunchArgument("arena_width_m", default_value="16.0"),
        DeclareLaunchArgument("arena_offset_x_m", default_value="0.0"),
        DeclareLaunchArgument("arena_offset_y_m", default_value="0.0"),
        DeclareLaunchArgument("arena_safety_margin_m", default_value="0.5"),
        DeclareLaunchArgument("vision_near_zone_width_m", default_value="2.0"),
        DeclareLaunchArgument("arena_start_corner", default_value="bottom_left"),
        DeclareLaunchArgument("map_cell_size_m", default_value="0.15"),
        DeclareLaunchArgument("arrow_length_m", default_value="1.0"),
        DeclareLaunchArgument("publish_rate_hz", default_value="5.0"),
        DeclareLaunchArgument("dynamic_snr_range", default_value="true"),
        DeclareLaunchArgument("snr_color_min_db", default_value="-10.0"),
        DeclareLaunchArgument("snr_color_max_db", default_value="20.0"),
    ]

    visualizer = Node(
        package="audio_capture",
        executable="region_local_gradient_rviz_visualizer",
        name="region_local_gradient_rviz_visualizer",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "odometry_topic": LaunchConfiguration("odometry_topic"),
                "snr_topic": LaunchConfiguration("snr_topic"),
                "region_gradient_topic": LaunchConfiguration(
                    "region_gradient_topic"
                ),
                "rolling_gradient_topic": LaunchConfiguration(
                    "rolling_gradient_topic"
                ),
                "homing_direction_topic": LaunchConfiguration(
                    "homing_direction_topic"
                ),
                "waypoint_topic": LaunchConfiguration("waypoint_topic"),
                "scan_center_topic": LaunchConfiguration("scan_center_topic"),
                "marker_topic": marker_topic,
                "arena_length_m": ParameterValue(
                    LaunchConfiguration("arena_length_m"), value_type=float
                ),
                "arena_width_m": ParameterValue(
                    LaunchConfiguration("arena_width_m"), value_type=float
                ),
                "arena_offset_x_m": ParameterValue(
                    LaunchConfiguration("arena_offset_x_m"), value_type=float
                ),
                "arena_offset_y_m": ParameterValue(
                    LaunchConfiguration("arena_offset_y_m"), value_type=float
                ),
                "arena_safety_margin_m": ParameterValue(
                    LaunchConfiguration("arena_safety_margin_m"), value_type=float
                ),
                "vision_near_zone_width_m": ParameterValue(
                    LaunchConfiguration("vision_near_zone_width_m"), value_type=float
                ),
                "arena_start_corner": LaunchConfiguration("arena_start_corner"),
                "map_cell_size_m": ParameterValue(
                    LaunchConfiguration("map_cell_size_m"), value_type=float
                ),
                "arrow_length_m": ParameterValue(
                    LaunchConfiguration("arrow_length_m"), value_type=float
                ),
                "publish_rate_hz": ParameterValue(
                    LaunchConfiguration("publish_rate_hz"), value_type=float
                ),
                "dynamic_snr_range": ParameterValue(
                    LaunchConfiguration("dynamic_snr_range"), value_type=bool
                ),
                "snr_color_min_db": ParameterValue(
                    LaunchConfiguration("snr_color_min_db"), value_type=float
                ),
                "snr_color_max_db": ParameterValue(
                    LaunchConfiguration("snr_color_max_db"), value_type=float
                ),
            }
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="region_local_gradient_rviz",
        arguments=["-d", rviz_config],
        remappings=[("/homing/rviz/markers", marker_topic)],
        parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
        output="screen",
    )
    return LaunchDescription(arguments + [visualizer, rviz])
