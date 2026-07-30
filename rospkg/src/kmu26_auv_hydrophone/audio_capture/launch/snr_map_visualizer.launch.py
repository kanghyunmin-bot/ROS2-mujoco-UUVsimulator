from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_cell_size_m = LaunchConfiguration("map_cell_size_m")
    map_radius_m = LaunchConfiguration("map_radius_m")
    display_rotation_rad = LaunchConfiguration("display_rotation_rad")
    plot_rate_hz = LaunchConfiguration("plot_rate_hz")
    snr_topic = LaunchConfiguration("snr_topic")
    odometry_topic = LaunchConfiguration("odometry_topic")
    reset_topic = LaunchConfiguration("reset_topic")
    direction_topic = LaunchConfiguration("direction_topic")
    confidence_topic = LaunchConfiguration("confidence_topic")
    estimator_ready_topic = LaunchConfiguration("estimator_ready_topic")
    state_topic = LaunchConfiguration("state_topic")

    arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("map_cell_size_m", default_value="0.15"),
        DeclareLaunchArgument("map_radius_m", default_value="2.0"),
        DeclareLaunchArgument("display_rotation_rad", default_value="0.0"),
        DeclareLaunchArgument("plot_rate_hz", default_value="5.0"),
        DeclareLaunchArgument(
            "snr_topic", default_value="/audio_frequency_detector/snr_db_stamped"
        ),
        DeclareLaunchArgument("odometry_topic", default_value="/odometry/filtered"),
        DeclareLaunchArgument("reset_topic", default_value="/homing/reset_estimator"),
        DeclareLaunchArgument("direction_topic", default_value="/homing/direction"),
        DeclareLaunchArgument(
            "confidence_topic", default_value="/homing/snr_confidence"
        ),
        DeclareLaunchArgument(
            "estimator_ready_topic", default_value="/homing/estimator_ready"
        ),
        DeclareLaunchArgument("state_topic", default_value="/homing/control_state"),
    ]

    visualizer = Node(
        package="audio_capture",
        executable="snr_map_visualizer.py",
        name="snr_map_visualizer",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "map_cell_size_m": ParameterValue(
                    map_cell_size_m, value_type=float
                ),
                "map_radius_m": ParameterValue(map_radius_m, value_type=float),
                "display_rotation_rad": ParameterValue(
                    display_rotation_rad, value_type=float
                ),
                "plot_rate_hz": ParameterValue(plot_rate_hz, value_type=float),
                "snr_topic": snr_topic,
                "odometry_topic": odometry_topic,
                "reset_topic": reset_topic,
                "direction_topic": direction_topic,
                "confidence_topic": confidence_topic,
                "estimator_ready_topic": estimator_ready_topic,
                "state_topic": state_topic,
            }
        ],
    )

    return LaunchDescription(arguments + [visualizer])
