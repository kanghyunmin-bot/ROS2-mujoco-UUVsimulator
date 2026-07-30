import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('ip_address', default_value='192.168.194.95'),
        launch.actions.DeclareLaunchArgument('velocity_frame_id', default_value='dvl_link'),
        launch.actions.DeclareLaunchArgument('position_frame_id', default_value='dvl_link'),
        launch.actions.DeclareLaunchArgument('configure_acoustic_on_startup', default_value='false'),
        launch.actions.DeclareLaunchArgument('startup_acoustic_enabled', default_value='true'),
        launch.actions.DeclareLaunchArgument('request_config_on_startup', default_value='true'),
        launch_ros.actions.Node(
            package='auv_dvl_a50',
            executable='auv_dvl_a50_sensor',
            parameters=[{
                'dvl_ip_address': launch.substitutions.LaunchConfiguration('ip_address'),
                'velocity_frame_id': launch.substitutions.LaunchConfiguration('velocity_frame_id'),
                'position_frame_id': launch.substitutions.LaunchConfiguration('position_frame_id'),
                'configure_acoustic_on_startup': ParameterValue(
                    launch.substitutions.LaunchConfiguration('configure_acoustic_on_startup'),
                    value_type=bool),
                'startup_acoustic_enabled': ParameterValue(
                    launch.substitutions.LaunchConfiguration('startup_acoustic_enabled'),
                    value_type=bool),
                'request_config_on_startup': ParameterValue(
                    launch.substitutions.LaunchConfiguration('request_config_on_startup'),
                    value_type=bool),
            }],
            output='screen'),
    ])
