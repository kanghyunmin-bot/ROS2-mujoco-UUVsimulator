from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model_path_default = PathJoinSubstitution(
        [FindPackageShare('hit25_interpreter'), 'models', '1122best_openvino_model', '1122best.xml']
    )

    return LaunchDescription([
        DeclareLaunchArgument('device_name', default_value='AMS-22'),
        DeclareLaunchArgument('sample_rate', default_value='96000'),
        DeclareLaunchArgument('block_size', default_value='4096'),
        DeclareLaunchArgument('channels', default_value='1'),
        DeclareLaunchArgument('freq_range_min', default_value='10000'),
        DeclareLaunchArgument('freq_range_max', default_value='30000'),
        DeclareLaunchArgument('target_frequency', default_value='12000.0'),
        DeclareLaunchArgument('freq_tolerance', default_value='150.0'),
        DeclareLaunchArgument('db_threshold', default_value='-40.0'),
        DeclareLaunchArgument('db_for_max_confidence', default_value='-20.0'),
        DeclareLaunchArgument('stop_pulseaudio_on_start', default_value='true'),

        DeclareLaunchArgument('model_path', default_value=model_path_default),
        DeclareLaunchArgument('conf_threshold', default_value='0.5'),
        DeclareLaunchArgument('iou_threshold', default_value='0.45'),
        DeclareLaunchArgument('lpf_alpha', default_value='0.5'),

        Node(
            package='hit25_interpreter',
            executable='hydrophone_analyzer_node.py',
            name='hydrophone_analyzer_node',
            output='screen',
            parameters=[{
                'device_name': LaunchConfiguration('device_name'),
                'sample_rate': LaunchConfiguration('sample_rate'),
                'block_size': LaunchConfiguration('block_size'),
                'channels': LaunchConfiguration('channels'),
                'freq_range_min': LaunchConfiguration('freq_range_min'),
                'freq_range_max': LaunchConfiguration('freq_range_max'),
                'target_frequency': LaunchConfiguration('target_frequency'),
                'freq_tolerance': LaunchConfiguration('freq_tolerance'),
                'db_threshold': LaunchConfiguration('db_threshold'),
                'db_for_max_confidence': LaunchConfiguration('db_for_max_confidence'),
                'stop_pulseaudio_on_start': LaunchConfiguration('stop_pulseaudio_on_start'),
            }],
        ),

        Node(
            package='hit25_interpreter',
            executable='vision_interpreter_node.py',
            name='vision_interpreter_node',
            output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'conf_threshold': LaunchConfiguration('conf_threshold'),
                'iou_threshold': LaunchConfiguration('iou_threshold'),
                'lpf_alpha': LaunchConfiguration('lpf_alpha'),
            }],
        ),
    ])
