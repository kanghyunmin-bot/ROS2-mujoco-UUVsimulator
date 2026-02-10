import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    use_image_view = LaunchConfiguration('use_image_view')
    rviz_config = LaunchConfiguration('rviz_config')
    image_remap = LaunchConfiguration('image_remap')

    pkg_share = get_package_share_directory('hit25_auv')
    default_rviz = os.path.join(pkg_share, 'rviz', 'control_pc_ros2.rviz')

    actions = [
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_image_view', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),
        DeclareLaunchArgument('image_remap', default_value='image:=/camera/color/image_raw/compressed'),
        Node(
            package='hit25_auv',
            executable='auv_state_gui',
            name='auv_state_gui',
            output='screen',
        ),
    ]

    actions.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(use_rviz),
        )
    )

    try:
        get_package_share_directory('rqt_image_view')
        actions.append(
            Node(
                package='rqt_image_view',
                executable='rqt_image_view',
                name='rqt_image_view',
                output='screen',
                arguments=['--ros-args', '-r', image_remap],
                condition=IfCondition(use_image_view),
            )
        )
    except PackageNotFoundError:
        actions.append(LogInfo(msg='[control_pc_gui] rqt_image_view package not found; skipping image view.'))

    return LaunchDescription(actions)
