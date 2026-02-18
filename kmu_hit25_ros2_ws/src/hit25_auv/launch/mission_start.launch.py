from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    run_buoy = LaunchConfiguration('run_buoy')
    run_gate = LaunchConfiguration('run_gate')
    run_line_qr = LaunchConfiguration('run_line_qr')
    run_hydrophone = LaunchConfiguration('run_hydrophone')
    run_return = LaunchConfiguration('run_return')

    return LaunchDescription([
        DeclareLaunchArgument('run_buoy', default_value='true'),
        DeclareLaunchArgument('run_gate', default_value='false'),
        DeclareLaunchArgument('run_line_qr', default_value='false'),
        DeclareLaunchArgument('run_hydrophone', default_value='false'),
        DeclareLaunchArgument('run_return', default_value='false'),
        Node(
            package='hit25_auv',
            executable='buoy_mission',
            name='buoy_mission_node',
            output='screen',
            condition=IfCondition(run_buoy),
        ),
        Node(
            package='hit25_auv',
            executable='gate_mission',
            name='gate_mission_node',
            output='screen',
            condition=IfCondition(run_gate),
        ),
        Node(
            package='hit25_auv',
            executable='line_qr_mission',
            name='line_qr_mission_node',
            output='screen',
            condition=IfCondition(run_line_qr),
        ),
        Node(
            package='hit25_auv',
            executable='hydrophone_mission',
            name='hydrophone_mission_node',
            output='screen',
            condition=IfCondition(run_hydrophone),
        ),
        Node(
            package='hit25_auv',
            executable='return_mission',
            name='return_mission_node',
            output='screen',
            condition=IfCondition(run_return),
        ),
    ])
