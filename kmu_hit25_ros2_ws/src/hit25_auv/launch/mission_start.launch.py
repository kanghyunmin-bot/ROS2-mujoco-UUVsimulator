from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='hit25_auv', executable='buoy_mission', name='buoy_mission_node', output='screen'),
        Node(package='hit25_auv', executable='gate_mission', name='gate_mission_node', output='screen'),
        Node(package='hit25_auv', executable='line_qr_mission', name='line_qr_mission_node', output='screen'),
        Node(package='hit25_auv', executable='hydrophone_mission', name='hydrophone_mission_node', output='screen'),
        Node(package='hit25_auv', executable='return_mission', name='return_mission_node', output='screen'),
    ])
