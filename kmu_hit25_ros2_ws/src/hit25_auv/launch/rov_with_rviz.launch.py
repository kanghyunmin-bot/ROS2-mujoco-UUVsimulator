import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _maybe_include_rovio(context, tag: str):
    """Include external ROVIO launch file only when enabled and path is valid."""
    use_rovio = LaunchConfiguration('use_rovio').perform(context).strip().lower()
    if use_rovio not in ('1', 'true', 'yes', 'on'):
        return []

    rovio_path = LaunchConfiguration('rovio_launch').perform(context).strip()
    if not rovio_path:
        return [LogInfo(msg=f'[{tag}] use_rovio=true but rovio_launch is empty; skipping ROVIO launch.')]
    if not os.path.exists(rovio_path):
        return [LogInfo(msg=f'[{tag}] rovio_launch not found: {rovio_path}; skipping ROVIO launch.')]

    return [IncludeLaunchDescription(AnyLaunchDescriptionSource(rovio_path))]


def generate_launch_description():
    use_mavros = LaunchConfiguration('use_mavros')
    use_dvl = LaunchConfiguration('use_dvl')
    use_rovio = LaunchConfiguration('use_rovio')
    use_dronecan_battery = LaunchConfiguration('use_dronecan_battery')

    fcu_url = LaunchConfiguration('fcu_url')
    dvl_ip = LaunchConfiguration('dvl_ip')
    rovio_launch = LaunchConfiguration('rovio_launch')

    actions = [
        DeclareLaunchArgument('use_mavros', default_value='true'),
        DeclareLaunchArgument('use_dvl', default_value='false'),
        DeclareLaunchArgument('use_rovio', default_value='false'),
        DeclareLaunchArgument('use_dronecan_battery', default_value='false'),
        DeclareLaunchArgument('fcu_url', default_value='/dev/ttyACM0:57600'),
        DeclareLaunchArgument('dvl_ip', default_value='192.168.194.95'),
        DeclareLaunchArgument('rovio_launch', default_value=''),
    ]

    try:
        mavros_share = get_package_share_directory('mavros')
        mavros_launch = os.path.join(mavros_share, 'launch', 'apm.launch')
        actions.append(
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(mavros_launch),
                launch_arguments={'fcu_url': fcu_url}.items(),
                condition=IfCondition(use_mavros),
            )
        )
    except PackageNotFoundError:
        actions.append(LogInfo(msg='[rov_with_rviz] mavros package not found; skipping MAVROS launch.'))

    try:
        dvl_share = get_package_share_directory('waterlinked_a50_ros_driver')
        dvl_launch_candidates = (
            os.path.join(dvl_share, 'launch', 'dvl_start.launch'),
            os.path.join(dvl_share, 'launch', 'launch_dvl.launch'),
        )
        dvl_launch = next((p for p in dvl_launch_candidates if os.path.exists(p)), '')
        if dvl_launch:
            actions.append(
                IncludeLaunchDescription(
                    AnyLaunchDescriptionSource(dvl_launch),
                    launch_arguments={'ip': dvl_ip}.items(),
                    condition=IfCondition(use_dvl),
                )
            )
        else:
            actions.append(LogInfo(msg='[rov_with_rviz] DVL launch file not found (dvl_start/launch_dvl); skipping.'))
    except PackageNotFoundError:
        actions.append(LogInfo(msg='[rov_with_rviz] waterlinked_a50_ros_driver not found; skipping DVL.'))

    actions.append(
        OpaqueFunction(function=lambda context: _maybe_include_rovio(context, 'rov_with_rviz'))
    )

    rviz_config = os.path.join(get_package_share_directory('hit25_auv'), 'rviz', 'rov.rviz')

    actions.extend([
        Node(package='hit25_auv', executable='joy2mavros', name='joy2mavros', output='screen'),
        Node(package='hit25_auv', executable='vfr2atm_pressure', name='vfr2atm_pressure', output='screen'),
        Node(
            package='hit25_auv',
            executable='dronecan2mavros_battery',
            name='dronecan2mavros_battery',
            output='screen',
            condition=IfCondition(use_dronecan_battery),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': open(os.path.join(get_package_share_directory('hit25_auv'), 'urdf', 'rov.urdf')).read()
            }],
        ),
        Node(
            package='hit25_auv',
            executable='rov_tf_broadcaster',
            name='rov_tf_broadcaster',
            output='screen',
            parameters=[{
                'map_frame': 'map',
                'odom_frame': 'odom',
                'publish_rate': 20.0,
                'base_frame': 'auv_link',
                'dvl_x': 0.0,
                'dvl_y': 0.0,
                'dvl_z': 0.0,
                'dvl_roll': 3.14159265,
                'dvl_pitch': 0.0,
                'dvl_yaw': 0.0,
                'dvl_use_odom': True,
                'dvl_odom_topic': '/dvl/odometry',
                'dvl_ned_to_flu': True,
            }],
        ),
        Node(
            package='hit25_auv',
            executable='ref_point_marker_node',
            name='ref_point_marker',
            output='screen',
            parameters=[{
                'parent_frame': 'odom',
                'auv_frame': 'auv_link',
                'max_path_points': 50000,
                'marker_topic': '/ref_point_marker',
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])

    return LaunchDescription(actions)
