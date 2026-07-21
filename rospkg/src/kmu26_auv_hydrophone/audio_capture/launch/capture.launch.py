from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# [오디오 캡처 launch 구성] 캡처 노드 실행 인자와 raw/stamped 오디오 remap을 정의한다.
def generate_launch_description():
    _src = LaunchConfiguration('src')
    _dst = LaunchConfiguration('dst')
    _device = LaunchConfiguration('device')
    _format = LaunchConfiguration('format')
    _bitrate = LaunchConfiguration('bitrate')
    _channels = LaunchConfiguration('channels')
    _depth = LaunchConfiguration('depth')
    _sample_rate = LaunchConfiguration('sample_rate')
    _sample_format = LaunchConfiguration('sample_format')
    _ns = LaunchConfiguration('ns')
    _audio_topic = LaunchConfiguration('audio_topic')
    _audio_stamped_topic = LaunchConfiguration('audio_stamped_topic')

    _src_launch_arg = DeclareLaunchArgument(
        'src',
        default_value='alsasrc'
    )
    _dst_launch_arg = DeclareLaunchArgument(
        'dst',
        default_value='appsink'
    )
    _device_launch_arg = DeclareLaunchArgument(
        'device',
        default_value=''
    )
    _format_launch_arg = DeclareLaunchArgument(
        'format',
        default_value='wave'
    )
    _bitrate_launch_arg = DeclareLaunchArgument(
        'bitrate',
        default_value='192'
    )
    _channels_launch_arg = DeclareLaunchArgument(
        'channels',
        default_value='2'
    )
    _depth_launch_arg = DeclareLaunchArgument(
        'depth',
        default_value='32'
    )
    _sample_rate_launch_arg = DeclareLaunchArgument(
        'sample_rate',
        default_value='96000'
    )
    _sample_format_launch_arg = DeclareLaunchArgument(
        'sample_format',
        default_value='S32LE'
    )
    _ns_launch_arg = DeclareLaunchArgument(
        'ns',
        default_value='audio'
    )
    _audio_topic_launch_arg = DeclareLaunchArgument(
        'audio_topic',
        default_value='/audio'
    )
    _audio_stamped_topic_launch_arg = DeclareLaunchArgument(
        'audio_stamped_topic',
        default_value='/audio_stamped'
    )

    _audio_capture_node = Node(
        package='audio_capture',
        name='audio_capture',
        executable='audio_capture_node',
        namespace=_ns,
        remappings=[
            ('audio', _audio_topic),
            ('audio_stamped', _audio_stamped_topic),
        ],
        parameters=[{
            'src': _src,
            'dst': _dst,
            'device': _device,
            'format': _format,
            'bitrate': _bitrate,
            'channels': _channels,
            'depth': _depth,
            'sample_rate': _sample_rate,
            'sample_format': _sample_format,
        }],
    )

    return LaunchDescription([
        _src_launch_arg,
        _dst_launch_arg,
        _device_launch_arg,
        _format_launch_arg,
        _bitrate_launch_arg,
        _channels_launch_arg,
        _depth_launch_arg,
        _sample_rate_launch_arg,
        _sample_format_launch_arg,
        _ns_launch_arg,
        _audio_topic_launch_arg,
        _audio_stamped_topic_launch_arg,
        _audio_capture_node,
    ])
