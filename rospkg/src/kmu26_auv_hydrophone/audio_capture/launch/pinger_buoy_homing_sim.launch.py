from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    signal_mode = LaunchConfiguration("signal_mode")
    noise_bag = LaunchConfiguration("noise_bag")
    frequency_hz = LaunchConfiguration("frequency_hz")
    pinger_x = LaunchConfiguration("pinger_x")
    pinger_y = LaunchConfiguration("pinger_y")
    pinger_z = LaunchConfiguration("pinger_z")
    source_amplitude = LaunchConfiguration("source_amplitude")
    clean_noise_amplitude = LaunchConfiguration("clean_noise_amplitude")
    minimum_distance_m = LaunchConfiguration("minimum_distance_m")
    attenuation_power = LaunchConfiguration("attenuation_power")
    sound_speed_mps = LaunchConfiguration("sound_speed_mps")
    odometry_topic = LaunchConfiguration("odometry_topic")
    arena_start_corner = LaunchConfiguration("arena_start_corner")
    arena_yaw_rad = LaunchConfiguration("arena_yaw_rad")
    search_yaw = LaunchConfiguration("search_yaw")
    search_forward = LaunchConfiguration("search_forward")
    initial_diagonal_command = LaunchConfiguration("initial_diagonal_command")
    forward_fast = LaunchConfiguration("forward_fast")
    forward_mid = LaunchConfiguration("forward_mid")
    forward_slow = LaunchConfiguration("forward_slow")
    confidence_speed_floor = LaunchConfiguration("confidence_speed_floor")
    initial_diagonal_duration_s = LaunchConfiguration("initial_diagonal_duration_s")
    stuck_progress_threshold_mps = LaunchConfiguration("stuck_progress_threshold_mps")
    stuck_yaw_rate_threshold_rps = LaunchConfiguration(
        "stuck_yaw_rate_threshold_rps"
    )
    collision_hold_s = LaunchConfiguration("collision_hold_s")
    wall_reverse_command = LaunchConfiguration("wall_reverse_command")
    wall_reverse_duration_s = LaunchConfiguration("wall_reverse_duration_s")
    wall_escape_forward_command = LaunchConfiguration("wall_escape_forward_command")
    wall_escape_duration_s = LaunchConfiguration("wall_escape_duration_s")
    collision_rearm_timeout_s = LaunchConfiguration("collision_rearm_timeout_s")
    wall_homing_resume_grace_s = LaunchConfiguration("wall_homing_resume_grace_s")

    arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "signal_mode",
            default_value="noisy",
            description=(
                "핑거 신호 구성: noisy(실측 배경 포함) 또는 "
                "clean(낮고 일정한 기준 잡음 + 사인파)."
            ),
        ),
        DeclareLaunchArgument(
            "noise_bag",
            default_value="/home/kim/new_hydrophone_ws/localization_20260719_185918",
            description="실측 배경 노이즈를 읽을 ROS 2 bag 디렉터리.",
        ),
        DeclareLaunchArgument("frequency_hz", default_value="21164.0"),
        DeclareLaunchArgument("pinger_x", default_value="-2.0"),
        DeclareLaunchArgument("pinger_y", default_value="0.65"),
        DeclareLaunchArgument("pinger_z", default_value="-0.5"),
        DeclareLaunchArgument("source_amplitude", default_value="0.03"),
        DeclareLaunchArgument(
            "clean_noise_amplitude",
            default_value="0.001",
            description="clean 모드의 거리 독립 기준 잡음 진폭.",
        ),
        DeclareLaunchArgument("minimum_distance_m", default_value="0.5"),
        DeclareLaunchArgument("attenuation_power", default_value="2.0"),
        DeclareLaunchArgument("sound_speed_mps", default_value="1500.0"),
        DeclareLaunchArgument("odometry_topic", default_value="/odometry/filtered"),
        DeclareLaunchArgument(
            "search_yaw",
            default_value="0.30",
            description="초기 곡선 탐색 yaw 명령. 작을수록 탐색 반경이 커진다.",
        ),
        DeclareLaunchArgument("search_forward", default_value="0.30"),
        DeclareLaunchArgument("initial_diagonal_command", default_value="0.30"),
        DeclareLaunchArgument("forward_fast", default_value="0.70"),
        DeclareLaunchArgument("forward_mid", default_value="0.45"),
        DeclareLaunchArgument("forward_slow", default_value="0.20"),
        DeclareLaunchArgument("confidence_speed_floor", default_value="0.40"),
        DeclareLaunchArgument(
            "initial_diagonal_duration_s",
            default_value="6.0",
            description="초기 대각선 open-loop 이동 시간(초).",
        ),
        DeclareLaunchArgument(
            "stuck_progress_threshold_mps",
            default_value="0.03",
            description="충돌로 판단할 명령 방향 투영 선속도 상한(m/s).",
        ),
        DeclareLaunchArgument(
            "stuck_yaw_rate_threshold_rps",
            default_value="0.05",
            description="회전 명령에 정상 반응했다고 볼 yaw rate 하한(rad/s).",
        ),
        DeclareLaunchArgument(
            "collision_hold_s",
            default_value="3.0",
            description="충돌 상태가 연속으로 유지되어야 하는 시간(초).",
        ),
        DeclareLaunchArgument(
            "wall_reverse_command",
            default_value="0.25",
            description="벽 회복 후진 명령 크기(0~1).",
        ),
        DeclareLaunchArgument(
            "wall_reverse_duration_s",
            default_value="2.0",
            description="벽 회복 후진 시간(초).",
        ),
        DeclareLaunchArgument("wall_escape_forward_command", default_value="0.25"),
        DeclareLaunchArgument("wall_escape_duration_s", default_value="1.5"),
        DeclareLaunchArgument("collision_rearm_timeout_s", default_value="1.5"),
        DeclareLaunchArgument("wall_homing_resume_grace_s", default_value="2.0"),
        # bottom_right 시작 → 안쪽은 왼쪽 위.
        # 관측: -π/2 → 오른쪽 위, 0 → 오른쪽 아래 이므로 π 로 맞춤.
        DeclareLaunchArgument("arena_start_corner", default_value="bottom_right"),
        DeclareLaunchArgument(
            "arena_yaw_rad",
            default_value="3.141592653589793",
        ),
    ]

    pinger_audio = Node(
        package="audio_capture",
        executable="pinger_buoy_audio_sim.py",
        name="pinger_buoy_audio_sim",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "signal_mode": signal_mode,
                "noise_bag": noise_bag,
                "frequency_hz": ParameterValue(frequency_hz, value_type=float),
                "pinger_x": ParameterValue(pinger_x, value_type=float),
                "pinger_y": ParameterValue(pinger_y, value_type=float),
                "pinger_z": ParameterValue(pinger_z, value_type=float),
                "source_amplitude": ParameterValue(
                    source_amplitude, value_type=float
                ),
                "clean_noise_amplitude": ParameterValue(
                    clean_noise_amplitude, value_type=float
                ),
                "minimum_distance_m": ParameterValue(
                    minimum_distance_m, value_type=float
                ),
                "attenuation_power": ParameterValue(
                    attenuation_power, value_type=float
                ),
                "sound_speed_mps": ParameterValue(
                    sound_speed_mps, value_type=float
                ),
                "odometry_topic": odometry_topic,
            }
        ],
    )

    homing_launch_path = (
        get_package_share_directory("audio_capture")
        + "/launch/snr_gradient_homing_control.launch.py"
    )
    homing = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(homing_launch_path),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_stamped_audio": "true",
            "audio_topic": "/audio",
            "audio_stamped_topic": "/audio_stamped",
            "odometry_topic": odometry_topic,
            "reference_frequency_hz": frequency_hz,
            "search_forward": search_forward,
            "search_yaw": search_yaw,
            "initial_diagonal_command": initial_diagonal_command,
            "initial_diagonal_duration_s": initial_diagonal_duration_s,
            "forward_fast": forward_fast,
            "forward_mid": forward_mid,
            "forward_slow": forward_slow,
            "confidence_speed_floor": confidence_speed_floor,
            "stuck_progress_threshold_mps": stuck_progress_threshold_mps,
            "stuck_yaw_rate_threshold_rps": stuck_yaw_rate_threshold_rps,
            "collision_hold_s": collision_hold_s,
            "wall_reverse_command": wall_reverse_command,
            "wall_reverse_duration_s": wall_reverse_duration_s,
            "wall_escape_forward_command": wall_escape_forward_command,
            "wall_escape_duration_s": wall_escape_duration_s,
            "collision_rearm_timeout_s": collision_rearm_timeout_s,
            "wall_homing_resume_grace_s": wall_homing_resume_grace_s,
            "arena_start_corner": arena_start_corner,
            "arena_yaw_rad": arena_yaw_rad,
        }.items(),
    )

    return LaunchDescription(arguments + [pinger_audio, homing])
