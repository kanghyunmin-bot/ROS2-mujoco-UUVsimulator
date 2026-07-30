from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


# [폐루프 homing launch 구성] 오디오 분석·방향 추정·AUV 제어 세 노드와 안전 기본값을 정의한다.
def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    audio_topic = LaunchConfiguration("audio_topic")
    audio_stamped_topic = LaunchConfiguration("audio_stamped_topic")
    use_stamped_audio = LaunchConfiguration("use_stamped_audio")
    odometry_topic = LaunchConfiguration("odometry_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    reference_frequency_hz = LaunchConfiguration("reference_frequency_hz")
    audio_input_latency_s = LaunchConfiguration("audio_input_latency_s")
    arena_width_m = LaunchConfiguration("arena_width_m")
    arena_height_m = LaunchConfiguration("arena_height_m")
    arena_start_corner = LaunchConfiguration("arena_start_corner")
    arena_yaw_rad = LaunchConfiguration("arena_yaw_rad")
    arena_start_inset_m = LaunchConfiguration("arena_start_inset_m")
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
    initial_diagonal_command = LaunchConfiguration("initial_diagonal_command")
    initial_diagonal_duration_s = LaunchConfiguration("initial_diagonal_duration_s")
    vertical_search_distance_m = LaunchConfiguration("vertical_search_distance_m")
    vertical_search_min_z_m = LaunchConfiguration("vertical_search_min_z_m")
    vertical_search_max_z_m = LaunchConfiguration("vertical_search_max_z_m")
    search_forward = LaunchConfiguration("search_forward")
    search_yaw = LaunchConfiguration("search_yaw")
    search_turn_sign = LaunchConfiguration("search_turn_sign")
    forward_fast = LaunchConfiguration("forward_fast")
    forward_mid = LaunchConfiguration("forward_mid")
    forward_slow = LaunchConfiguration("forward_slow")
    confidence_speed_floor = LaunchConfiguration("confidence_speed_floor")
    invert_rc_yaw = LaunchConfiguration("invert_rc_yaw")
    invert_rc_lateral = LaunchConfiguration("invert_rc_lateral")
    rc_override_topic = LaunchConfiguration("rc_override_topic")
    rc_preview_topic = LaunchConfiguration("rc_preview_topic")

    arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("audio_topic", default_value="/audio"),
        DeclareLaunchArgument("audio_stamped_topic", default_value="/audio_stamped"),
        DeclareLaunchArgument(
            "use_stamped_audio",
            default_value="true",
            description="실기에서는 capture node의 원본 PTS가 든 stamped audio를 사용한다.",
        ),
        DeclareLaunchArgument("odometry_topic", default_value="/odometry/filtered"),
        DeclareLaunchArgument("depth_topic", default_value="/depth/pose"),
        DeclareLaunchArgument(
            "reference_frequency_hz",
            default_value="21164.0",
            description="예상 pinger 주파수(Hz).",
        ),
        DeclareLaunchArgument(
            "audio_input_latency_s",
            default_value="0.0",
            description="오디오 수신 시각에서 추가로 보정할 캡처/전송 지연(초).",
        ),
        DeclareLaunchArgument(
            "arena_width_m",
            default_value="2.0",
            description="수조/경기장의 가로 폭(m).",
        ),
        DeclareLaunchArgument(
            "arena_height_m",
            default_value="5.0",
            description="수조/경기장의 세로 길이(m).",
        ),
        DeclareLaunchArgument(
            "arena_start_corner",
            default_value="bottom_right",
            description="AUV 시작 모서리: bottom_left 또는 bottom_right.",
        ),
        DeclareLaunchArgument(
            "arena_yaw_rad",
            default_value="0.0",
            description=(
                "수조 가로축이 위치 지도(odom) 기준으로 얼마나 돌아가 있는지(rad). "
                "초기 대각선·벽 회복 접선 방향을 맞출 때 쓴다."
            ),
        ),
        DeclareLaunchArgument(
            "arena_start_inset_m",
            default_value="0.12",
            description="첫 odometry 위치를 경기장 모서리에서 안쪽으로 둘 거리(m).",
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
            description="WALL_RECOVERY 후진 명령 크기(0~1).",
        ),
        DeclareLaunchArgument(
            "wall_reverse_duration_s",
            default_value="2.0",
            description="WALL_RECOVERY 후진 유지 시간(초).",
        ),
        DeclareLaunchArgument(
            "wall_escape_forward_command",
            default_value="0.25",
            description="회피 방향으로 벽에서 이탈할 전진 명령 크기(0~1).",
        ),
        DeclareLaunchArgument(
            "wall_escape_duration_s",
            default_value="1.5",
            description="회피 방향 전진 유지 시간(초).",
        ),
        DeclareLaunchArgument(
            "collision_rearm_timeout_s",
            default_value="1.5",
            description="복귀 후 충돌 판정을 다시 활성화할 최대 대기 시간(초).",
        ),
        DeclareLaunchArgument(
            "wall_homing_resume_grace_s",
            default_value="2.0",
            description="벽 회피 후 충돌 당시 homing 방향을 유지할 시간(초).",
        ),
        DeclareLaunchArgument(
            "initial_diagonal_command",
            default_value="0.40",
            description="초기 대각선 구간의 정규화된 수평 이동 명령(0~1).",
        ),
        DeclareLaunchArgument(
            "initial_diagonal_duration_s",
            default_value="10.0",
            description="첫 odometry 수신 후 초기 대각선 open-loop 명령을 유지할 시간(초).",
        ),
        DeclareLaunchArgument(
            "vertical_search_distance_m",
            default_value="0.50",
            description="SNR trigger 후 현재 수심에서 위·아래로 탐색할 거리(m).",
        ),
        DeclareLaunchArgument(
            "vertical_search_min_z_m",
            default_value="-1.30",
            description="수직 탐색에서 허용할 가장 깊은 odometry z(m).",
        ),
        DeclareLaunchArgument(
            "vertical_search_max_z_m",
            default_value="-0.20",
            description="수직 탐색에서 허용할 가장 얕은 odometry z(m).",
        ),
        DeclareLaunchArgument("search_forward", default_value="0.30"),
        DeclareLaunchArgument("search_yaw", default_value="0.30"),
        DeclareLaunchArgument("search_turn_sign", default_value="1.0"),
        DeclareLaunchArgument("forward_fast", default_value="0.70"),
        DeclareLaunchArgument("forward_mid", default_value="0.45"),
        DeclareLaunchArgument("forward_slow", default_value="0.20"),
        DeclareLaunchArgument("confidence_speed_floor", default_value="0.40"),
        DeclareLaunchArgument("invert_rc_yaw", default_value="true"),
        DeclareLaunchArgument("invert_rc_lateral", default_value="false"),
        DeclareLaunchArgument("rc_override_topic", default_value="/mavros/rc/override"),
        DeclareLaunchArgument("rc_preview_topic", default_value="/homing/rc_override_preview"),
    ]

    container = ComposableNodeContainer(
        name="snr_gradient_homing_control_pipeline",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="audio_capture",
                plugin="audio_capture::AudioFrequencyDetectorNode",
                name="audio_frequency_detector",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    }
                ],
            ),
            ComposableNode(
                package="audio_capture",
                plugin="audio_capture::SnrGradientHomingNodeV2",
                name="snr_gradient_homing_v2",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    }
                ],
            ),
            ComposableNode(
                package="audio_capture",
                plugin="audio_capture::SnrGradientHomingControllerNode",
                name="snr_gradient_homing_controller",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "odometry_topic": odometry_topic,
                        "arena_width_m": ParameterValue(
                            arena_width_m, value_type=float
                        ),
                        "arena_height_m": ParameterValue(
                            arena_height_m, value_type=float
                        ),
                        "arena_start_corner": arena_start_corner,
                        "arena_yaw_rad": ParameterValue(
                            arena_yaw_rad, value_type=float
                        ),
                        "arena_start_inset_m": ParameterValue(
                            arena_start_inset_m, value_type=float
                        ),
                        "stuck_progress_threshold_mps": ParameterValue(
                            stuck_progress_threshold_mps, value_type=float
                        ),
                        "stuck_yaw_rate_threshold_rps": ParameterValue(
                            stuck_yaw_rate_threshold_rps, value_type=float
                        ),
                        "collision_hold_s": ParameterValue(
                            collision_hold_s, value_type=float
                        ),
                        "wall_reverse_command": ParameterValue(
                            wall_reverse_command, value_type=float
                        ),
                        "wall_reverse_duration_s": ParameterValue(
                            wall_reverse_duration_s, value_type=float
                        ),
                        "wall_escape_forward_command": ParameterValue(
                            wall_escape_forward_command, value_type=float
                        ),
                        "wall_escape_duration_s": ParameterValue(
                            wall_escape_duration_s, value_type=float
                        ),
                        "collision_rearm_timeout_s": ParameterValue(
                            collision_rearm_timeout_s, value_type=float
                        ),
                        "wall_homing_resume_grace_s": ParameterValue(
                            wall_homing_resume_grace_s, value_type=float
                        ),
                        "initial_diagonal_command": ParameterValue(
                            initial_diagonal_command, value_type=float
                        ),
                        "initial_diagonal_duration_s": ParameterValue(
                            initial_diagonal_duration_s, value_type=float
                        ),
                        "vertical_search_distance_m": ParameterValue(
                            vertical_search_distance_m, value_type=float
                        ),
                        "vertical_search_min_z_m": ParameterValue(
                            vertical_search_min_z_m, value_type=float
                        ),
                        "vertical_search_max_z_m": ParameterValue(
                            vertical_search_max_z_m, value_type=float
                        ),
                        "required_direction_frame": "base_link",
                        "search_forward": ParameterValue(search_forward, value_type=float),
                        "search_yaw": ParameterValue(search_yaw, value_type=float),
                        "search_turn_sign": ParameterValue(search_turn_sign, value_type=float),
                        "forward_fast": ParameterValue(forward_fast, value_type=float),
                        "forward_mid": ParameterValue(forward_mid, value_type=float),
                        "forward_slow": ParameterValue(forward_slow, value_type=float),
                        "confidence_speed_floor": ParameterValue(
                            confidence_speed_floor, value_type=float
                        ),
                        "invert_rc_yaw": ParameterValue(invert_rc_yaw, value_type=bool),
                        "invert_rc_lateral": ParameterValue(
                            invert_rc_lateral, value_type=bool
                        ),
                        "rc_override_topic": rc_override_topic,
                        "rc_preview_topic": rc_preview_topic,
                    }
                ],
            ),
        ],
        output="screen",
    )

    return LaunchDescription(arguments + [container])
