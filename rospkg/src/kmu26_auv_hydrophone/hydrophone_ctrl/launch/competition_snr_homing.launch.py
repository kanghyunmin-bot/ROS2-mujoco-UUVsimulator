import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    start_immediately = LaunchConfiguration("start_immediately")
    vision_handoff_enabled = LaunchConfiguration("vision_handoff_enabled")
    raw_sim_odometry_topic = "/sim/odom"
    odometry_topic = "/homing/sim_odom"
    start_frame_topic = "/start_frame"
    snr_topic = "/audio_frequency_detector/snr_db_stamped"

    # tank_current_scene.xml (current default spawn world=-15.881, 1.305):
    # physical pool in the rebased default-start frame:
    #   x=[-1.619, 33.381], y=[-16.305, 13.695]
    # A-side (our half) center:
    #   world=(-8.750, 0.000), start-frame=(7.131, -1.305)
    common = {
        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
        "odometry_topic": odometry_topic,
        "start_frame_topic": start_frame_topic,
        "arena_frame_id": "arena",
    }
    arena = {
        "arena_length_m": 35.0,
        "arena_width_m": 30.0,
        "arena_offset_x_m": -1.619,
        "arena_offset_y_m": 13.695,
        "arena_safety_margin_m": 0.45,
        "arena_start_corner": "bottom_left",
    }

    detector = ComposableNode(
        package="audio_capture",
        plugin="audio_capture::AudioFrequencyDetectorNode",
        name="audio_frequency_detector",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                # Calculation reference only; waveform noise remains zero.
                "noise_floor_reference_magnitude": 0.05,
            }
        ],
    )
    estimator = ComposableNode(
        package="hydrophone_ctrl",
        plugin="audio_capture::RegionLocalGradientEstimatorNode",
        name="region_local_gradient_estimator",
        parameters=[
            {
                **common,
                "snr_topic": snr_topic,
                "state_topic": "/homing/control_state",
                "region_gradient_topic": "/homing/region_gradient",
                "rolling_gradient_topic": "/homing/rolling_gradient",
                "region_sample_spacing_m": 0.10,
                "homing_gradient_window_size": 16,
                "min_homing_gradient_samples": 8,
                "min_region_gradient_magnitude": 0.02,
                "min_region_lateral_spread_m": 0.10,
                "odometry_timeout_s": 0.70,
                "max_snr_odom_skew_s": 0.20,
            }
        ],
    )
    controller = ComposableNode(
        package="hydrophone_ctrl",
        plugin="audio_capture::WaypointHomingControllerNode",
        name="waypoint_homing_controller",
        parameters=[
            {
                **common,
                **arena,
                "snr_topic": snr_topic,
                "state_topic": "/homing/control_state",
                "region_gradient_topic": "/homing/region_gradient",
                "rolling_gradient_topic": "/homing/rolling_gradient",
                "homing_direction_topic": "/homing/homing_direction",
                "direction_body_topic": "/mission/hydrophone/direction_body",
                "waypoint_topic": "/homing/current_waypoint",
                "scan_center_topic": "/homing/scan_center",
                "rc_override_topic": "/mavros/rc/override",
                "use_explicit_initial_scan_center": True,
                "initial_scan_center_x_m": 7.131,
                "initial_scan_center_y_m": -1.305,
                "initial_scan_radius_m": 1.30,
                "rescan_radius_m": 0.80,
                "homing_waypoint_step_m": 0.70,
                "homing_zigzag_offset_m": 0.20,
                "rolling_gradient_alpha": 0.15,
                "rolling_gradient_conflict_angle_rad": 1.5708,
                "rolling_gradient_conflict_limit": 6,
                "waypoint_reach_tolerance_m": 0.22,
                # Slow the RC arc enough for body yaw to keep up with lateral
                # motion.  The prior 0.45 sway limit completed the last
                # quadrant with ~150 deg yaw lag and expanded to 1.96 m.
                "scan_lookahead_rad": 0.20,
                "scan_xy_gain": 0.70,
                "scan_sway_gain": 0.75,
                "scan_sway_limit": 0.28,
                "scan_heading_gate_rad": 0.55,
                "scan_completion_radius_tolerance_m": 0.30,
                "vision_near_zone_width_m": 0.0,
                "vision_handoff_enabled": ParameterValue(
                    vision_handoff_enabled, value_type=bool
                ),
                "acoustic_timeout_s": 0.0,
                # /homing/sim_odom z is rebased to zero at the default spawn;
                # the pinger acoustic site is about 0.30 m deeper.
                "target_depth_z_m": -0.30,
                "depth_tolerance_m": 0.12,
                "depth_kp": 0.8,
                "depth_ki": 0.12,
                "forward_cruise": 0.32,
                "yaw_kp": 1.15,
                "yaw_ki": 0.12,
                "yaw_kd": 0.06,
                "yaw_limit": 0.68,
                "move_heading_tolerance_rad": 0.24,
                "rc_pwm_span": 400.0,
                "invert_rc_yaw": True,
                # Controller sway is ROS FLU-left, while ArduSub RC6 is
                # body-FRD right.  Invert once at the RC boundary.
                "invert_rc_lateral": True,
                "success_snr_db": 70.0,
                "success_hold_s": 1.0,
                "success_snr_timeout_s": 1.0,
                "success_snr_window_size": 15,
                "enable_keyboard_emergency_stop": False,
                "rate_hz": 30.0,
            }
        ],
    )

    pipeline = ComposableNodeContainer(
        name="competition_snr_homing_pipeline",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        output="screen",
        composable_node_descriptions=[detector, estimator, controller],
    )

    sim_odometry_rebaser = Node(
        package="audio_capture",
        executable="sim_odometry_rebaser",
        name="competition_sim_odometry_rebaser",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "input_topic": raw_sim_odometry_topic,
                "output_topic": odometry_topic,
                "output_frame": "odom",
                "zero_z": True,
            }
        ],
    )

    visualizer = Node(
        package="hydrophone_ctrl",
        executable="region_local_gradient_rviz_visualizer",
        name="competition_snr_rviz_visualizer",
        output="screen",
        parameters=[
            {
                **common,
                **arena,
                "snr_topic": snr_topic,
                "state_topic": "/homing/control_state",
                "region_gradient_topic": "/homing/region_gradient",
                "rolling_gradient_topic": "/homing/rolling_gradient",
                "homing_direction_topic": "/homing/homing_direction",
                "waypoint_topic": "/homing/current_waypoint",
                "scan_center_topic": "/homing/scan_center",
                "marker_topic": "/homing/rviz/markers",
                "scan_radius_m": 1.30,
                "owned_region_enabled": True,
                "owned_region_x_min_m": -1.619,
                "owned_region_x_max_m": 15.881,
                "owned_region_y_min_m": -16.305,
                "owned_region_y_max_m": 13.695,
                "vision_near_zone_width_m": 0.0,
                "map_cell_size_m": 0.12,
                "arrow_length_m": 1.20,
                "publish_rate_hz": 8.0,
                "dynamic_snr_range": True,
                "trajectory_spacing_m": 0.04,
            }
        ],
    )

    rviz_config = os.path.join(
        get_package_share_directory("hydrophone_ctrl"),
        "rviz",
        "region_local_gradient.rviz",
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="competition_snr_rviz",
        arguments=["-d", rviz_config],
        parameters=[
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}
        ],
        output="screen",
        condition=IfCondition(use_rviz),
    )

    start_frame = Node(
        package="hydrophone_ctrl",
        executable="start_frame_publisher",
        name="competition_start_frame_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "odometry_topic": odometry_topic,
                "start_frame_topic": start_frame_topic,
                "frame_id": "odom",
                "publish_rate_hz": 1.0,
            }
        ],
        condition=IfCondition(start_immediately),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "vision_handoff_enabled",
                default_value="false",
                description=(
                    "Keep false for hydrophone-only SUCCESS; true for the "
                    "separate underwater vision handoff test."
                ),
            ),
            DeclareLaunchArgument(
                "start_immediately",
                default_value="false",
                description=(
                    "Keep false for safe startup; supply /start_frame only "
                    "after STABILIZE and arming."
                ),
            ),
            sim_odometry_rebaser,
            pipeline,
            visualizer,
            rviz,
            start_frame,
        ]
    )
