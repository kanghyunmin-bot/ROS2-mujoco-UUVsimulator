from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _default_model_path() -> str:
    configured = os.environ.get("UUV_YOLO_MODEL")
    if configured:
        return configured

    relative_model = Path("sim/current/assets/yolo/best.pt")
    for root in (Path.cwd(), *Path(__file__).resolve().parents):
        candidate = root / relative_model
        if candidate.is_file():
            return str(candidate)
    return str(Path.cwd() / relative_model)


def generate_launch_description() -> LaunchDescription:
    detector_launch = os.path.join(
        get_package_share_directory("auv_buoy_vision_control"),
        "launch",
        "underwater_pinger_yolo.launch.py",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    odometry_topic = LaunchConfiguration("odometry_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "odometry_topic",
                default_value="/homing/sim_odom",
                description=(
                    "Use /homing/sim_odom for hydrophone integration. "
                    "Use /sim/odom only for a forced handoff test whose "
                    "/start_frame is published in the same world frame."
                ),
            ),
            DeclareLaunchArgument(
                "model_path",
                default_value=_default_model_path(),
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/camera/camera/color/image_raw/compressed",
            ),
            DeclareLaunchArgument("device", default_value="auto"),
            DeclareLaunchArgument("show_preview", default_value="false"),
            DeclareLaunchArgument("include_detector", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(detector_launch),
                condition=IfCondition(LaunchConfiguration("include_detector")),
                launch_arguments={
                    "model_path": LaunchConfiguration("model_path"),
                    "image_topic": LaunchConfiguration("image_topic"),
                    "device": LaunchConfiguration("device"),
                    "show_preview": LaunchConfiguration("show_preview"),
                }.items(),
            ),
            Node(
                package="auv_lane_vision_control",
                executable="course_buoy_detach_monitor",
                name="course_buoy_detach_monitor",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            use_sim_time, value_type=bool
                        ),
                    }
                ],
            ),
            Node(
                package="auv_lane_vision_control",
                executable="lane_vision_controller_node",
                name="competition_a_lane_vision_controller",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            use_sim_time, value_type=bool
                        ),
                        "odometry_topic": odometry_topic,
                        "bbox_topic": "/vision/buoy_bbox",
                        "depth_pose_topic": "/depth/pose",
                        "depth_topic": "/auv/depth",
                        "start_frame_topic": "/start_frame",
                        "physical_detach_count_topic": (
                            "/vision/course_buoy_detach_count"
                        ),
                        # This controller owns depth/yaw/forward RC loops.
                        # ALT_HOLD suppresses its direct vertical correction.
                        "required_fcu_mode": "STABILIZE",
                        # Competition map A half, represented in the
                        # hydrophone start frame.  Four 7.275 m-spaced sweeps
                        # cover world x [-17.05, -0.45], y [-14.55, 14.55].
                        "arena_length_m": 17.5,
                        "arena_width_m": 30.0,
                        "arena_offset_x_m": -1.619,
                        "arena_offset_y_m": 13.695,
                        "arena_safety_margin_m": 0.45,
                        "arena_start_corner": "bottom_left",
                        "lane_search_offset_m": 3.6375,
                        "expected_lane_count": 4,
                        "max_depth_m": 10.5,
                        "initial_search_radius_m": 3.0,
                        "initial_scan_yaw_pwm": 1700,
                        "initial_target_reacquire_timeout_sec": 8.0,
                        # Far, edge-of-frame lane targets need enough time for
                        # the vehicle to rotate through the camera FOV.
                        "target_reacquire_timeout_sec": 8.0,
                        "detection_timeout_sec": 0.8,
                        # External MAVROS pressure arrives around 1.3 Hz under
                        # CPU YOLO load; a 1 s timeout rejects healthy samples.
                        "depth_timeout_sec": 3.0,
                        "target_confirm_hits": 3,
                        "target_confirm_sec": 0.25,
                        "buoy_class_id": 0,
                        "stick_class_id": 1,
                        "align_target_x": 0.50,
                        "align_target_y": 0.50,
                        "align_deadband_x": 0.08,
                        "align_deadband_y": 0.10,
                        "approach_area_ratio": 0.0035,
                        "approach_forward_pwm": 1620,
                        "approach_forward_min_pwm": 1540,
                        # Keep the PVC on the innermost port rake at contact.
                        # Continuous visual correction during insertion makes
                        # this an approach angle instead of a far-field miss.
                        "fork_target_x": 0.44,
                        "fork_target_y": 0.54,
                        "lane_fork_target_y": 0.54,
                        # Lane targets keep the pre-approach coverage depth;
                        # vertical bbox control is intentionally disabled.
                        "lane_fork_depth_offset_m": 0.0,
                        "stick_deadband_x": 0.04,
                        "stick_deadband_y": 0.08,
                        "align_stable_sec": 0.30,
                        "insert_fork_pwm": 1700,
                        "insert_fork_duration_sec": 2.0,
                        "detach_pwm": 1700,
                        "detach_duration_sec": 1.0,
                        "go_back_pwm": 1350,
                        "go_back_duration_sec": 1.0,
                        "verify_timeout_sec": 3.0,
                        "max_target_retries": 2,
                        "require_physical_detach": True,
                        "lane_forward_pwm": 1660,
                        # In STABILIZE, +/-45 to 56 PWM did not overcome the
                        # simulated yaw dead zone; +/-80 produced a measured
                        # turn while remaining below the 180 PWM limit.
                        "min_effective_yaw_delta_pwm": 80,
                        "min_effective_vertical_delta_pwm": 110,
                        "max_vision_throttle_delta_pwm": 250,
                        "vertical_full_weight_error": 0.25,
                        "buoyancy_hold_delta_pwm": 0,
                        "depth_deadband_m": 0.04,
                    }
                ],
            ),
        ]
    )
