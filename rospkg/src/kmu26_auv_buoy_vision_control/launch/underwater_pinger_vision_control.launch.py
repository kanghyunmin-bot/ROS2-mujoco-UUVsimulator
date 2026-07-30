from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "force_control_grant",
                default_value="true",
                description=(
                    "true for standalone vision control; false for acoustic handoff"
                ),
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("work_depth_m", default_value="8.65"),
            DeclareLaunchArgument("max_depth_m", default_value="10.5"),
            DeclareLaunchArgument(
                "physical_target_id",
                default_value="course_buoy_pinger_white_1_float",
            ),
            Node(
                package="auv_buoy_vision_control",
                executable="pinger_detach_monitor",
                name="underwater_pinger_detach_monitor",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                        "status_topic": "/mujoco/course_buoys/status",
                        "detached_topic": "/vision/pinger_detached",
                        "target_id": LaunchConfiguration("physical_target_id"),
                    }
                ],
            ),
            Node(
                package="auv_buoy_vision_control",
                executable="mission_state_machine_node",
                name="underwater_pinger_vision_controller",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                        "bbox_topic": "/vision/buoy_bbox",
                        "depth_topic": "/depth",
                        "depth_pose_topic": "/depth/pose",
                        "depth_pose_scale": -1.0,
                        "vision_search_request_topic": (
                            "/homing/vision_search_active"
                        ),
                        "target_confirmed_topic": "/vision/target_confirmed",
                        "vision_control_granted_topic": (
                            "/homing/vision_control_granted"
                        ),
                        "physical_detached_topic": "/vision/pinger_detached",
                        "force_control_grant": ParameterValue(
                            LaunchConfiguration("force_control_grant"),
                            value_type=bool,
                        ),
                        "require_physical_detach": True,
                        "single_target_mode": True,
                        "work_depth_m": ParameterValue(
                            LaunchConfiguration("work_depth_m"), value_type=float
                        ),
                        "surface_depth_m": 0.4,
                        "max_depth_m": ParameterValue(
                            LaunchConfiguration("max_depth_m"), value_type=float
                        ),
                        "buoy_class_id": 0,
                        "stick_class_id": 1,
                        "approach_area_ratio": 0.008,
                        "approach_forward_pwm": 1620,
                        "approach_forward_min_pwm": 1540,
                        "approach_motion_deadband_x": 0.16,
                        "approach_motion_deadband_y": 0.16,
                        "max_tracking_depth_delta": 250,
                        # Competition SITL uses RC4_DZ=40 and THR_DZ=100.
                        # Keep visual corrections just outside those FCU
                        # deadzones until the image alignment gate is met.
                        "min_effective_yaw_delta_pwm": 45,
                        "min_effective_vertical_delta_pwm": 110,
                        # At the 1.3 m camera-to-target insertion range,
                        # x=0.44 projects the PVC onto the port rake centre
                        # (local y about +0.16 m).  x=0.30 leaves about
                        # 0.48 m lateral offset and the straight insert pulse
                        # passes outside the rake.
                        "fork_target_x": 0.44,
                        # The pinger PVC contact is only 0.062 m below the
                        # forward camera (rake z=-0.082, camera z=-0.020).
                        # At the insertion range it projects near y=0.54.
                        # A lower image target such as 0.62 makes ALT_HOLD
                        # settle inside its RC deadband before ALIGN can pass.
                        "fork_target_y": 0.54,
                        "stick_deadband_x": 0.04,
                        "stick_deadband_y": 0.08,
                        "align_stable_sec": 0.3,
                        "insert_pwm": 1700,
                        "insert_duration_sec": 2.0,
                        "detach_pwm": 1700,
                        "detach_duration_sec": 1.0,
                        "backoff_pwm": 1350,
                        "backoff_duration_sec": 1.0,
                        "search_yaw_pwm": 1550,
                        "buoyancy_hold_delta_pwm": 0,
                        "vertical_positive_is_up": True,
                    }
                ],
            ),
        ]
    )
