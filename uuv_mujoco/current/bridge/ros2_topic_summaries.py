"""Launch and runtime summary strings for ROS2 bridge topics."""

from __future__ import annotations


def build_bridge_topic_summary(
    *,
    enable_ping360: bool,
    real_pkg_compat: bool,
    strict_sitl_sensor_transport: bool = False,
    unsafe_legacy_ground_truth_odometry_filtered: bool = False,
) -> str:
    """Return the launch-time ROS2/MAVROS topic summary string."""
    if real_pkg_compat:
        strict_sensor_topics = (
            "/mavros/imu/data_raw, /mavros/imu/static_pressure, "
            if strict_sitl_sensor_transport
            else ""
        )
        bridge_topics = (
            "[bridge] strict real-package surface: "
            f"{strict_sensor_topics}/dvl/data, /dvl/position, /battery, "
            "/sim/odom(simulation-only ground-truth oracle; forbidden for estimator/control), "
            "/mujoco/ground_truth/pose, /mujoco/course_buoys/status, /collector/state, "
            "/mujoco/hydrophone/direction(diagnostic only), /mujoco/hydrophone/status, "
            "/audio, /audio_info, "
            "/camera/camera/color/image_raw, /camera/camera/color/image_raw/compressed, "
            "/camera/camera/color/camera_info, /imx219/camera0/image_raw, "
            "/imx219/camera0/image_raw/compressed, "
            "/imx219/camera0/camera_info, /imx219/camera1/image_raw, "
            "/imx219/camera1/image_raw/compressed, "
            "/imx219/camera1/camera_info, /tf_static, /robot_description; "
            "simulator_mavros_control_surface=disabled direct_command=disabled"
        )
        if strict_sitl_sensor_transport:
            bridge_topics += "; external_mavros_ahrs=/mavros/imu/data"
    else:
        bridge_topics = (
            "[bridge] enabled: /cmd_vel(TwistStamped) -> control, /imu/data, "
            "/dvl/velocity, /dvl/twist, /dvl/odometry, /dvl/altitude, /dvl/data, /dvl/position, "
            "/depth, /depth/pose, /bar30/pressure_pa, /rovio/odometry, "
            "/sim/odom(simulation-only ground-truth oracle; forbidden for estimator/control), "
            "/mujoco/ground_truth/pose, /mujoco/course_buoys/status, /collector/state, "
            "/mujoco/hydrophone/status, /mujoco/hydrophone/direction, /audio, /audio_info, "
            "/imx219/camera0/image_raw, /imx219/camera0/image_raw/compressed, "
            "/imx219/camera0/camera_info, /imx219/camera1/image_raw, "
            "/imx219/camera1/image_raw/compressed, /imx219/camera1/camera_info, "
            "/tf, /tf_static, /robot_description"
        )
    if enable_ping360:
        bridge_topics += (
            ", /ping360/image, /ping360/scan_image, /ping360/scan, /ping360/scan_echo, "
            "/ping360/echo, /ping360/status, /ping360/config"
        )
    if unsafe_legacy_ground_truth_odometry_filtered and not real_pkg_compat:
        bridge_topics += (
            ", /odometry/filtered(UNSAFE LEGACY exact-state alias; not an estimate)"
        )
    if not real_pkg_compat:
        bridge_topics += (
            ", /mavros/state, /mavros/imu/*, /mavros/vfr_hud, "
            "/mavros/local_position/pose, /mavros/local_position/velocity_local, "
            "/mavros/local_position/velocity_body, /mavros/local_position/velocity_body_cov, "
            "/mavros/local_position/odom, /mavros/vision_pose/pose, "
            "/mavros/battery, /mavros/rc/in, /mavros/rc/override, "
            "/mavros/setpoint_raw/local, /mavros/cmd/arming, /mavros/set_mode, "
            "/mavros/cmd/command"
        )
    return bridge_topics


def sitl_servo_mode_from_endpoint(endpoint: str | None) -> str:
    """Return the displayed SITL servo transport mode for a MAVLink endpoint value."""
    value = str(endpoint or "").strip().lower()
    if value in {"", "none", "off", "disabled", "disable"}:
        return "json"
    return "mavlink"


def build_sitl_transport_summary(*, sitl_mavlink_endpoint: str | None) -> str:
    """Return the launch-time SITL transport summary string."""
    return (
        "[bridge] SITL transport enabled "
        f"(sensor=UDP JSON, servo={sitl_servo_mode_from_endpoint(sitl_mavlink_endpoint)}, ROS2 disabled)."
    )


def build_ros2_bridge_active_log(
    *,
    mavros_surface_enabled: bool,
    strict_sitl_sensor_transport: bool = False,
    unsafe_legacy_ground_truth_odometry_filtered: bool = False,
) -> str:
    """Return the ROS node logger message for the active bridge surface."""
    if mavros_surface_enabled:
        summary = (
            "ROS2 bridge active (lightweight real-robot interface, full MAVROS surface): "
            "/cmd_vel(TwistStamped), /imu/data, /dvl/*, /rovio/odometry, /sim/odom, "
            "/audio, /audio_info, /mujoco/hydrophone/status, "
            "/mujoco/hydrophone/direction, "
            "/ping360/image, /ping360/scan_image, /ping360/scan, /ping360/scan_echo, /ping360/echo, "
            "/ping360/status, /ping360/config, /uuv_mujoco/sitl/command_override, "
            "/mavros/state, /mavros/imu/*, /mavros/vfr_hud, /mavros/local_position/*, "
            "/mavros/vision_pose/pose, /mavros/battery, /mavros/rc/in, /mavros/rc/out, "
            "/mavros/rc/override, /tf, /robot_description"
        )
        if unsafe_legacy_ground_truth_odometry_filtered:
            summary += (
                "; UNSAFE LEGACY exact MuJoCo state alias active on "
                "/odometry/filtered"
            )
        return summary
    strict_sensor_summary = (
        "/mavros/imu/data_raw, /mavros/imu/static_pressure "
        "(bridge delivery-driven), /mavros/imu/data (external MAVROS AHRS), "
        if strict_sitl_sensor_transport
        else ""
    )
    return (
        "ROS2 bridge active (strict external real-package compatibility): "
        f"{strict_sensor_summary}/dvl/data, /dvl/position, /imx219/camera0/*, "
        "/imx219/camera1/*, and raw sensor surfaces only; "
        "simulator MAVROS control/status surface, derived localization, "
        "dynamic odom TF, and direct command subscriptions are disabled"
    )


__all__ = [
    "build_bridge_topic_summary",
    "sitl_servo_mode_from_endpoint",
    "build_sitl_transport_summary",
    "build_ros2_bridge_active_log",
]
