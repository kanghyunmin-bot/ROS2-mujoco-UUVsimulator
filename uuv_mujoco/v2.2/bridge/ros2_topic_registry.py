"""Central registry for ROS2 publishers, subscribers, and services."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class PublisherSpec:
    name: str
    msg_symbol: str
    topic: str
    qsize: int


@dataclass(frozen=True)
class SubscriberSpec:
    name: str
    msg_symbol: str
    topic: str
    callback_name: str
    qsize: int


@dataclass(frozen=True)
class ServiceSpec:
    name: str
    srv_symbol: str
    topic: str
    callback_name: str


PUBLISHER_SPECS: tuple[PublisherSpec, ...] = (
    PublisherSpec("imu", "Imu", "/imu/data", 1),
    PublisherSpec("dvl_vel", "TwistStamped", "/dvl/velocity", 1),
    PublisherSpec("dvl_alt", "Range", "/dvl/altitude", 1),
    PublisherSpec("depth", "Float32", "/depth", 1),
    PublisherSpec("bar30_pressure", "Float32", "/bar30/pressure_pa", 1),
    PublisherSpec("dvl_odom", "Odometry", "/dvl/odometry", 1),
    PublisherSpec("rovio_odom", "Odometry", "/rovio/odometry", 1),
    PublisherSpec("ground_truth", "PoseStamped", "/mujoco/ground_truth/pose", 1),
    PublisherSpec("ping360_image", "Image", "/ping360/image", 1),
    PublisherSpec("ping360_scan_image", "Image", "/ping360/scan_image", 1),
    PublisherSpec("ping360_scan", "LaserScan", "/ping360/scan", 10),
    PublisherSpec("ping360_scan_echo", "SonarEcho", "/ping360/scan_echo", 10),
    PublisherSpec("ping360_echo", "SonarEcho", "/ping360/echo", 10),
    PublisherSpec("ping360_status", "String", "/ping360/status", 10),
    PublisherSpec("tf", "TFMessage", "/tf", 1),
    PublisherSpec("tf_static", "TFMessage", "/tf_static", 1),
    PublisherSpec("robot_description", "String", "/robot_description", 1),
    PublisherSpec("mavros_imu_data_raw", "Imu", "/mavros/imu/data_raw", 1),
    PublisherSpec("mavros_local_pose", "PoseStamped", "/mavros/local_position/pose", 1),
    PublisherSpec("mavros_local_vel", "TwistStamped", "/mavros/local_position/velocity_local", 1),
    PublisherSpec("mavros_local_odom", "Odometry", "/mavros/local_position/odom", 1),
    PublisherSpec("mavros_state", "MavrosState", "/mavros/state", 1),
    PublisherSpec("mavros_vision_pose", "PoseStamped", "/mavros/vision_pose/pose", 1),
    PublisherSpec("mavros_vfr_hud", "VfrHud", "/mavros/vfr_hud", 1),
    PublisherSpec("mavros_imu_atm_pressure", "FluidPressure", "/mavros/imu/atm_pressure", 1),
    PublisherSpec("mavros_battery", "BatteryState", "/mavros/battery", 1),
    PublisherSpec("mavros_rc_in", "RCIn", "/mavros/rc/in", 1),
    PublisherSpec("dvl_data", "DVLMsg", "/dvl/data", 1),
    PublisherSpec("dvl_position", "DVLDRMsg", "/dvl/position", 1),
)


SUBSCRIBER_SPECS: tuple[SubscriberSpec, ...] = (
    SubscriberSpec("cmd", "TwistStamped", "/cmd_vel", "_on_cmd_vel_stamped", 1),
    SubscriberSpec("ping360_config", "String", "/ping360/config", "_on_ping360_config", 10),
    SubscriberSpec(
        "mavros_setpoint",
        "PositionTarget",
        "/mavros/setpoint_raw/local",
        "_on_mavros_setpoint",
        10,
    ),
    SubscriberSpec(
        "mavros_rc_override",
        "OverrideRCIn",
        "/mavros/rc/override",
        "_on_mavros_rc_override",
        10,
    ),
    SubscriberSpec(
        "mavros_manual_control",
        "ManualControl",
        "/mavros/manual_control/send",
        "_on_mavros_manual_control",
        10,
    ),
)


SERVICE_SPECS: tuple[ServiceSpec, ...] = (
    ServiceSpec("mavros_cmd_arming", "MavrosCommandBool", "/mavros/cmd/arming", "_on_mavros_cmd_arming"),
    ServiceSpec("mavros_set_mode", "MavrosSetMode", "/mavros/set_mode", "_on_mavros_set_mode"),
    ServiceSpec("mavros_command", "MavrosCommandLong", "/mavros/cmd/command", "_on_mavros_command_long"),
)


CORE_BRIDGE_TOPICS: tuple[str, ...] = (
    "/cmd_vel(TwistStamped) -> control",
    "/imu/data",
    "/dvl/velocity",
    "/dvl/odometry",
    "/dvl/altitude",
    "/dvl/data",
    "/dvl/position",
    "/depth",
    "/bar30/pressure_pa",
    "/rovio/odometry",
    "/mujoco/ground_truth/pose",
    "/ping360/image",
    "/ping360/scan_image",
    "/ping360/scan",
    "/ping360/scan_echo",
    "/ping360/echo",
    "/ping360/status",
    "/ping360/config(String) -> runtime sonar params",
    "/tf",
    "/tf_static",
    "/robot_description",
)


MAVROS_SURFACE_TOPIC_LABELS: tuple[str, ...] = (
    "/mavros/vfr_hud",
    "/mavros/imu/data_raw",
    "/mavros/local_position/pose",
    "/mavros/local_position/velocity_local",
    "/mavros/local_position/odom",
    "/mavros/state",
    "/mavros/cmd/arming",
    "/mavros/set_mode",
    "/mavros/cmd/command",
    "/mavros/setpoint_raw/local",
    "/mavros/vision_pose/pose",
    "/mavros/imu/atm_pressure",
    "/mavros/battery",
    "/mavros/rc/in",
)
