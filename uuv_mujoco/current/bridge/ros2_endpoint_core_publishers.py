"""Core simulator ROS2 publisher endpoint construction."""

from __future__ import annotations


def create_core_sensor_publishers(bridge, *, q10) -> None:
    node = bridge.node
    # MuJoCo is the sole simulation-time authority.  Downstream nodes use
    # ``use_sim_time:=true`` and must never infer time from wall-clock arrival.
    bridge.pub_clock = node.create_publisher(bridge.Clock, "/clock", q10)
    # A private clock keeps this simulator deterministic even when another
    # Gazebo/MuJoCo instance is visible on the same ROS domain.  Simulator
    # consumers remap their internal /clock subscription to this topic.
    bridge.pub_uuv_mujoco_clock = node.create_publisher(
        bridge.Clock, "/uuv_mujoco/clock", q10
    )
    if bridge._real_pkg_compat:
        bridge.pub_imu = None
        bridge.pub_depth = None
        bridge.pub_depth_pose = None
        bridge.pub_bar30_pressure = None
        bridge.pub_dvl_velocity = None
        bridge.pub_dvl_twist = None
        bridge.pub_dvl_altitude = None
        bridge.pub_dvl_odometry = None
        bridge.pub_rovio_odometry = None
        bridge.pub_odometry_filtered = None
    else:
        bridge.pub_imu = node.create_publisher(bridge.Imu, "/imu/data", q10)
        bridge.pub_depth = node.create_publisher(bridge.Float32, "/depth", q10)
        bridge.pub_depth_pose = node.create_publisher(bridge.PoseWithCovarianceStamped, "/depth/pose", q10)
        bridge.pub_bar30_pressure = node.create_publisher(bridge.Float32, "/bar30/pressure_pa", q10)
        bridge.pub_dvl_velocity = node.create_publisher(bridge.TwistStamped, "/dvl/velocity", q10)
        bridge.pub_dvl_twist = node.create_publisher(bridge.TwistWithCovarianceStamped, "/dvl/twist", q10)
        bridge.pub_dvl_altitude = node.create_publisher(bridge.Range, "/dvl/altitude", q10)
        bridge.pub_dvl_odometry = node.create_publisher(bridge.Odometry, "/dvl/odometry", q10)
        bridge.pub_rovio_odometry = node.create_publisher(bridge.Odometry, "/rovio/odometry", q10)
        bridge.pub_odometry_filtered = (
            node.create_publisher(bridge.Odometry, "/odometry/filtered", q10)
            if bool(
                getattr(
                    bridge,
                    "_unsafe_legacy_ground_truth_odometry_filtered",
                    False,
                )
            )
            else None
        )
    bridge.pub_sim_odometry = node.create_publisher(bridge.Odometry, "/sim/odom", q10)
    bridge.pub_battery = node.create_publisher(bridge.BatteryState, "/battery", q10)
    bridge.pub_mujoco_sim_time = node.create_publisher(bridge.Float32, "/mujoco/sim_time", q10)
    bridge.pub_ground_truth = node.create_publisher(bridge.PoseStamped, "/mujoco/ground_truth/pose", q10)
    bridge.pub_course_buoy_status = node.create_publisher(bridge.String, "/mujoco/course_buoys/status", q10)
    bridge.pub_collector_state = (
        node.create_publisher(bridge.CollectorState, "/collector/state", q10)
        if bridge.CollectorState else None
    )


def create_sitl_status_publishers(bridge, *, q10) -> None:
    node = bridge.node
    bridge.pub_sitl_sensor_replay_status = node.create_publisher(
        bridge.String,
        "/uuv_mujoco/sitl/sensor_replay_status",
        q10,
    )
    bridge.pub_sitl_mavlink_telemetry_status = node.create_publisher(
        bridge.String,
        "/uuv_mujoco/sitl/mavlink_telemetry_status",
        q10,
    )


def create_core_publishers(bridge, *, q10) -> None:
    create_core_sensor_publishers(bridge, q10=q10)
    create_sitl_status_publishers(bridge, q10=q10)


__all__ = [
    "create_core_sensor_publishers",
    "create_sitl_status_publishers",
    "create_core_publishers",
]
