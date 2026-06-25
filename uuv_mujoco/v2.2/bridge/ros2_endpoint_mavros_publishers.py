"""MAVROS-compatible ROS2 publisher endpoint construction."""

from __future__ import annotations

_MAVROS_DISABLED_PUBLISHER_ATTRS = (
    "pub_mavros_state",
    "pub_mavros_imu_data",
    "pub_mavros_imu_data_raw",
    "pub_mavros_imu_static_pressure",
    "pub_mavros_imu_atm_pressure",
    "pub_mavros_local_pose",
    "pub_mavros_local_odom",
    "pub_mavros_local_vel",
    "pub_mavros_local_vel_body",
    "pub_mavros_local_vel_body_cov",
    "pub_mavros_vision_pose",
    "pub_mavros_battery",
    "pub_mavros_rc_in",
    "pub_mavros_rc_out",
)


def disable_mavros_publishers(bridge) -> None:
    for attr in _MAVROS_DISABLED_PUBLISHER_ATTRS:
        setattr(bridge, attr, None)


def create_mavros_publishers(bridge, *, q10) -> None:
    node = bridge.node
    bridge.pub_mavros_vfr_hud = node.create_publisher(bridge.VfrHud, "/mavros/vfr_hud", q10)
    if not bridge._mavros_surface_enabled:
        disable_mavros_publishers(bridge)
        return

    bridge.pub_mavros_state = node.create_publisher(bridge.MavrosState, "/mavros/state", q10)
    bridge.pub_mavros_imu_data = node.create_publisher(bridge.Imu, "/mavros/imu/data", q10)
    bridge.pub_mavros_imu_data_raw = node.create_publisher(bridge.Imu, "/mavros/imu/data_raw", q10)
    bridge.pub_mavros_imu_static_pressure = node.create_publisher(
        bridge.FluidPressure,
        "/mavros/imu/static_pressure",
        q10,
    )
    bridge.pub_mavros_imu_atm_pressure = node.create_publisher(
        bridge.FluidPressure,
        "/mavros/imu/atm_pressure",
        q10,
    )
    bridge.pub_mavros_local_pose = node.create_publisher(
        bridge.PoseStamped,
        "/mavros/local_position/pose",
        q10,
    )
    bridge.pub_mavros_local_odom = node.create_publisher(
        bridge.Odometry,
        "/mavros/local_position/odom",
        q10,
    )
    bridge.pub_mavros_local_vel = node.create_publisher(
        bridge.TwistStamped,
        "/mavros/local_position/velocity_local",
        q10,
    )
    bridge.pub_mavros_local_vel_body = node.create_publisher(
        bridge.TwistStamped,
        "/mavros/local_position/velocity_body",
        q10,
    )
    bridge.pub_mavros_local_vel_body_cov = node.create_publisher(
        bridge.TwistWithCovarianceStamped,
        "/mavros/local_position/velocity_body_cov",
        q10,
    )
    bridge.pub_mavros_vision_pose = node.create_publisher(
        bridge.PoseStamped,
        "/mavros/vision_pose/pose",
        q10,
    )
    bridge.pub_mavros_battery = node.create_publisher(bridge.BatteryState, "/mavros/battery", q10)
    bridge.pub_mavros_rc_in = node.create_publisher(bridge.RCIn, "/mavros/rc/in", q10)
    bridge.pub_mavros_rc_out = (
        node.create_publisher(bridge.RCOut, "/mavros/rc/out", q10)
        if bridge.RCOut
        else None
    )


__all__ = ["create_mavros_publishers", "disable_mavros_publishers"]
