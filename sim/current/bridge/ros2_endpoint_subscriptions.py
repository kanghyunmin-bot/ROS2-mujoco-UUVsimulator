"""ROS2 subscription endpoint construction."""

from __future__ import annotations

from .ros2_hydrophone_sim import create_hydrophone_subscriptions
from .ros2_mission_contract import SCORE_RELEASE_TOPIC


def create_core_subscriptions(bridge, *, q10) -> None:
    node = bridge.node
    command_qos = 1
    bridge.sub_cmd_vel = None
    bridge.sub_sitl_command_override = None
    if not bridge._real_pkg_compat:
        bridge.sub_cmd_vel = node.create_subscription(
            bridge.TwistStamped,
            "/cmd_vel",
            bridge._on_cmd_vel_stamped,
            command_qos,
        )
        bridge.sub_sitl_command_override = node.create_subscription(
            bridge.String,
            "/uuv_mujoco/sitl/command_override",
            bridge._on_sitl_command_override,
            command_qos,
        )
    bridge.sub_ping360_config = node.create_subscription(
        bridge.String,
        "/ping360/config",
        bridge._on_ping360_config,
        q10,
    )
    bridge.sub_score_release_contract = node.create_subscription(
        bridge.String,
        SCORE_RELEASE_TOPIC,
        bridge._on_score_release_contract,
        1,
    )


def create_mavros_subscriptions(bridge, *, q10) -> None:
    node = bridge.node
    command_qos = 1
    if bridge._mavros_surface_enabled:
        bridge.sub_mavros_rc_override = node.create_subscription(
            bridge.OverrideRCIn,
            "/mavros/rc/override",
            bridge._on_mavros_rc_override,
            command_qos,
        )
        bridge.sub_replay_rcout_override = (
            node.create_subscription(
                bridge.RCOut,
                "/uuv_mujoco/rc/out_override",
                bridge._on_replay_rcout_override,
                command_qos,
            )
            if bridge.RCOut and bridge._allow_rcout_plant_override
            else None
        )
        bridge.sub_mavros_manual_control = node.create_subscription(
            bridge.ManualControl,
            "/mavros/manual_control/send",
            bridge._on_mavros_manual_control,
            command_qos,
        )
        bridge.sub_mavros_setpoint = (
            node.create_subscription(
                bridge.PositionTarget,
                "/mavros/setpoint_raw/local",
                bridge._on_mavros_setpoint,
                command_qos,
            )
            if bridge._mavros_setpoint_enabled
            else None
        )
        return

    bridge.sub_mavros_rc_override = None
    bridge.sub_replay_rcout_override = None
    bridge.sub_mavros_manual_control = None
    bridge.sub_mavros_setpoint = None


def create_ros2_subscriptions(bridge, *, q10) -> None:
    create_core_subscriptions(bridge, q10=q10)
    create_hydrophone_subscriptions(bridge, q10=q10)
    create_mavros_subscriptions(bridge, q10=q10)


__all__ = ["create_core_subscriptions", "create_mavros_subscriptions", "create_ros2_subscriptions"]
