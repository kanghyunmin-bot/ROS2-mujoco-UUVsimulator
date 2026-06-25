"""SITL transport construction helpers for Ros2Bridge."""

from __future__ import annotations

from typing import Any

from .sitl_env import env_to_float, env_to_int
from .sitl_transport import SitlTransport


def create_sitl_transport_if_enabled(
    bridge: Any,
    *,
    enable_sitl: bool,
    sitl_ip: str,
    sitl_port: int,
    sitl_send_port: int,
    sitl_mavlink_endpoint: str,
    sitl_mavlink_servo_hz: float,
    sitl_mavlink_target_sysid: int,
    sitl_mavlink_target_compid: int,
    sitl_mavlink_source_sysid: int,
    sitl_mavlink_source_compid: int,
) -> Any | None:
    """Create the optional MuJoCo <-> ArduSub SITL transport."""

    bridge._sitl_cmd_vel_warned = False
    bridge._sitl_prev_vel_sim_t = None
    bridge._sitl_prev_vel_enu = None
    if not enable_sitl:
        return None
    return SitlTransport(
        model=bridge.model,
        sitl_ip=sitl_ip,
        sitl_port=int(sitl_port),
        sitl_send_port=int(sitl_send_port),
        sitl_mavlink_endpoint=sitl_mavlink_endpoint,
        sitl_mavlink_servo_hz=float(sitl_mavlink_servo_hz),
        sitl_mavlink_target_sysid=int(sitl_mavlink_target_sysid),
        sitl_mavlink_target_compid=int(sitl_mavlink_target_compid),
        sitl_mavlink_source_sysid=int(sitl_mavlink_source_sysid),
        sitl_mavlink_source_compid=int(sitl_mavlink_source_compid),
        enu_to_ned=bridge._enu_to_ned,
        surface_pressure_pa=bridge._bar30_surface_pressure_pa,
        water_density=bridge._bar30_water_density,
        gravity=bridge._bar30_gravity,
        home_alt_m=bridge._sitl_home_alt_m,
        rangefinder_max_m=float(env_to_float("ROS2_UUV_SITL_RANGEFINDER_MAX_M", 30.0)),
        command_debug=bool(env_to_int("ROS2_UUV_SITL_CMD_DEBUG", 0)),
    )


__all__ = ["create_sitl_transport_if_enabled"]
