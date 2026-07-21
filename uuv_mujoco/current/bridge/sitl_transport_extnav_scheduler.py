"""ExternalNav scheduler policy for SitlTransport."""

from __future__ import annotations

import os


def configure_extnav_scheduler(transport: object) -> None:
    extnav_scheduler = os.getenv("ROS2_UUV_SITL_EXTNAV_SCHEDULER", "").strip().lower()
    if extnav_scheduler in {"sim", "sim_time"}:
        transport._sitl_extnav_scheduler = "sim_time"
    elif extnav_scheduler in {"wall", "wall_time", "wall_clock", "realtime", "real_time"}:
        transport._sitl_extnav_scheduler = "wall_time"
    else:
        transport._sitl_extnav_scheduler = (
            "sim_time" if transport._sensor_replay_frames or transport._native_vpd_events else "wall_time"
        )
    if transport._sitl_extnav_scheduler == "wall_time" and os.getenv("ROS2_UUV_SITL_EXTNAV_HZ") in (None, ""):
        transport._sitl_extnav_rate_hz = max(float(transport._sitl_extnav_rate_hz), 15.0)


__all__ = ["configure_extnav_scheduler"]
