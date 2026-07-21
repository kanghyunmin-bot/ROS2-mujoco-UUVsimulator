"""Auto-ready state configuration for SitlTransport."""

from __future__ import annotations

import os

from bridge.sitl_env import env_flag


def initialize_auto_ready_state(transport: object) -> None:
    legacy_auto_safe = env_flag("SITL_AUTO_SAFE_SEQUENCE", False)
    transport._sitl_auto_ready_enabled = env_flag("ROS2_UUV_SITL_AUTO_READY", legacy_auto_safe)
    transport._sitl_auto_ready_mode = (
        os.getenv("ROS2_UUV_SITL_AUTO_READY_MODE", os.getenv("ROS2_UUV_SITL_AUTO_MODE", "MANUAL"))
        .strip()
        .upper()
        or "MANUAL"
    )
    transport._sitl_auto_ready_started_wall = -1.0
    transport._sitl_auto_ready_done_wall = -1.0
    transport._sitl_auto_ready_last_log_wall = -1.0
    transport._sitl_auto_ready_last_neutral_wall = -1.0
    transport._sitl_auto_ready_state = "disabled" if not transport._sitl_auto_ready_enabled else "waiting"


__all__ = ["initialize_auto_ready_state"]
