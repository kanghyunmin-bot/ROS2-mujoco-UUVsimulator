"""ExternalNav runtime watchdog state for SitlTransport."""

from __future__ import annotations

import time

import numpy as np

from bridge.sitl_env import env_flag, env_to_float


def initialize_extnav_runtime_state(transport: object) -> None:
    transport._sitl_extnav_last_send_sim_t = -1.0
    transport._sitl_extnav_last_bootstrap_wall = -1.0
    transport._sitl_extnav_bootstrap_count = 0
    transport._sitl_extnav_last_log_wall = -1.0
    transport._sitl_extnav_send_failed_wall = -1.0
    requested_required = env_flag("ROS2_UUV_REQUIRE_EXTNAV_TX", transport._sitl_extnav_enabled)
    transport._sitl_extnav_required = bool(
        requested_required
        and not bool(getattr(transport, "_sitl_truth_extnav_blocked", False))
    )
    transport._sitl_extnav_rate_fatal = env_flag("ROS2_UUV_EXTNAV_RATE_FATAL", False)
    transport._sitl_extnav_min_tx_hz = float(np.clip(env_to_float("ROS2_UUV_EXTNAV_MIN_TX_HZ", 10.0), 1.0, 50.0))
    transport._sitl_extnav_grace_s = float(np.clip(env_to_float("ROS2_UUV_EXTNAV_TX_GRACE_S", 6.0), 0.5, 30.0))
    transport._sitl_extnav_max_stale_s = float(np.clip(env_to_float("ROS2_UUV_EXTNAV_MAX_STALE_S", 0.5), 0.1, 5.0))
    transport._sitl_extnav_start_wall = time.monotonic()
    transport._sitl_extnav_last_send_wall = -1.0
    transport._sitl_extnav_tx_window_start_wall = transport._sitl_extnav_start_wall
    transport._sitl_extnav_tx_window_count = 0
    transport._sitl_extnav_last_rate_hz = 0.0
    transport._sitl_extnav_fault = ""
    transport._sitl_extnav_rate_warn_wall = -1.0
    transport._sitl_extnav_latest_state = None


__all__ = ["initialize_extnav_runtime_state"]
