"""Hydrophone and synchronization ROS topic scheduling."""

from __future__ import annotations

import numpy as np

from .sitl_env import env_to_float


def schedule_hydrophone_jobs(
    bridge,
    jobs,
    add_rate_limited,
    *,
    builders: dict[str, object],
) -> None:
    cfg = bridge._hydrophone_config
    if not cfg.enabled:
        return
    sync_hz = float(
        np.clip(
            env_to_float(
                "ROS2_UUV_HYDROPHONE_SYNC_HZ",
                max(cfg.publish_hz, cfg.status_hz, 50.0),
            ),
            1.0,
            100.0,
        )
    )
    if not bridge._real_pkg_compat:
        add_rate_limited(
            bridge.pub_depth_pose,
            "/depth/pose:hydrophone_sync",
            builders["depth_pose"],
            sync_hz,
        )
        if bool(
            getattr(
                bridge,
                "_unsafe_legacy_ground_truth_odometry_filtered",
                False,
            )
        ):
            add_rate_limited(
                bridge.pub_odometry_filtered,
                "/odometry/filtered:hydrophone_sync",
                builders["sim_odom"],
                sync_hz,
            )
    add_rate_limited(
        bridge.pub_hydrophone_status,
        "/mujoco/hydrophone/status",
        builders["hydrophone_status"],
        cfg.status_hz,
        on_demand=True,
    )
    add_rate_limited(
        bridge.pub_hydrophone_direction,
        "/mujoco/hydrophone/direction",
        builders["hydrophone_direction"],
        cfg.status_hz,
        on_demand=True,
    )
    add_rate_limited(
        bridge.pub_hydrophone_audio_info,
        "/audio_info",
        builders["hydrophone_audio_info"],
        cfg.info_hz,
        on_demand=True,
    )
    add_rate_limited(
        bridge.pub_hydrophone_audio,
        "/audio",
        builders["hydrophone_audio"],
        cfg.publish_hz,
        on_demand=True,
    )


__all__ = ["schedule_hydrophone_jobs"]
