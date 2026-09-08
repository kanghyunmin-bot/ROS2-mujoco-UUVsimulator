"""Ping360 LaserScan-style ROS2 message builder."""

from __future__ import annotations

from typing import Any

import numpy as np

from .ping360_types import PING360_GRADS_PER_REV, Ping360Config, Ping360Sample


def build_ping360_scan_msg(
    laser_scan_type: type,
    stamp: Any,
    sample: Ping360Sample,
    config: Ping360Config,
) -> Any:
    msg = laser_scan_type()
    msg.header.stamp = stamp
    msg.header.frame_id = config.frame_id
    angle_increment = 2.0 * np.pi / float(PING360_GRADS_PER_REV)
    msg.angle_min = 0.0
    msg.angle_max = 2.0 * np.pi - angle_increment
    msg.angle_increment = angle_increment
    # This topic is a fixed-angle snapshot assembled from the rolling Ping360
    # history, not one monotonically ordered laser sweep.  A non-zero value
    # makes RViz add a second full-scan period to ``header.stamp`` and request
    # transforms from the future.  Consumers that need the native per-profile
    # timing must use /ping360/echo and /ping360/status.
    msg.time_increment = 0.0
    msg.scan_time = float(sample.settings.scan_period_s)
    msg.range_min = float(config.min_range_m)
    msg.range_max = float(sample.settings.effective_range_m)
    ranges = np.asarray(sample.ranges_m, dtype=np.float32)
    ranges = np.where(np.isfinite(ranges), ranges, np.inf).astype(np.float32, copy=False)
    msg.ranges = ranges.tolist()
    msg.intensities = np.asarray(sample.intensities, dtype=np.float32).tolist()
    return msg


__all__ = ["build_ping360_scan_msg"]
