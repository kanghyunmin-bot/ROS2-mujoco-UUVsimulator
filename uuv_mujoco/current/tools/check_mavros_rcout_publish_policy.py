#!/usr/bin/env python3
"""Regression checks for MAVROS RC output publication policy defaults."""

from __future__ import annotations

import os
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from bridge.ros2_bridge_config_mavros_rcout import configure_mavros_rcout_policy  # noqa: E402


class FakeBridge:
    pass


def _configured(*, publish_mode: str | None = None, stamp_source: str | None = None) -> FakeBridge:
    saved_publish_mode = os.environ.get("ROS2_UUV_RCOUT_PUBLISH_MODE")
    saved_stamp_source = os.environ.get("ROS2_UUV_RCOUT_HEADER_STAMP_SOURCE")
    try:
        if publish_mode is None:
            os.environ.pop("ROS2_UUV_RCOUT_PUBLISH_MODE", None)
        else:
            os.environ["ROS2_UUV_RCOUT_PUBLISH_MODE"] = publish_mode
        if stamp_source is None:
            os.environ.pop("ROS2_UUV_RCOUT_HEADER_STAMP_SOURCE", None)
        else:
            os.environ["ROS2_UUV_RCOUT_HEADER_STAMP_SOURCE"] = stamp_source
        bridge = FakeBridge()
        configure_mavros_rcout_policy(bridge)
        return bridge
    finally:
        if saved_publish_mode is None:
            os.environ.pop("ROS2_UUV_RCOUT_PUBLISH_MODE", None)
        else:
            os.environ["ROS2_UUV_RCOUT_PUBLISH_MODE"] = saved_publish_mode
        if saved_stamp_source is None:
            os.environ.pop("ROS2_UUV_RCOUT_HEADER_STAMP_SOURCE", None)
        else:
            os.environ["ROS2_UUV_RCOUT_HEADER_STAMP_SOURCE"] = saved_stamp_source


def main() -> int:
    default_bridge = _configured()
    assert default_bridge._mavros_rc_out_publish_mode == "rate_limited", default_bridge.__dict__
    assert default_bridge._mavros_rc_out_header_stamp_source == "ros_time", default_bridge.__dict__

    invalid_bridge = _configured(publish_mode="invalid")
    assert invalid_bridge._mavros_rc_out_publish_mode == "rate_limited", invalid_bridge.__dict__

    event_bridge = _configured(publish_mode="event")
    assert event_bridge._mavros_rc_out_publish_mode == "event", event_bridge.__dict__

    rate_limited_bridge = _configured(publish_mode="RATE_LIMITED", stamp_source="WALL_TIME")
    assert rate_limited_bridge._mavros_rc_out_publish_mode == "rate_limited", rate_limited_bridge.__dict__
    assert rate_limited_bridge._mavros_rc_out_header_stamp_source == "wall_time", rate_limited_bridge.__dict__

    print("mavros_rcout_publish_policy=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
