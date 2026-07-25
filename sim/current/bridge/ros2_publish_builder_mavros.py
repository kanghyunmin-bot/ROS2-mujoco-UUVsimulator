"""MAVROS-compatible ROS message builders for publish jobs."""

from __future__ import annotations

from .ros2_publish_mavros_cache import MavrosPublishBuilderCache
from .ros2_publish_state import RosPublishState


def build_mavros_publish_builders(self, stamp, state: RosPublishState) -> dict[str, object]:
    return MavrosPublishBuilderCache(self, stamp, state).builders()
