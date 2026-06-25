"""DVL ROS message builders for publish jobs."""

from __future__ import annotations

from typing import TYPE_CHECKING

from .ros2_publish_dvl_cache import DvlPublishBuilderCache

if TYPE_CHECKING:
    from .ros2_publish_state import RosPublishState


def build_dvl_publish_builders(self, stamp, state: RosPublishState) -> dict[str, object]:
    return DvlPublishBuilderCache(self, stamp, state).builders()


__all__ = ["DvlPublishBuilderCache", "build_dvl_publish_builders"]
