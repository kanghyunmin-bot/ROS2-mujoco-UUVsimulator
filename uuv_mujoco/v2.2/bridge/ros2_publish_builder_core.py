"""Core ROS message builders for publish jobs."""

from __future__ import annotations

from typing import TYPE_CHECKING

from .ros2_publish_core_cache import CorePublishBuilderCache

if TYPE_CHECKING:
    from .ros2_publish_state import RosPublishState


def build_core_publish_builders(self, stamp, state: "RosPublishState") -> dict[str, object]:
    return CorePublishBuilderCache(self, stamp, state).builders()


__all__ = ["CorePublishBuilderCache", "build_core_publish_builders"]
