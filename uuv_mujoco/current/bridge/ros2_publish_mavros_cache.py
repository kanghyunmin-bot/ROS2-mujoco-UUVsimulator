"""Lazy MAVROS-compatible message builder cache."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Callable

from .ros2_publish_mavros_cache_factories import MAVROS_CACHE_FACTORIES

if TYPE_CHECKING:
    from .ros2_publish_state import RosPublishState

CacheFactory = Callable[[object, object, "RosPublishState"], object]


@dataclass
class MavrosPublishBuilderCache:
    bridge: object
    stamp: object
    state: RosPublishState
    _cache: dict[str, object] = field(default_factory=dict)

    def _cached(self, key: str, factory):
        if key not in self._cache:
            self._cache[key] = factory()
        return self._cache[key]

    def _message(self, key: str, factory: CacheFactory):
        return self._cached(key, lambda: factory(self.bridge, self.stamp, self.state))

    def builders(self) -> dict[str, object]:
        return {
            key: (lambda key=key, factory=factory: self._message(key, factory))
            for key, factory in MAVROS_CACHE_FACTORIES
        }


__all__ = ["MavrosPublishBuilderCache"]
