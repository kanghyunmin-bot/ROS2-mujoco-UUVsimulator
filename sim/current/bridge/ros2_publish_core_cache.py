"""Lazy core ROS message builder cache."""

from __future__ import annotations

from dataclasses import dataclass, field

from .ros2_publish_core_factories import CORE_PUBLISH_FACTORIES, CoreFactory


@dataclass
class CorePublishBuilderCache:
    bridge: object
    stamp: object
    state: object
    _cache: dict[str, object] = field(default_factory=dict)

    def _cached(self, key: str, factory: CoreFactory):
        if key not in self._cache:
            self._cache[key] = factory(self.bridge, self.stamp, self.state)
        return self._cache[key]

    def builders(self) -> dict[str, object]:
        return {
            key: (lambda key=key, factory=factory: self._cached(key, factory))
            for key, factory in CORE_PUBLISH_FACTORIES
        }


__all__ = ["CorePublishBuilderCache"]
