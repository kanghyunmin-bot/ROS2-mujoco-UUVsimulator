"""Lazy MAVROS-compatible message builder cache."""

from __future__ import annotations

from dataclasses import dataclass, field, replace
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
        builders = {
            key: (lambda key=key, factory=factory: self._message(key, factory))
            for key, factory in MAVROS_CACHE_FACTORIES
        }
        factories = dict(MAVROS_CACHE_FACTORIES)
        for key in ("mavros_imu", "mavros_imu_raw"):
            batch_key = f"{key}_batch"
            builders[batch_key] = (
                lambda batch_key=batch_key, key=key: self._delivery_batch(
                    batch_key,
                    factories[key],
                    self.state.imu_sensor_deliveries,
                    "imu_sensor_delivery",
                )
            )
        builders["mavros_static_pressure_batch"] = lambda: self._delivery_batch(
            "mavros_static_pressure_batch",
            factories["mavros_static_pressure"],
            self.state.bar30_sensor_deliveries,
            "bar30_sensor_delivery",
        )
        return builders

    def _delivery_batch(self, key, factory, deliveries, state_attr):
        if key not in self._cache:
            messages = []
            for delivery in deliveries:
                state = replace(self.state, **{state_attr: delivery})
                message = factory(self.bridge, self.stamp, state)
                if message is not None:
                    messages.append(message)
            self._cache[key] = tuple(messages)
        return self._cache[key]


__all__ = ["MavrosPublishBuilderCache"]
