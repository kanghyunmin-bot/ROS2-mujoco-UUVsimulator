"""Lazy DVL ROS message builder cache."""

from __future__ import annotations

from dataclasses import dataclass, field, replace

from .ros2_publish_dvl_factories import (
    DVL_PUBLISH_FACTORIES,
    DvlFactory,
    build_dvl_data_msg,
    build_dvl_position_msg,
)


@dataclass
class DvlPublishBuilderCache:
    bridge: object
    stamp: object
    state: object
    _cache: dict[str, object] = field(default_factory=dict)

    def _cached(self, key: str, factory: DvlFactory):
        if key not in self._cache:
            self._cache[key] = factory(self.bridge, self.stamp, self.state)
        return self._cache[key]

    def builders(self) -> dict[str, object]:
        builders = {
            key: (lambda key=key, factory=factory: self._cached(key, factory))
            for key, factory in DVL_PUBLISH_FACTORIES
        }
        builders["dvl_data_batch"] = lambda: self._delivery_batch(
            "dvl_data_batch",
            build_dvl_data_msg,
            "_dvl_sensor_new_deliveries",
            "dvl_sensor_delivery",
        )
        builders["dvl_position_batch"] = lambda: self._delivery_batch(
            "dvl_position_batch",
            build_dvl_position_msg,
            "_dvl_sensor_new_position_deliveries",
            "dvl_position_delivery",
        )
        return builders

    def _delivery_batch(
        self,
        key: str,
        factory: DvlFactory,
        deliveries_attr: str,
        state_attr: str,
    ) -> tuple[object, ...]:
        if key not in self._cache:
            messages = []
            for delivery in getattr(self.bridge, deliveries_attr, ()):
                state = replace(self.state, **{state_attr: delivery})
                message = factory(self.bridge, self.stamp, state)
                if message is not None:
                    messages.append(message)
            self._cache[key] = tuple(messages)
        return self._cache[key]


__all__ = ["DvlPublishBuilderCache"]
