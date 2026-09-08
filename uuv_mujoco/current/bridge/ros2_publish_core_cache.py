"""Lazy core ROS message builder cache."""

from __future__ import annotations

from dataclasses import dataclass, field, replace

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
        builders = {
            key: (lambda key=key, factory=factory: self._cached(key, factory))
            for key, factory in CORE_PUBLISH_FACTORIES
        }
        factories = dict(CORE_PUBLISH_FACTORIES)
        builders["imu_batch"] = lambda: self._delivery_batch(
            "imu_batch",
            factories["imu"],
            self.state.imu_sensor_deliveries,
            "imu_sensor_delivery",
        )
        for key in ("depth", "depth_pose", "baro"):
            batch_key = f"{key}_batch"
            builders[batch_key] = (
                lambda batch_key=batch_key, key=key: self._delivery_batch(
                    batch_key,
                    factories[key],
                    self.state.bar30_sensor_deliveries,
                    "bar30_sensor_delivery",
                )
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


__all__ = ["CorePublishBuilderCache"]
