"""Lazy ROS publish job queue."""

from __future__ import annotations

from typing import Any, Callable

from .ros2_publisher_demand import PublisherDemandCache


class PublishQueue:
    """Collect publish jobs lazily and flush them through a provided publish fn."""

    def __init__(self, demand_cache: PublisherDemandCache, sim_t: float) -> None:
        self._demand_cache = demand_cache
        self._sim_t = float(sim_t)
        self._jobs: list[tuple[Any, Any, str]] = []

    def add(
        self,
        publisher: Any,
        label: str,
        builder: Callable[[], Any] | Any,
        *,
        on_demand: bool = False,
        probe_period_s: float | None = None,
    ) -> None:
        if publisher is None:
            return
        if not self._demand_cache.wants_output(
            publisher,
            self._sim_t,
            on_demand=on_demand,
            probe_period_s=probe_period_s,
        ):
            return
        msg = builder() if callable(builder) else builder
        if msg is None:
            return
        self._jobs.append((publisher, msg, label))

    def flush(self, publish_fn: Callable[[Any, Any, str], bool]) -> bool:
        for publisher, msg, label in self._jobs:
            if not publish_fn(publisher, msg, label):
                return False
        return True


__all__ = ["PublishQueue"]
