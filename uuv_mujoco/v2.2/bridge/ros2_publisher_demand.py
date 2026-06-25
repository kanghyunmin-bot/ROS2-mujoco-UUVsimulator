"""Publisher subscriber-demand cache."""

from __future__ import annotations

from typing import Any

from .ros2_publisher_demand_probe import probe_has_subscribers
from .ros2_publisher_demand_state import DemandState, demand_probe_period


class PublisherDemandCache:
    """Cache subscription demand to avoid rebuilding unused stream messages."""

    def __init__(self, default_probe_period_s: float = 0.25) -> None:
        self._default_probe_period_s = max(float(default_probe_period_s), 0.0)
        self._states: dict[int, DemandState] = {}

    def wants_output(
        self,
        publisher: Any,
        sim_t: float,
        *,
        on_demand: bool = False,
        probe_period_s: float | None = None,
    ) -> bool:
        if publisher is None:
            return False
        if not on_demand:
            return True
        period = demand_probe_period(self._default_probe_period_s, probe_period_s)
        key = id(publisher)
        state = self._states.get(key)
        if state is not None and sim_t + 1.0e-9 < state.next_probe_t:
            return state.has_subscribers
        has_subscribers = self._probe_has_subscribers(publisher)
        self._states[key] = DemandState(next_probe_t=sim_t + period, has_subscribers=has_subscribers)
        return has_subscribers

    @staticmethod
    def _probe_has_subscribers(publisher: Any) -> bool:
        return probe_has_subscribers(publisher)


__all__ = ["PublisherDemandCache"]
