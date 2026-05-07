"""Runtime publish helpers for the lightweight MuJoCo ROS2 bridge."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable


@dataclass
class _DemandState:
    next_probe_t: float
    has_subscribers: bool


class PublisherDemandCache:
    """Cache subscription demand to avoid rebuilding unused stream messages."""

    def __init__(self, default_probe_period_s: float = 0.25) -> None:
        self._default_probe_period_s = max(float(default_probe_period_s), 0.0)
        self._states: dict[int, _DemandState] = {}

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
        period = self._default_probe_period_s if probe_period_s is None else max(float(probe_period_s), 0.0)
        key = id(publisher)
        state = self._states.get(key)
        if state is not None and sim_t + 1.0e-9 < state.next_probe_t:
            return state.has_subscribers
        has_subscribers = self._probe_has_subscribers(publisher)
        self._states[key] = _DemandState(next_probe_t=sim_t + period, has_subscribers=has_subscribers)
        return has_subscribers

    @staticmethod
    def _probe_has_subscribers(publisher: Any) -> bool:
        try:
            count = int(publisher.get_subscription_count())
        except Exception:
            # If we cannot query demand reliably, keep publishing to preserve behavior.
            return True
        intra_process_getter = getattr(publisher, "get_intra_process_subscription_count", None)
        if callable(intra_process_getter):
            try:
                count += int(intra_process_getter())
            except Exception:
                pass
        return count > 0


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


class StaticContextPublisher:
    """Manage low-rate static bridge outputs outside the main hot path."""

    def __init__(
        self,
        *,
        tf_static_pub: Any,
        robot_description_pub: Any,
        string_factory: Callable[[], Any],
        build_tf_message: Callable[[Any, Any], Any],
        safe_publish: Callable[[Any, Any, str], bool],
        static_tf_specs: Any,
        robot_description_text: str,
        robot_description_pub_period_s: float,
    ) -> None:
        self._tf_static_pub = tf_static_pub
        self._robot_description_pub = robot_description_pub
        self._string_factory = string_factory
        self._build_tf_message = build_tf_message
        self._safe_publish = safe_publish
        self._static_tf_specs = static_tf_specs
        self._robot_description_text = robot_description_text
        self._robot_description_pub_period_s = float(robot_description_pub_period_s)
        self._robot_description_next_t = 0.0
        self._static_tf_published = False

    def publish(self, stamp: Any, sim_t: float) -> bool:
        if not self._static_tf_published and self._tf_static_pub is not None:
            static_msg = self._build_tf_message(stamp, self._static_tf_specs)
            if static_msg is not None and not self._safe_publish(self._tf_static_pub, static_msg, "/tf_static"):
                return False
            self._static_tf_published = True
        if (
            self._robot_description_pub is not None
            and self._robot_description_text
            and sim_t + 1.0e-9 >= self._robot_description_next_t
        ):
            robot_msg = self._string_factory()
            robot_msg.data = self._robot_description_text
            if not self._safe_publish(self._robot_description_pub, robot_msg, "/robot_description"):
                return False
            self._robot_description_next_t = sim_t + self._robot_description_pub_period_s
        return True
