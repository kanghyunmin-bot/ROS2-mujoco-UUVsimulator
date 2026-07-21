"""Backend selection tie-break policy."""

from __future__ import annotations

from .config import BACKEND_MAVROS, BACKEND_NONE, BACKEND_SIM_BRIDGE, DEFAULT_AUTO_BACKEND
from .node_backend_score_fields import has_mavros_tie_breaker


def select_backend(current_backend: str, mavros: int, sim_bridge: int, counts: dict[str, int]) -> str:
    if _external_mavros_ready(counts):
        return BACKEND_MAVROS
    if mavros > 0 and (
        mavros > sim_bridge
        or (
            mavros == sim_bridge
            and has_mavros_tie_breaker(counts)
        )
    ):
        return BACKEND_MAVROS
    if sim_bridge > 0:
        return BACKEND_SIM_BRIDGE
    return BACKEND_NONE if current_backend == BACKEND_NONE else DEFAULT_AUTO_BACKEND


def _external_mavros_ready(counts: dict[str, int]) -> bool:
    return (
        counts["vehicle_info_services"] > 0
        and counts["arm_services"] > 0
        and counts["mode_services"] > 0
    )


__all__ = ["select_backend"]
