"""Backend scoring and selection policy."""

from __future__ import annotations

from .node_backend_score_fields import MAVROS_SCORE_FIELDS, SIM_BRIDGE_SCORE_FIELDS, score_fields
from .node_backend_select_policy import select_backend


def mavros_score(counts: dict[str, int]) -> int:
    return score_fields(counts, MAVROS_SCORE_FIELDS)


def sim_bridge_score(counts: dict[str, int]) -> int:
    return score_fields(counts, SIM_BRIDGE_SCORE_FIELDS)


__all__ = [
    "mavros_score",
    "select_backend",
    "sim_bridge_score",
]
