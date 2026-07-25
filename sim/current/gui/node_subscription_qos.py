"""QoS profile builders for GUI node subscriptions."""

from __future__ import annotations

from .runtime import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def build_state_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


def build_best_effort_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=50,
    )


__all__ = ["build_best_effort_qos", "build_state_qos"]
