"""Backend graph probing and RC layout helpers for UuvGuiNode."""

from __future__ import annotations

import time
from typing import Any

from .config import BACKEND_AUTO
from .node_backend_counts import (
    probe_graph_counts,
    safe_count_publishers,
    safe_count_subscribers,
    service_ready,
)
from .node_backend_layout import active_layout, backend_label, effective_backend, rc_mapping_summary
from .node_backend_selection import (
    mavros_score as _mavros_score,
    select_backend as _select_backend,
    sim_bridge_score as _sim_bridge_score,
)
from .node_snapshot_runtime import push_event


def probe_backend(owner: Any, *, force: bool = False) -> None:
    now = time.monotonic()
    if not force and owner._last_graph_probe_wall >= 0.0 and (now - owner._last_graph_probe_wall) < 3.0:
        return
    owner._last_graph_probe_wall = now

    counts = probe_graph_counts(owner)
    owner._rc_override_subscribers = counts["rc_override_subscribers"]
    owner._manual_control_subscribers = counts["manual_control_subscribers"]
    owner._vehicle_info_supported = counts["vehicle_info_services"] > 0
    if owner._backend_preference != BACKEND_AUTO:
        return

    mavros_score = _mavros_score(counts)
    sim_score = _sim_bridge_score(counts)
    next_backend = _select_backend(owner._backend_detected, mavros_score, sim_score, counts)
    if next_backend != owner._backend_detected:
        owner._backend_detected = next_backend
        push_event(owner, f"backend -> {backend_label(owner)} ({rc_mapping_summary(owner)})")


__all__ = [
    "active_layout",
    "backend_label",
    "effective_backend",
    "probe_backend",
    "rc_mapping_summary",
    "safe_count_publishers",
    "safe_count_subscribers",
    "service_ready",
]
