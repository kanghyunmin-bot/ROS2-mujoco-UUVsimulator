"""Subscription-count probing for ROS2 publishers."""

from __future__ import annotations

from typing import Any


def probe_has_subscribers(publisher: Any) -> bool:
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


__all__ = ["probe_has_subscribers"]
