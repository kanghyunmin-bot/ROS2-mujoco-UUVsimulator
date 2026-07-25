"""Header helper for MAVROS state messages."""

from __future__ import annotations


def fill_state_header(msg, stamp) -> None:
    header = getattr(msg, "header", None)
    if header is None:
        return
    header.stamp = stamp
    if hasattr(header, "frame_id"):
        header.frame_id = "base_link"


__all__ = ["fill_state_header"]
