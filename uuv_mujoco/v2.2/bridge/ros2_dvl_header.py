"""Header helper for best-effort dvl_msgs compatibility builders."""

from __future__ import annotations

from typing import Any


def stamp_header(msg: Any, stamp: Any, frame_id: str) -> None:
    header = getattr(msg, "header", None)
    if header is None:
        return
    header.stamp = stamp
    if hasattr(header, "frame_id"):
        header.frame_id = frame_id


__all__ = ["stamp_header"]
