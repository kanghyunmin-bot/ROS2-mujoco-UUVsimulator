"""QGC video runtime cleanup helpers."""

from __future__ import annotations

from typing import Any


def close_qgc_video_renderer(renderer: Any | None) -> None:
    if renderer is None:
        return
    try:
        renderer.close()
    except Exception:
        pass


def close_qgc_video_streamer(streamer: Any | None) -> None:
    if streamer is not None:
        streamer.close()


__all__ = ["close_qgc_video_renderer", "close_qgc_video_streamer"]
