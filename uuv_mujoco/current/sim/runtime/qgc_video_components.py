"""QGC video runtime component model."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass
class QgcVideoComponents:
    streamer: Any | None
    renderer: Any | None
    share_bridge: bool


def disabled_qgc_video_components() -> QgcVideoComponents:
    return QgcVideoComponents(streamer=None, renderer=None, share_bridge=False)


__all__ = ["QgcVideoComponents", "disabled_qgc_video_components"]
