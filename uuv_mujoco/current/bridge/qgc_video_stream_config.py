"""State normalization for QGC RTP/H264 video streaming."""

from __future__ import annotations

import subprocess
from typing import Optional


def configure_qgc_video_stream(
    owner,
    *,
    host: str,
    port: int,
    width: int,
    height: int,
    fps: float,
    bitrate_kbps: int,
) -> None:
    owner.host = str(host)
    owner.port = int(port)
    owner.width = int(width)
    owner.height = int(height)
    owner.fps = float(max(1.0, fps))
    owner.bitrate_kbps = int(max(300, bitrate_kbps))
    owner._proc: Optional[subprocess.Popen] = None


__all__ = ["configure_qgc_video_stream"]
