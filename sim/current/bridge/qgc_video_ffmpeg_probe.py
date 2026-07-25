"""FFmpeg availability and encoder probing."""

from __future__ import annotations

import platform
import shutil
import subprocess
from functools import lru_cache


def ffmpeg_available() -> bool:
    return shutil.which("ffmpeg") is not None


@lru_cache(maxsize=1)
def available_ffmpeg_encoders() -> set[str]:
    if not ffmpeg_available():
        return set()
    result = subprocess.run(
        ["ffmpeg", "-hide_banner", "-encoders"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )
    return _parse_video_encoders(result.stdout)


def _parse_video_encoders(output: str) -> set[str]:
    encoders: set[str] = set()
    for line in output.splitlines():
        parts = line.split()
        if len(parts) >= 2 and parts[0].startswith("V"):
            encoders.add(parts[1])
    return encoders


def select_h264_encoder() -> str:
    encoders = available_ffmpeg_encoders()
    if platform.system() == "Darwin" and "h264_videotoolbox" in encoders:
        return "h264_videotoolbox"
    return "libx264"


__all__ = ["available_ffmpeg_encoders", "ffmpeg_available", "select_h264_encoder"]
