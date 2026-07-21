"""Compatibility surface for QGC RTP/H264 FFmpeg helpers."""

from __future__ import annotations

from bridge.qgc_video_ffmpeg_cmd import build_qgc_video_ffmpeg_cmd
from bridge.qgc_video_ffmpeg_probe import available_ffmpeg_encoders, ffmpeg_available, select_h264_encoder
from bridge.qgc_video_ffmpeg_process import close_ffmpeg_process, open_ffmpeg_process

__all__ = [
    "available_ffmpeg_encoders",
    "build_qgc_video_ffmpeg_cmd",
    "close_ffmpeg_process",
    "ffmpeg_available",
    "open_ffmpeg_process",
    "select_h264_encoder",
]
