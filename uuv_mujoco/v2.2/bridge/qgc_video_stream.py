#!/usr/bin/env python3
"""Direct MuJoCo RGB frame -> QGroundControl RTP/H264 UDP streamer."""

from __future__ import annotations

from bridge.qgc_video_stream_config import configure_qgc_video_stream
from bridge.qgc_video_stream_lifecycle import (
    qgc_video_available_encoders,
    qgc_video_build_cmd,
    qgc_video_close,
    qgc_video_is_available,
    qgc_video_select_encoder,
    qgc_video_start,
)
from bridge.qgc_video_stream_write import qgc_video_write_frame


class QgcVideoStreamer:
    def __init__(
        self,
        host: str,
        port: int,
        width: int,
        height: int,
        fps: float,
        bitrate_kbps: int,
    ) -> None:
        configure_qgc_video_stream(
            self,
            host=host,
            port=port,
            width=width,
            height=height,
            fps=fps,
            bitrate_kbps=bitrate_kbps,
        )

    @staticmethod
    def is_available() -> bool:
        return qgc_video_is_available()

    @staticmethod
    def _available_encoders() -> set[str]:
        return qgc_video_available_encoders()

    def _select_encoder(self) -> str:
        return qgc_video_select_encoder()

    def _build_cmd(self) -> list[str]:
        return qgc_video_build_cmd(self)

    def start(self) -> None:
        qgc_video_start(self)

    def write(self, frame_rgb) -> None:
        qgc_video_write_frame(self, frame_rgb)

    def close(self) -> None:
        qgc_video_close(self)


__all__ = ["QgcVideoStreamer"]
