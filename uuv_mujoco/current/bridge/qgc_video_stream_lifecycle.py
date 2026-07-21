"""QGC RTP/H264 streamer lifecycle helpers."""

from __future__ import annotations

from bridge.qgc_video_ffmpeg import (
    available_ffmpeg_encoders,
    build_qgc_video_ffmpeg_cmd,
    close_ffmpeg_process,
    ffmpeg_available,
    open_ffmpeg_process,
    select_h264_encoder,
)


def qgc_video_is_available() -> bool:
    return ffmpeg_available()


def qgc_video_available_encoders() -> set[str]:
    return available_ffmpeg_encoders()


def qgc_video_select_encoder() -> str:
    return select_h264_encoder()


def qgc_video_build_cmd(owner) -> list[str]:
    return build_qgc_video_ffmpeg_cmd(
        host=owner.host,
        port=owner.port,
        width=owner.width,
        height=owner.height,
        fps=owner.fps,
        bitrate_kbps=owner.bitrate_kbps,
    )


def qgc_video_start(owner) -> None:
    if owner._proc is not None:
        return
    if not qgc_video_is_available():
        raise RuntimeError("ffmpeg not found")
    owner._proc = open_ffmpeg_process(qgc_video_build_cmd(owner))


def qgc_video_close(owner) -> None:
    if owner._proc is None:
        return
    close_ffmpeg_process(owner._proc)
    owner._proc = None


__all__ = [
    "qgc_video_available_encoders",
    "qgc_video_build_cmd",
    "qgc_video_close",
    "qgc_video_is_available",
    "qgc_video_select_encoder",
    "qgc_video_start",
]
