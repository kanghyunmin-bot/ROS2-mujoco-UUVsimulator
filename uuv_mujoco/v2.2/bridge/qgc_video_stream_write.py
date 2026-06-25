"""Frame write helper for QGC RTP/H264 video streaming."""

from __future__ import annotations

from bridge.qgc_video_stream_lifecycle import qgc_video_start


def qgc_video_write_frame(owner, frame_rgb) -> None:
    if owner._proc is None:
        qgc_video_start(owner)
    if owner._proc is None or owner._proc.stdin is None:
        return
    if owner._proc.poll() is not None:
        raise RuntimeError("ffmpeg exited")
    owner._proc.stdin.write(frame_rgb.tobytes())


__all__ = ["qgc_video_write_frame"]
