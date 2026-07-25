"""QGC video setup policy helpers."""

from __future__ import annotations

from typing import Any


def qgc_video_requested(args: Any) -> bool:
    return bool(args.qgc_video)


def stereo_left_camera_available(camera_ids: Any) -> bool:
    return "stereo_left" in camera_ids


def qgc_video_size(args: Any) -> tuple[int, int]:
    return int(args.qgc_video_width), int(args.qgc_video_height)


def qgc_video_stream_kwargs(args: Any, width: int, height: int) -> dict[str, int | float | str]:
    return {
        "host": args.qgc_video_host,
        "port": int(args.qgc_video_port),
        "width": width,
        "height": height,
        "fps": float(args.qgc_video_fps),
        "bitrate_kbps": int(args.qgc_video_bitrate_kbps),
    }


def qgc_video_enabled_message(args: Any) -> str:
    return (
        "[qgc_video] direct UDP stream enabled: stereo_left -> "
        f"rtp://{args.qgc_video_host}:{args.qgc_video_port} "
        f"({args.qgc_video_width}x{args.qgc_video_height}@{args.qgc_video_fps:.1f}fps)"
    )


__all__ = [
    "qgc_video_enabled_message",
    "qgc_video_requested",
    "qgc_video_size",
    "qgc_video_stream_kwargs",
    "stereo_left_camera_available",
]
