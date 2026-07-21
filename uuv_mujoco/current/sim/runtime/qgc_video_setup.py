"""QGC video runtime setup helpers."""

from __future__ import annotations

from typing import Any

from bridge.qgc_video_stream import QgcVideoStreamer
from .qgc_video_components import QgcVideoComponents, disabled_qgc_video_components
from .qgc_video_setup_policy import (
    qgc_video_enabled_message,
    qgc_video_requested,
    qgc_video_size,
    qgc_video_stream_kwargs,
    stereo_left_camera_available,
)


def can_share_ros_camera_renderer(ros_bridge: Any | None, width: int, height: int) -> bool:
    return bool(
        ros_bridge is not None
        and hasattr(ros_bridge, "can_share_camera_renderer")
        and ros_bridge.can_share_camera_renderer("stereo_left", int(width), int(height))
    )


def create_qgc_video_components(
    *,
    args: Any,
    model: Any,
    mujoco_module: Any,
    ros_bridge: Any | None,
    camera_ids: Any,
    streamer_cls: type = QgcVideoStreamer,
) -> QgcVideoComponents:
    if not qgc_video_requested(args):
        return disabled_qgc_video_components()
    if not stereo_left_camera_available(camera_ids):
        print("[qgc_video] stereo_left camera not found; video disabled", flush=True)
        return disabled_qgc_video_components()
    if not streamer_cls.is_available():
        print("[qgc_video] ffmpeg not found; video disabled", flush=True)
        return disabled_qgc_video_components()

    width, height = qgc_video_size(args)
    share_bridge = can_share_ros_camera_renderer(ros_bridge, width, height)
    renderer = None
    if not share_bridge:
        renderer = mujoco_module.Renderer(model, height=height, width=width)
    streamer = streamer_cls(**qgc_video_stream_kwargs(args, width, height))
    print(qgc_video_enabled_message(args), flush=True)
    if share_bridge:
        print("[qgc_video] sharing stereo_left renderer with ROS2 image bridge", flush=True)
    return QgcVideoComponents(streamer=streamer, renderer=renderer, share_bridge=share_bridge)


__all__ = ["QgcVideoComponents", "can_share_ros_camera_renderer", "create_qgc_video_components"]
