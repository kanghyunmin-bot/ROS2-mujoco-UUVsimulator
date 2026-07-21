"""QGC video frame rendering helpers."""

from __future__ import annotations

from typing import Any


def should_publish_qgc_video_frame(data_time: float, next_t: float) -> bool:
    return float(data_time) + 1e-9 >= float(next_t)


def render_qgc_video_frame(
    *,
    args: Any,
    model: Any,
    mujoco_module: Any,
    renderer: Any | None,
    share_bridge: bool,
    data: Any,
    ros_bridge: Any | None,
) -> tuple[Any | None, Any | None, bool]:
    rgb = None
    if share_bridge and ros_bridge is not None:
        try:
            rgb = ros_bridge.render_camera_rgb("stereo_left", data)
        except Exception as exc:
            print(
                f"[qgc_video] shared renderer failed, falling back to dedicated renderer: {exc}",
                flush=True,
            )
            share_bridge = False
    if rgb is None:
        if renderer is None:
            renderer = mujoco_module.Renderer(
                model,
                height=int(args.qgc_video_height),
                width=int(args.qgc_video_width),
            )
        renderer.update_scene(data, camera="stereo_left")
        rgb = renderer.render()
    return rgb, renderer, share_bridge


__all__ = ["render_qgc_video_frame", "should_publish_qgc_video_frame"]
