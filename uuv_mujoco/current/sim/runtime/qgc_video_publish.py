"""QGC video frame publish helper."""

from __future__ import annotations

import numpy as np

from .qgc_video_frame import render_qgc_video_frame, should_publish_qgc_video_frame
from .qgc_video_lifecycle import close_qgc_video_streamer


def publish_qgc_video_frame(runtime, data, ros_bridge) -> None:
    if runtime.streamer is None:
        return
    if not should_publish_qgc_video_frame(data.time, runtime.next_t):
        return
    runtime.next_t = data.time + runtime.dt
    try:
        rgb, runtime.renderer, runtime.share_bridge = render_qgc_video_frame(
            args=runtime.args,
            model=runtime.model,
            mujoco_module=runtime.mujoco,
            renderer=runtime.renderer,
            share_bridge=runtime.share_bridge,
            data=data,
            ros_bridge=ros_bridge,
        )
        if rgb is not None:
            runtime.streamer.write(np.ascontiguousarray(rgb))
    except Exception as exc:
        print(f"[qgc_video] stream failed, disabling video: {exc}", flush=True)
        close_qgc_video_streamer(runtime.streamer)
        runtime.streamer = None


__all__ = ["publish_qgc_video_frame"]
