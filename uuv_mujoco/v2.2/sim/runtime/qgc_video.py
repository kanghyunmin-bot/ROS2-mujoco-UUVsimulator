"""QGroundControl video streaming runtime wrapper."""

from __future__ import annotations

from .qgc_video_lifecycle import close_qgc_video_renderer, close_qgc_video_streamer
from .qgc_video_publish import publish_qgc_video_frame
from .qgc_video_setup import create_qgc_video_components


class QgcVideoRuntime:
    def __init__(
        self,
        *,
        args,
        model,
        mujoco_module,
        streamer,
        renderer,
        share_bridge: bool,
    ) -> None:
        self.args = args
        self.model = model
        self.mujoco = mujoco_module
        self.streamer = streamer
        self.renderer = renderer
        self.share_bridge = bool(share_bridge)
        self.dt = 1.0 / max(float(args.qgc_video_fps), 1.0)
        self.next_t = 0.0

    @classmethod
    def create(cls, *, args, model, mujoco_module, ros_bridge, camera_ids) -> "QgcVideoRuntime":
        components = create_qgc_video_components(
            args=args,
            model=model,
            mujoco_module=mujoco_module,
            ros_bridge=ros_bridge,
            camera_ids=camera_ids,
        )
        return cls(
            args=args,
            model=model,
            mujoco_module=mujoco_module,
            streamer=components.streamer,
            renderer=components.renderer,
            share_bridge=components.share_bridge,
        )

    def publish(self, data, ros_bridge) -> None:
        publish_qgc_video_frame(self, data, ros_bridge)

    def close(self) -> None:
        close_qgc_video_streamer(self.streamer)
        self.streamer = None
        close_qgc_video_renderer(self.renderer)
        self.renderer = None


__all__ = ["QgcVideoRuntime"]
