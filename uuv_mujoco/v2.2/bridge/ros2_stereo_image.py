"""Stereo camera image publishing helpers for the ROS2 bridge."""

from __future__ import annotations

from array import array
from typing import Any

import mujoco
import numpy as np


CAMERA_NAMES = ("stereo_left", "stereo_right")
DEFAULT_IMAGE_WIDTH = 640
DEFAULT_IMAGE_HEIGHT = 360
DEFAULT_IMAGE_HZ = 4.0


def configure_stereo_image_runtime(
    bridge: Any,
    *,
    publish_images: bool,
    image_width: int,
    image_height: int,
    image_hz: float,
) -> None:
    bridge._stereo_image_enabled = bool(publish_images)
    bridge._stereo_image_width = int(np.clip(int(image_width), 64, 1920))
    bridge._stereo_image_height = int(np.clip(int(image_height), 64, 1080))
    bridge._stereo_image_hz = float(np.clip(float(image_hz), 0.1, 60.0))
    bridge._stereo_image_renderers: dict[str, Any] = {}
    bridge._stereo_image_warned: set[str] = set()
    _ensure_offscreen_buffer(bridge, bridge._stereo_image_width, bridge._stereo_image_height)


def create_stereo_image_publishers(bridge: Any, *, q1: Any) -> None:
    node = bridge.node
    bridge.pub_stereo_left_image = node.create_publisher(bridge.Image, "/stereo/left/image_raw", q1)
    bridge.pub_stereo_right_image = node.create_publisher(bridge.Image, "/stereo/right/image_raw", q1)


def build_stereo_publish_builders(self: Any, data: Any, stamp: Any) -> dict[str, object]:
    return {
        "stereo_left_image": lambda: build_stereo_image_msg(self, "stereo_left", data, stamp),
        "stereo_right_image": lambda: build_stereo_image_msg(self, "stereo_right", data, stamp),
    }


def schedule_stereo_image_jobs(
    self: Any,
    jobs: Any,
    add_rate_limited: Any,
    *,
    builders: dict[str, object],
) -> None:
    if not bool(getattr(self, "_stereo_image_enabled", False)):
        return
    hz = float(getattr(self, "_stereo_image_hz", DEFAULT_IMAGE_HZ))
    add_rate_limited(
        self.pub_stereo_left_image,
        "/stereo/left/image_raw",
        builders["stereo_left_image"],
        hz,
        on_demand=True,
    )
    add_rate_limited(
        self.pub_stereo_right_image,
        "/stereo/right/image_raw",
        builders["stereo_right_image"],
        hz,
        on_demand=True,
    )


def can_share_camera_renderer(self: Any, camera_name: str, width: int, height: int) -> bool:
    if not bool(getattr(self, "_stereo_image_enabled", False)):
        return False
    if str(camera_name) not in CAMERA_NAMES:
        return False
    return (
        int(width) == int(getattr(self, "_stereo_image_width", 0))
        and int(height) == int(getattr(self, "_stereo_image_height", 0))
    )


def render_camera_rgb(self: Any, camera_name: str, data: Any) -> np.ndarray:
    camera = str(camera_name)
    if camera not in CAMERA_NAMES:
        raise ValueError(f"unsupported stereo camera: {camera}")
    renderer = _renderer_for_camera(self, camera)
    renderer.update_scene(data, camera=camera)
    return np.ascontiguousarray(renderer.render(), dtype=np.uint8)


def build_stereo_image_msg(self: Any, camera_name: str, data: Any, stamp: Any) -> Any | None:
    if not bool(getattr(self, "_stereo_image_enabled", False)):
        return None
    try:
        rgb = render_camera_rgb(self, camera_name, data)
    except Exception as exc:
        _warn_once(self, str(camera_name), f"stereo camera render failed: {exc}")
        return None
    msg = self.Image()
    msg.header.stamp = stamp
    msg.header.frame_id = f"{camera_name}_optical"
    msg.height = int(rgb.shape[0])
    msg.width = int(rgb.shape[1])
    msg.encoding = "rgb8"
    msg.is_bigendian = 0
    msg.step = int(rgb.shape[1] * 3)
    msg.data = array("B", rgb.tobytes())
    return msg


def close_stereo_image_renderers(self: Any) -> None:
    renderers = getattr(self, "_stereo_image_renderers", {})
    for renderer in list(renderers.values()):
        try:
            renderer.close()
        except Exception:
            pass
    renderers.clear()


def _renderer_for_camera(self: Any, camera_name: str) -> Any:
    renderers = getattr(self, "_stereo_image_renderers", None)
    if renderers is None:
        renderers = {}
        self._stereo_image_renderers = renderers
    renderer = renderers.get(camera_name)
    if renderer is None:
        renderer = mujoco.Renderer(
            self.model,
            height=int(getattr(self, "_stereo_image_height", DEFAULT_IMAGE_HEIGHT)),
            width=int(getattr(self, "_stereo_image_width", DEFAULT_IMAGE_WIDTH)),
        )
        renderers[camera_name] = renderer
    return renderer


def _ensure_offscreen_buffer(bridge: Any, width: int, height: int) -> None:
    model = getattr(bridge, "model", None)
    if model is None:
        return
    visual_global = getattr(getattr(model, "vis", None), "global_", None)
    if visual_global is None:
        return
    visual_global.offwidth = max(int(getattr(visual_global, "offwidth", 0)), int(width))
    visual_global.offheight = max(int(getattr(visual_global, "offheight", 0)), int(height))


def _warn_once(self: Any, key: str, message: str) -> None:
    warned = getattr(self, "_stereo_image_warned", set())
    if key in warned:
        return
    warned.add(key)
    self._stereo_image_warned = warned
    node = getattr(self, "node", None)
    if node is not None:
        try:
            node.get_logger().warn(message)
            return
        except Exception:
            pass
    print(f"[stereo_image] {message}", flush=True)


__all__ = [
    "build_stereo_image_msg",
    "build_stereo_publish_builders",
    "can_share_camera_renderer",
    "close_stereo_image_renderers",
    "configure_stereo_image_runtime",
    "create_stereo_image_publishers",
    "render_camera_rgb",
    "schedule_stereo_image_jobs",
]
