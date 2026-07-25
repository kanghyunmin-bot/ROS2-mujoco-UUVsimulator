"""Stereo camera image publishing helpers for the ROS2 bridge."""

from __future__ import annotations

import os
import threading
from array import array
from typing import Any

import mujoco
import numpy as np


CAMERA_NAMES = ("stereo_left", "stereo_right")
DEFAULT_IMAGE_WIDTH = 960
DEFAULT_IMAGE_HEIGHT = 540
DEFAULT_IMAGE_HZ = 10.0
DEFAULT_COMPRESSED_JPEG_QUALITY = 75
REAL_CAMERA_RAW_TOPIC = "/camera/camera/color/image_raw"
REAL_CAMERA_COMPRESSED_TOPIC = "/camera/camera/color/image_raw/compressed"
REAL_CAMERA_INFO_TOPIC = "/camera/camera/color/camera_info"
REAL_CAMERA_OPTICAL_FRAME = "camera_color_optical_frame"
_CV2: Any | None = None
_CV2_IMPORT_ATTEMPTED = False


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
    bridge._stereo_image_jpeg_quality = _env_int(
        "ROS2_UUV_CAMERA_JPEG_QUALITY",
        DEFAULT_COMPRESSED_JPEG_QUALITY,
        minimum=40,
        maximum=95,
    )
    bridge._stereo_image_renderers: dict[str, Any] = {}
    bridge._stereo_image_warned: set[str] = set()
    bridge._stereo_image_async_enabled = _env_bool("ROS2_UUV_ASYNC_CAMERA_RENDER", True)
    bridge._stereo_image_async_lock = threading.Lock()
    bridge._stereo_image_async_event = threading.Event()
    bridge._stereo_image_async_stop = threading.Event()
    bridge._stereo_image_async_pending: dict[str, Any] = {}
    # MjData owns a large MuJoCo arena in the competition scene.  Creating and
    # destroying a complete snapshot for every camera frame causes allocator
    # churn and leaves hundreds of megabytes resident.  At most one frame can
    # be pending while one is rendered, so a two-buffer pool per camera is a
    # strict upper bound and preserves the latest-frame/drop-old policy.
    bridge._stereo_image_async_free: dict[str, list[Any]] = {}
    bridge._stereo_image_async_latest: dict[str, np.ndarray] = {}
    bridge._stereo_image_async_latest_jpeg: dict[str, bytes] = {}
    bridge._stereo_image_async_thread = None
    _ensure_offscreen_buffer(bridge, bridge._stereo_image_width, bridge._stereo_image_height)


def create_stereo_image_publishers(bridge: Any, *, q1: Any) -> None:
    node = bridge.node
    bridge.pub_stereo_left_image = node.create_publisher(bridge.Image, "/stereo/left/image_raw", q1)
    bridge.pub_stereo_right_image = node.create_publisher(bridge.Image, "/stereo/right/image_raw", q1)
    bridge.pub_real_camera_raw = node.create_publisher(
        bridge.Image,
        REAL_CAMERA_RAW_TOPIC,
        q1,
    )
    bridge.pub_real_camera_compressed = node.create_publisher(
        bridge.CompressedImage,
        REAL_CAMERA_COMPRESSED_TOPIC,
        q1,
    )
    bridge.pub_real_camera_info = node.create_publisher(
        bridge.CameraInfo,
        REAL_CAMERA_INFO_TOPIC,
        q1,
    )


def build_stereo_publish_builders(self: Any, data: Any, stamp: Any) -> dict[str, object]:
    left_rgb_cache: dict[str, np.ndarray | None] = {}
    left_image_cache: dict[str, Any] = {}

    def render_left_rgb() -> np.ndarray | None:
        if "rgb" not in left_rgb_cache:
            left_rgb_cache["rgb"] = _camera_rgb_for_publish(self, "stereo_left", data)
        return left_rgb_cache["rgb"]

    def build_left_image() -> Any | None:
        if "msg" not in left_image_cache:
            left_image_cache["msg"] = build_stereo_image_msg_from_rgb(
                self,
                "stereo_left",
                render_left_rgb(),
                stamp,
            )
        return left_image_cache["msg"]

    def build_real_camera_raw() -> Any | None:
        return build_stereo_image_msg_from_rgb(
            self,
            "stereo_left",
            render_left_rgb(),
            stamp,
            frame_id=REAL_CAMERA_OPTICAL_FRAME,
        )

    def build_real_camera_compressed() -> Any | None:
        rgb = render_left_rgb()
        if bool(getattr(self, "_stereo_image_async_enabled", False)):
            encoded = _latest_async_camera_jpeg(self, "stereo_left")
            return build_stereo_compressed_image_msg_from_jpeg(
                self, "stereo_left", encoded, stamp
            )
        return build_stereo_compressed_image_msg_from_rgb(
            self, "stereo_left", rgb, stamp
        )

    return {
        "stereo_left_image": build_left_image,
        "stereo_right_image": lambda: build_stereo_image_msg(self, "stereo_right", data, stamp),
        "real_camera_raw": build_real_camera_raw,
        "real_camera_compressed": build_real_camera_compressed,
        "real_camera_info": lambda: build_camera_info_msg(self, stamp),
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
    add_rate_limited(
        self.pub_real_camera_raw,
        REAL_CAMERA_RAW_TOPIC,
        builders["real_camera_raw"],
        hz,
        on_demand=True,
    )
    add_rate_limited(
        self.pub_real_camera_compressed,
        REAL_CAMERA_COMPRESSED_TOPIC,
        builders["real_camera_compressed"],
        hz,
        on_demand=True,
    )
    add_rate_limited(
        self.pub_real_camera_info,
        REAL_CAMERA_INFO_TOPIC,
        builders["real_camera_info"],
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
    return build_stereo_image_msg_from_rgb(
        self,
        camera_name,
        _render_camera_rgb_or_none(self, camera_name, data),
        stamp,
    )


def build_stereo_image_msg_from_rgb(
    self: Any,
    camera_name: str,
    rgb: np.ndarray | None,
    stamp: Any,
    *,
    frame_id: str | None = None,
) -> Any | None:
    if rgb is None:
        return None
    msg = self.Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id or f"{camera_name}_optical"
    msg.height = int(rgb.shape[0])
    msg.width = int(rgb.shape[1])
    msg.encoding = "rgb8"
    msg.is_bigendian = 0
    msg.step = int(rgb.shape[1] * 3)
    msg.data = array("B", rgb.tobytes())
    return msg


def build_stereo_compressed_image_msg(self: Any, camera_name: str, data: Any, stamp: Any) -> Any | None:
    if not bool(getattr(self, "_stereo_image_enabled", False)):
        return None
    return build_stereo_compressed_image_msg_from_rgb(
        self,
        camera_name,
        _render_camera_rgb_or_none(self, camera_name, data),
        stamp,
    )


def build_stereo_compressed_image_msg_from_rgb(
    self: Any,
    camera_name: str,
    rgb: np.ndarray | None,
    stamp: Any,
) -> Any | None:
    if rgb is None:
        return None
    cv2 = _load_cv2()
    if cv2 is None:
        _warn_once(self, "compressed_camera_cv2", "OpenCV is unavailable; compressed camera topic is disabled")
        return None
    try:
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        quality = int(getattr(self, "_stereo_image_jpeg_quality", DEFAULT_COMPRESSED_JPEG_QUALITY))
        ok, encoded = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    except Exception as exc:
        _warn_once(self, f"{camera_name}_compressed", f"compressed camera render failed: {exc}")
        return None
    if not ok:
        _warn_once(self, f"{camera_name}_compressed_encode", "compressed camera JPEG encode failed")
        return None
    return build_stereo_compressed_image_msg_from_jpeg(
        self, camera_name, encoded.tobytes(), stamp
    )


def build_stereo_compressed_image_msg_from_jpeg(
    self: Any,
    camera_name: str,
    encoded: bytes | None,
    stamp: Any,
) -> Any | None:
    if not encoded:
        return None
    msg = self.CompressedImage()
    msg.header.stamp = stamp
    msg.header.frame_id = (
        REAL_CAMERA_OPTICAL_FRAME
        if camera_name == "stereo_left"
        else f"{camera_name}_optical"
    )
    msg.format = "jpeg"
    msg.data = array("B", encoded)
    return msg


def build_camera_info_msg(self: Any, stamp: Any) -> Any:
    width = int(self._stereo_image_width)
    height = int(self._stereo_image_height)
    fovy_deg = 90.0
    camera_id = int(getattr(self, "_cam_left_id", -1))
    if camera_id >= 0:
        try:
            fovy_deg = float(self.model.cam_fovy[camera_id])
        except Exception:
            pass
    fy = 0.5 * height / np.tan(0.5 * np.deg2rad(fovy_deg))
    fx = fy
    cx = 0.5 * (width - 1)
    cy = 0.5 * (height - 1)
    msg = self.CameraInfo()
    msg.header.stamp = stamp
    msg.header.frame_id = REAL_CAMERA_OPTICAL_FRAME
    msg.width = width
    msg.height = height
    msg.distortion_model = "plumb_bob"
    msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0]
    msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return msg


def _render_camera_rgb_or_none(self: Any, camera_name: str, data: Any) -> np.ndarray | None:
    try:
        return render_camera_rgb(self, camera_name, data)
    except Exception as exc:
        _warn_once(self, str(camera_name), f"stereo camera render failed: {exc}")
        return None


def _camera_rgb_for_publish(self: Any, camera_name: str, data: Any) -> np.ndarray | None:
    if not bool(getattr(self, "_stereo_image_async_enabled", False)):
        return _render_camera_rgb_or_none(self, camera_name, data)
    return _submit_async_camera_render(self, camera_name, data)


def _submit_async_camera_render(self: Any, camera_name: str, data: Any) -> np.ndarray | None:
    _start_async_camera_worker(self)
    camera = str(camera_name)
    lock = self._stereo_image_async_lock
    with lock:
        latest = self._stereo_image_async_latest.get(camera)
        # One queued snapshot is enough.  Replacing it requires another full
        # MjData copy on the physics thread and cannot make the renderer catch
        # up any faster.
        if camera in self._stereo_image_async_pending:
            return latest
    snapshot = _acquire_async_camera_snapshot(self, camera)
    try:
        mujoco.mj_copyData(snapshot, self.model, data)
    except Exception as exc:
        _recycle_async_camera_snapshot(self, camera, snapshot)
        _warn_once(self, "async_camera_snapshot", f"async camera snapshot failed: {exc}")
        return None

    with lock:
        # Be defensive if another caller filled the single pending slot while
        # this snapshot was copied.  Do not replace a frame already queued for
        # the renderer; return this buffer to the bounded pool instead.
        if camera in self._stereo_image_async_pending:
            free = self._stereo_image_async_free.setdefault(camera, [])
            if len(free) < 2:
                free.append(snapshot)
            return self._stereo_image_async_latest.get(camera)
        self._stereo_image_async_pending[camera] = snapshot
        latest = self._stereo_image_async_latest.get(camera)
    self._stereo_image_async_event.set()
    return latest


def _start_async_camera_worker(self: Any) -> None:
    thread = getattr(self, "_stereo_image_async_thread", None)
    if thread is not None and thread.is_alive():
        return
    thread = threading.Thread(
        target=_async_camera_worker,
        args=(self,),
        name="uuv-camera-render",
        daemon=True,
    )
    self._stereo_image_async_thread = thread
    thread.start()


def _async_camera_worker(self: Any) -> None:
    event = self._stereo_image_async_event
    stop = self._stereo_image_async_stop
    lock = self._stereo_image_async_lock
    while not stop.is_set():
        event.wait(timeout=0.25)
        event.clear()
        with lock:
            pending = self._stereo_image_async_pending
            self._stereo_image_async_pending = {}
        for camera_name, snapshot in pending.items():
            try:
                if stop.is_set():
                    continue
                rgb = _render_camera_rgb_or_none(self, camera_name, snapshot)
                if rgb is None:
                    continue
                encoded = _encode_camera_jpeg(self, camera_name, rgb)
                with lock:
                    self._stereo_image_async_latest[camera_name] = rgb
                    if encoded is not None:
                        self._stereo_image_async_latest_jpeg[camera_name] = encoded
            finally:
                _recycle_async_camera_snapshot(self, camera_name, snapshot)


def _acquire_async_camera_snapshot(self: Any, camera_name: str) -> Any:
    lock = self._stereo_image_async_lock
    with lock:
        free = self._stereo_image_async_free.setdefault(str(camera_name), [])
        if free:
            return free.pop()
    return mujoco.MjData(self.model)


def _recycle_async_camera_snapshot(self: Any, camera_name: str, snapshot: Any) -> None:
    lock = self._stereo_image_async_lock
    with lock:
        free = self._stereo_image_async_free.setdefault(str(camera_name), [])
        if len(free) < 2:
            free.append(snapshot)


def _latest_async_camera_jpeg(self: Any, camera_name: str) -> bytes | None:
    lock = self._stereo_image_async_lock
    with lock:
        return self._stereo_image_async_latest_jpeg.get(str(camera_name))


def _encode_camera_jpeg(
    self: Any, camera_name: str, rgb: np.ndarray
) -> bytes | None:
    cv2 = _load_cv2()
    if cv2 is None:
        _warn_once(
            self,
            "compressed_camera_cv2",
            "OpenCV is unavailable; compressed camera topic is disabled",
        )
        return None
    try:
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        quality = int(
            getattr(
                self,
                "_stereo_image_jpeg_quality",
                DEFAULT_COMPRESSED_JPEG_QUALITY,
            )
        )
        ok, encoded = cv2.imencode(
            ".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), quality]
        )
    except Exception as exc:
        _warn_once(
            self,
            f"{camera_name}_compressed",
            f"compressed camera render failed: {exc}",
        )
        return None
    if not ok:
        _warn_once(
            self,
            f"{camera_name}_compressed_encode",
            "compressed camera JPEG encode failed",
        )
        return None
    return encoded.tobytes()


def _load_cv2() -> Any | None:
    global _CV2, _CV2_IMPORT_ATTEMPTED
    if _CV2_IMPORT_ATTEMPTED:
        return _CV2
    _CV2_IMPORT_ATTEMPTED = True
    try:
        import cv2  # type: ignore
    except Exception:
        _CV2 = None
    else:
        try:
            cv2.setNumThreads(1)
        except Exception:
            pass
        _CV2 = cv2
    return _CV2


def _env_int(name: str, default: int, *, minimum: int, maximum: int) -> int:
    try:
        value = int(float(str(os.environ.get(name, default)).strip()))
    except (TypeError, ValueError):
        value = int(default)
    return max(int(minimum), min(int(maximum), value))


def _env_bool(name: str, default: bool) -> bool:
    raw = str(os.environ.get(name, "1" if default else "0")).strip().lower()
    return raw in {"1", "true", "yes", "on"}


def close_stereo_image_renderers(self: Any) -> None:
    stop = getattr(self, "_stereo_image_async_stop", None)
    event = getattr(self, "_stereo_image_async_event", None)
    thread = getattr(self, "_stereo_image_async_thread", None)
    if stop is not None:
        stop.set()
    if event is not None:
        event.set()
    if thread is not None and thread.is_alive() and thread is not threading.current_thread():
        thread.join(timeout=2.0)
    lock = getattr(self, "_stereo_image_async_lock", None)
    if lock is not None:
        with lock:
            getattr(self, "_stereo_image_async_pending", {}).clear()
            getattr(self, "_stereo_image_async_free", {}).clear()
            getattr(self, "_stereo_image_async_latest", {}).clear()
            getattr(self, "_stereo_image_async_latest_jpeg", {}).clear()
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
    "REAL_CAMERA_COMPRESSED_TOPIC",
    "REAL_CAMERA_RAW_TOPIC",
    "build_stereo_compressed_image_msg",
    "build_stereo_compressed_image_msg_from_jpeg",
    "build_stereo_compressed_image_msg_from_rgb",
    "build_stereo_image_msg",
    "build_stereo_image_msg_from_rgb",
    "build_stereo_publish_builders",
    "can_share_camera_renderer",
    "close_stereo_image_renderers",
    "configure_stereo_image_runtime",
    "create_stereo_image_publishers",
    "render_camera_rgb",
    "schedule_stereo_image_jobs",
]
