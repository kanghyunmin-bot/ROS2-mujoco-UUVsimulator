"""Stereo camera image cache for the web GUI."""

from __future__ import annotations

import io
import os
import time
from dataclasses import dataclass
from typing import Any

import numpy as np
from PIL import Image as PILImage

from .yolo_buoy_detector import create_yolo_buoy_detector


@dataclass(frozen=True)
class StereoCameraFrame:
    data: bytes
    content_type: str
    width: int
    height: int
    encoding: str
    seq: int
    wall_s: float


SIDES = ("left", "right")
SUBSCRIBED_SIDES = ("left",)
TOPICS = {
    "left": "/stereo/left/image_raw",
    "right": "/stereo/right/image_raw",
}
DEFAULT_JPEG_QUALITY = 92
STALE_FRAME_MAX_AGE_S = 2.5


def initialize_stereo_camera_state(owner: Any) -> None:
    owner._stereo_camera_frames: dict[str, StereoCameraFrame] = {}
    owner._stereo_camera_seq: dict[str, int] = {side: 0 for side in SIDES}
    owner._stereo_camera_last_error = ""
    owner._stereo_camera_enabled = True
    owner._stereo_camera_subscriptions: dict[str, Any] = {}
    owner._stereo_camera_image_type = None
    owner._stereo_camera_qos = None
    owner._stereo_camera_buoy_detector = create_yolo_buoy_detector()
    owner._stereo_camera_jpeg_quality = _env_int(
        "UUV_GUI_CAMERA_JPEG_QUALITY",
        DEFAULT_JPEG_QUALITY,
        minimum=50,
        maximum=95,
    )


def initialize_stereo_camera_subscriptions(owner: Any, *, image_type: Any, qos: Any) -> None:
    owner._stereo_camera_image_type = image_type
    owner._stereo_camera_qos = qos
    set_stereo_camera_enabled(owner, True)


def set_stereo_camera_enabled(owner: Any, enabled: bool) -> None:
    owner._stereo_camera_enabled = bool(enabled)
    if owner._stereo_camera_enabled:
        _ensure_stereo_camera_subscriptions(owner)
        return
    _destroy_stereo_camera_subscriptions(owner)
    clear_stereo_camera_frames(owner)


def clear_stereo_camera_frames(owner: Any) -> None:
    with owner._lock:
        owner._stereo_camera_frames.clear()
        owner._stereo_camera_seq = {side: 0 for side in SIDES}


def _ensure_stereo_camera_subscriptions(owner: Any) -> None:
    image_type = getattr(owner, "_stereo_camera_image_type", None)
    qos = getattr(owner, "_stereo_camera_qos", None)
    if image_type is None or qos is None:
        return
    for side in SUBSCRIBED_SIDES:
        if side in owner._stereo_camera_subscriptions:
            continue
        owner._stereo_camera_subscriptions[side] = owner.create_subscription(
            image_type,
            TOPICS[side],
            lambda msg, side=side: owner._on_stereo_camera_image(side, msg),
            qos,
        )


def _destroy_stereo_camera_subscriptions(owner: Any) -> None:
    for side, subscription in list(owner._stereo_camera_subscriptions.items()):
        try:
            owner.destroy_subscription(subscription)
        except Exception as exc:
            owner._stereo_camera_last_error = str(exc)
        finally:
            owner._stereo_camera_subscriptions.pop(side, None)


def on_stereo_camera_image(owner: Any, side: str, msg: Any) -> None:
    if side not in SIDES or not bool(getattr(owner, "_stereo_camera_enabled", True)):
        return
    try:
        rgb = _image_msg_to_rgb_array(msg)
        detector = getattr(owner, "_stereo_camera_buoy_detector", None)
        if detector is not None:
            rgb, _status = detector.process_rgb(rgb)
        jpeg = _rgb_array_to_jpeg(rgb, quality=int(owner._stereo_camera_jpeg_quality))
    except Exception as exc:
        owner._stereo_camera_last_error = str(exc)
        return
    with owner._lock:
        seq = int(owner._stereo_camera_seq.get(side, 0)) + 1
        owner._stereo_camera_seq[side] = seq
        owner._stereo_camera_frames[side] = StereoCameraFrame(
            data=jpeg,
            content_type="image/jpeg",
            width=int(msg.width),
            height=int(msg.height),
            encoding=str(msg.encoding),
            seq=seq,
            wall_s=time.monotonic(),
        )
    owner._touch(f"stereo_{side}")


def stereo_camera_status(owner: Any) -> dict[str, Any]:
    now = time.monotonic()
    with owner._lock:
        frames = dict(owner._stereo_camera_frames)
        error = str(owner._stereo_camera_last_error)
        enabled = bool(owner._stereo_camera_enabled)
        subscribed_sides = sorted(owner._stereo_camera_subscriptions)
        jpeg_quality = int(owner._stereo_camera_jpeg_quality)
    payload: dict[str, Any] = {
        "enabled": enabled,
        "error": error,
        "jpeg_quality": jpeg_quality,
        "subscribed_sides": subscribed_sides,
    }
    detector = getattr(owner, "_stereo_camera_buoy_detector", None)
    if detector is not None:
        payload["detection"] = detector.status_payload()
    for side in SIDES:
        frame = frames.get(side)
        if frame is None or not _frame_fresh(frame, now):
            payload[side] = {
                "available": False,
                "age_s": None if frame is None else max(0.0, now - frame.wall_s),
                "seq": 0,
                "width": 0,
                "height": 0,
                "encoding": "",
            }
            continue
        payload[side] = {
            "available": True,
            "age_s": now - frame.wall_s,
            "seq": frame.seq,
            "width": frame.width,
            "height": frame.height,
            "encoding": frame.encoding,
        }
    return payload


def stereo_camera_frame(owner: Any, side: str) -> StereoCameraFrame | None:
    if not bool(getattr(owner, "_stereo_camera_enabled", True)):
        return None
    with owner._lock:
        frame = owner._stereo_camera_frames.get(side)
    if frame is None or not _frame_fresh(frame, time.monotonic()):
        return None
    return frame


def _frame_fresh(frame: StereoCameraFrame, now: float) -> bool:
    return max(0.0, now - frame.wall_s) <= STALE_FRAME_MAX_AGE_S


def _image_msg_to_rgb_array(msg: Any) -> np.ndarray:
    width = int(msg.width)
    height = int(msg.height)
    encoding = str(msg.encoding).lower()
    if width <= 0 or height <= 0:
        raise ValueError("empty stereo image")
    raw = bytes(msg.data)
    if encoding == "rgb8":
        image = PILImage.frombytes(
            "RGB",
            (width, height),
            _contiguous_rows(raw, height, int(msg.step), width * 3),
        )
    elif encoding == "bgr8":
        image = PILImage.frombytes(
            "RGB",
            (width, height),
            _contiguous_rows(raw, height, int(msg.step), width * 3),
            "raw",
            "BGR",
        )
    elif encoding == "mono8":
        image = PILImage.frombytes(
            "L",
            (width, height),
            _contiguous_rows(raw, height, int(msg.step), width),
        )
    elif encoding in {"rgba8", "bgra8"}:
        raw_mode = "RGBA" if encoding == "rgba8" else "BGRA"
        image = PILImage.frombytes(
            "RGBA",
            (width, height),
            _contiguous_rows(raw, height, int(msg.step), width * 4),
            "raw",
            raw_mode,
        ).convert("RGB")
    else:
        raise ValueError(f"unsupported stereo image encoding: {msg.encoding}")
    return np.ascontiguousarray(np.asarray(image.convert("RGB")), dtype=np.uint8)


def _rgb_array_to_jpeg(rgb: np.ndarray, *, quality: int) -> bytes:
    image = PILImage.fromarray(np.ascontiguousarray(rgb, dtype=np.uint8), mode="RGB")
    out = io.BytesIO()
    image.save(out, format="JPEG", quality=int(quality), subsampling=0, optimize=False)
    return out.getvalue()


def _contiguous_rows(raw: bytes, height: int, step: int, wanted_step: int) -> bytes:
    if step == wanted_step:
        return raw[: height * wanted_step]
    if step < wanted_step:
        raise ValueError("image step is smaller than encoded row width")
    return b"".join(raw[row * step : row * step + wanted_step] for row in range(height))


def _env_int(name: str, default: int, *, minimum: int, maximum: int) -> int:
    try:
        value = int(str(os.environ.get(name, default)).strip())
    except (TypeError, ValueError):
        value = int(default)
    return max(int(minimum), min(int(maximum), value))


__all__ = [
    "StereoCameraFrame",
    "clear_stereo_camera_frames",
    "initialize_stereo_camera_state",
    "initialize_stereo_camera_subscriptions",
    "on_stereo_camera_image",
    "set_stereo_camera_enabled",
    "stereo_camera_frame",
    "stereo_camera_status",
]
