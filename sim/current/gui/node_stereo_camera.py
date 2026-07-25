"""Stereo camera image cache for the web GUI."""

from __future__ import annotations

import io
import json
import os
import time
from dataclasses import dataclass
from typing import Any

import numpy as np
from PIL import Image as PILImage

from .runtime import String
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
VISION_SOURCE = "vision"
DEFAULT_TOPICS = {
    "left": "/camera/camera/color/image_raw/compressed",
    "right": "/stereo/right/image_raw",
    VISION_SOURCE: "/vision/buoy/image_annotated/compressed",
}
DEFAULT_JPEG_QUALITY = 82
STALE_FRAME_MAX_AGE_S = 2.5
VISION_STATUS_MAX_AGE_S = 2.5
VISION_STATUS_TOPIC = "/vision/buoy/status"
YOLO_DETECTION_TOPIC = "/uuv_mujoco/yolo_buoy_detections"


def initialize_stereo_camera_state(owner: Any) -> None:
    owner._stereo_camera_frames: dict[str, StereoCameraFrame] = {}
    owner._stereo_camera_seq: dict[str, int] = {
        side: 0 for side in (*SIDES, VISION_SOURCE)
    }
    owner._stereo_camera_last_error = ""
    owner._stereo_camera_enabled = True
    owner._stereo_camera_subscriptions: dict[str, Any] = {}
    owner._stereo_camera_image_type = None
    owner._stereo_camera_compressed_image_type = None
    owner._stereo_camera_qos = None
    requested_mode = str(os.environ.get("UUV_GUI_CAMERA_DISPLAY_MODE", "vision")).strip().lower()
    owner._stereo_camera_display_mode = "vision" if requested_mode == "vision" else "raw"
    owner._stereo_camera_vision_status: dict[str, Any] = {}
    owner._stereo_camera_vision_status_wall = 0.0
    owner._stereo_camera_buoy_detector = create_yolo_buoy_detector()
    owner._stereo_camera_fast_display = _env_flag("UUV_GUI_CAMERA_FAST_DISPLAY", True)
    owner._stereo_camera_last_process_wall = 0.0
    owner._stereo_camera_process_period_s = 1.0 / _env_float(
        "UUV_GUI_CAMERA_PROCESS_HZ",
        1.5,
        minimum=0.1,
        maximum=30.0,
    )
    owner._stereo_camera_jpeg_quality = _env_int(
        "UUV_GUI_CAMERA_JPEG_QUALITY",
        DEFAULT_JPEG_QUALITY,
        minimum=50,
        maximum=95,
    )


def initialize_stereo_camera_subscriptions(
    owner: Any,
    *,
    image_type: Any,
    compressed_image_type: Any | None = None,
    qos: Any,
) -> None:
    owner._stereo_camera_image_type = image_type
    owner._stereo_camera_compressed_image_type = compressed_image_type
    owner._stereo_camera_qos = qos
    set_stereo_camera_enabled(owner, True)


def set_stereo_camera_enabled(owner: Any, enabled: bool) -> None:
    owner._stereo_camera_enabled = bool(enabled)
    if owner._stereo_camera_enabled:
        _ensure_stereo_camera_subscriptions(owner)
        return
    _destroy_stereo_camera_subscriptions(owner)
    clear_stereo_camera_frames(owner)


def set_stereo_camera_display_mode(owner: Any, mode: str) -> None:
    normalized = str(mode).strip().lower()
    if normalized not in {"raw", "vision"}:
        raise ValueError(f"unsupported camera display mode: {mode}")
    with owner._lock:
        owner._stereo_camera_display_mode = normalized


def clear_stereo_camera_frames(owner: Any) -> None:
    with owner._lock:
        owner._stereo_camera_frames.clear()
        owner._stereo_camera_seq = {side: 0 for side in (*SIDES, VISION_SOURCE)}


def _ensure_stereo_camera_subscriptions(owner: Any) -> None:
    image_type = getattr(owner, "_stereo_camera_image_type", None)
    qos = getattr(owner, "_stereo_camera_qos", None)
    if image_type is None or qos is None:
        return
    for side in SUBSCRIBED_SIDES:
        if side in owner._stereo_camera_subscriptions:
            continue
        topic = stereo_camera_topic(side)
        msg_type = _message_type_for_topic(owner, topic)
        owner._stereo_camera_subscriptions[side] = owner.create_subscription(
            msg_type,
            topic,
            lambda msg, side=side: owner._on_stereo_camera_image(side, msg),
            qos,
        )
        owner._push_event(f"Camera feed subscribed: {topic}")
    if VISION_SOURCE not in owner._stereo_camera_subscriptions:
        topic = stereo_camera_topic(VISION_SOURCE)
        msg_type = _message_type_for_topic(owner, topic)
        owner._stereo_camera_subscriptions[VISION_SOURCE] = owner.create_subscription(
            msg_type,
            topic,
            lambda msg: _on_compressed_stereo_camera_image(
                owner, VISION_SOURCE, msg, process_detector=False
            ),
            qos,
        )
        owner._push_event(f"Vision camera feed subscribed: {topic}")
    if "vision_status" not in owner._stereo_camera_subscriptions:
        owner._stereo_camera_subscriptions["vision_status"] = owner.create_subscription(
            String,
            VISION_STATUS_TOPIC,
            lambda msg: _on_vision_status(owner, msg),
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
    if _is_compressed_image_msg(msg) and bool(getattr(owner, "_stereo_camera_fast_display", True)):
        _on_compressed_stereo_camera_image(owner, side, msg)
        return
    status: dict[str, Any] | None = None
    try:
        rgb = _msg_to_rgb_array(msg)
        detector = getattr(owner, "_stereo_camera_buoy_detector", None)
        if detector is not None:
            rgb, status = detector.process_rgb(rgb)
        jpeg = _rgb_array_to_jpeg(rgb, quality=int(owner._stereo_camera_jpeg_quality))
    except Exception as exc:
        owner._stereo_camera_last_error = str(exc)
        return
    _publish_yolo_detections(owner, side, msg, status)
    width, height, encoding = _frame_metadata(msg, rgb)
    with owner._lock:
        seq = int(owner._stereo_camera_seq.get(side, 0)) + 1
        owner._stereo_camera_seq[side] = seq
        owner._stereo_camera_frames[side] = StereoCameraFrame(
            data=jpeg,
            content_type="image/jpeg",
            width=width,
            height=height,
            encoding=encoding,
            seq=seq,
            wall_s=time.monotonic(),
        )
    owner._touch(f"stereo_{side}")


def _on_compressed_stereo_camera_image(
    owner: Any, side: str, msg: Any, *, process_detector: bool = True
) -> None:
    data = bytes(msg.data)
    now = time.monotonic()
    status: dict[str, Any] | None = None
    detector = getattr(owner, "_stereo_camera_buoy_detector", None)
    should_process = process_detector and detector is not None and _camera_processing_due(owner, now)
    if should_process:
        try:
            rgb = _compressed_image_msg_to_rgb_array(msg)
            rgb, status = detector.process_rgb(rgb)
            data = _rgb_array_to_jpeg(rgb, quality=int(owner._stereo_camera_jpeg_quality))
            width, height, encoding = _frame_metadata(msg, rgb)
        except Exception as exc:
            owner._stereo_camera_last_error = str(exc)
            width, height = _compressed_frame_dimensions(owner, side, data)
            encoding = str(getattr(msg, "format", "jpeg") or "jpeg")
        else:
            _publish_yolo_detections(owner, side, msg, status)
    else:
        width, height = _compressed_frame_dimensions(owner, side, data)
        encoding = str(getattr(msg, "format", "jpeg") or "jpeg")

    with owner._lock:
        seq = int(owner._stereo_camera_seq.get(side, 0)) + 1
        owner._stereo_camera_seq[side] = seq
        owner._stereo_camera_frames[side] = StereoCameraFrame(
            data=data,
            content_type="image/jpeg",
            width=width,
            height=height,
            encoding=encoding,
            seq=seq,
            wall_s=now,
        )
    owner._touch(f"stereo_{side}")


def _on_vision_status(owner: Any, msg: Any) -> None:
    try:
        payload = json.loads(str(msg.data))
    except (AttributeError, TypeError, json.JSONDecodeError):
        return
    if not isinstance(payload, dict):
        return
    with owner._lock:
        owner._stereo_camera_vision_status = payload
        owner._stereo_camera_vision_status_wall = time.monotonic()


def _camera_processing_due(owner: Any, now: float) -> bool:
    period = float(getattr(owner, "_stereo_camera_process_period_s", 0.2))
    last = float(getattr(owner, "_stereo_camera_last_process_wall", 0.0))
    if last > 0.0 and now - last < period:
        return False
    owner._stereo_camera_last_process_wall = now
    return True


def _publish_yolo_detections(owner: Any, side: str, msg: Any, status: dict[str, Any] | None) -> None:
    publisher = getattr(owner, "_yolo_buoy_detection_pub", None)
    if publisher is None or status is None:
        return
    payload = dict(status)
    payload["side"] = side
    payload["topic"] = stereo_camera_topic(side)
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is not None:
        payload["stamp"] = {
            "sec": int(getattr(stamp, "sec", 0)),
            "nanosec": int(getattr(stamp, "nanosec", 0)),
        }
    ros_msg = String()
    ros_msg.data = json.dumps(payload, separators=(",", ":"))
    publisher.publish(ros_msg)


def stereo_camera_status(owner: Any) -> dict[str, Any]:
    now = time.monotonic()
    with owner._lock:
        frames = dict(owner._stereo_camera_frames)
        error = str(owner._stereo_camera_last_error)
        enabled = bool(owner._stereo_camera_enabled)
        subscribed_sides = sorted(owner._stereo_camera_subscriptions)
        jpeg_quality = int(owner._stereo_camera_jpeg_quality)
        display_mode = str(owner._stereo_camera_display_mode)
        vision_status = dict(owner._stereo_camera_vision_status)
        vision_status_wall = float(owner._stereo_camera_vision_status_wall)
    raw_frame = frames.get("left")
    vision_frame = frames.get(VISION_SOURCE)
    vision_available = vision_frame is not None and _frame_fresh(vision_frame, now)
    display_source = "vision" if display_mode == "vision" and vision_available else "raw"
    selected_frame = vision_frame if display_source == "vision" else raw_frame
    payload: dict[str, Any] = {
        "enabled": enabled,
        "error": error,
        "jpeg_quality": jpeg_quality,
        "subscribed_sides": subscribed_sides,
        "topics": {side: stereo_camera_topic(side) for side in SIDES},
        "vision_topic": stereo_camera_topic(VISION_SOURCE),
        "display_mode": display_mode,
        "display_source": display_source,
        "vision_fallback": display_mode == "vision" and not vision_available,
    }
    detector = getattr(owner, "_stereo_camera_buoy_detector", None)
    if vision_status and now - vision_status_wall <= VISION_STATUS_MAX_AGE_S:
        payload["detection"] = _external_vision_detection_status(vision_status)
    elif detector is not None:
        payload["detection"] = detector.status_payload()
    payload["sources"] = {
        "raw": _frame_status(raw_frame, now),
        "vision": _frame_status(vision_frame, now),
    }
    for side in SIDES:
        frame = selected_frame if side == "left" else frames.get(side)
        payload[side] = _frame_status(frame, now)
    return payload


def _frame_status(frame: StereoCameraFrame | None, now: float) -> dict[str, Any]:
    if frame is None or not _frame_fresh(frame, now):
        return {
            "available": False,
            "age_s": None if frame is None else max(0.0, now - frame.wall_s),
            "seq": 0,
            "width": 0,
            "height": 0,
            "encoding": "",
        }
    return {
        "available": True,
        "age_s": now - frame.wall_s,
        "seq": frame.seq,
        "width": frame.width,
        "height": frame.height,
        "encoding": frame.encoding,
    }


def _external_vision_detection_status(status: dict[str, Any]) -> dict[str, Any]:
    target = status.get("target") if isinstance(status.get("target"), dict) else None
    detections: list[dict[str, Any]] = []
    if target is not None:
        detections.append({
            "label": str(target.get("label", "")),
            "confidence": float(target.get("confidence", 0.0) or 0.0),
            "xyxy": list(target.get("bbox_xyxy", [])),
            "center_px": list(target.get("mask_center_px", [])),
        })
    return {
        "enabled": True,
        "model_found": bool(status.get("model")),
        "count": int(status.get("raw_detection_count", len(detections)) or 0),
        "candidate_count": int(status.get("control_candidate_count", len(detections)) or 0),
        "target_detected": target is not None,
        "detections": detections,
        "last_inference_ms": float(status.get("inference_ms", 0.0) or 0.0),
        "model": dict(status.get("model", {})) if isinstance(status.get("model"), dict) else {},
        "runtime": "auv_buoy_vision_control",
        "vision": {"state": str(status.get("state", "SEARCH"))},
        "external": True,
    }


def stereo_camera_frame(owner: Any, side: str) -> StereoCameraFrame | None:
    if not bool(getattr(owner, "_stereo_camera_enabled", True)):
        return None
    with owner._lock:
        display_mode = str(owner._stereo_camera_display_mode)
        vision = owner._stereo_camera_frames.get(VISION_SOURCE)
        raw = owner._stereo_camera_frames.get(side)
    frame = vision if side == "left" and display_mode == "vision" and (
        vision is not None and _frame_fresh(vision, time.monotonic())
    ) else raw
    if frame is None or not _frame_fresh(frame, time.monotonic()):
        return None
    return frame


def _frame_fresh(frame: StereoCameraFrame, now: float) -> bool:
    return max(0.0, now - frame.wall_s) <= STALE_FRAME_MAX_AGE_S


def stereo_camera_topic(side: str) -> str:
    env_name = f"UUV_GUI_STEREO_{side.upper()}_TOPIC"
    topic = str(os.environ.get(env_name, DEFAULT_TOPICS.get(side, ""))).strip()
    return topic or DEFAULT_TOPICS.get(side, "")


def _message_type_for_topic(owner: Any, topic: str) -> Any:
    compressed_type = getattr(owner, "_stereo_camera_compressed_image_type", None)
    image_type = getattr(owner, "_stereo_camera_image_type", None)
    transport = str(os.environ.get("UUV_GUI_STEREO_CAMERA_TRANSPORT", "")).strip().lower()
    if compressed_type is not None and (transport == "compressed" or topic.endswith("/compressed")):
        return compressed_type
    return image_type


def _msg_to_rgb_array(msg: Any) -> np.ndarray:
    if _is_compressed_image_msg(msg):
        return _compressed_image_msg_to_rgb_array(msg)
    return _image_msg_to_rgb_array(msg)


def _is_compressed_image_msg(msg: Any) -> bool:
    return hasattr(msg, "format") and not hasattr(msg, "width")


def _compressed_image_msg_to_rgb_array(msg: Any) -> np.ndarray:
    image = PILImage.open(io.BytesIO(bytes(msg.data))).convert("RGB")
    return np.ascontiguousarray(np.asarray(image), dtype=np.uint8)


def _compressed_frame_dimensions(owner: Any, side: str, data: bytes) -> tuple[int, int]:
    with owner._lock:
        previous = owner._stereo_camera_frames.get(side)
    if previous is not None and previous.width > 0 and previous.height > 0:
        return previous.width, previous.height
    try:
        image = PILImage.open(io.BytesIO(data))
        return int(image.width), int(image.height)
    except Exception:
        return 0, 0


def _frame_metadata(msg: Any, rgb: np.ndarray) -> tuple[int, int, str]:
    if hasattr(msg, "width") and hasattr(msg, "height"):
        return int(msg.width), int(msg.height), str(getattr(msg, "encoding", ""))
    height, width = rgb.shape[:2]
    return int(width), int(height), str(getattr(msg, "format", "jpeg") or "jpeg")


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


def _env_float(name: str, default: float, *, minimum: float, maximum: float) -> float:
    try:
        value = float(str(os.environ.get(name, default)).strip())
    except (TypeError, ValueError):
        value = float(default)
    return max(float(minimum), min(float(maximum), value))


def _env_flag(name: str, default: bool) -> bool:
    raw = os.environ.get(name)
    if raw is None:
        return bool(default)
    return str(raw).strip().lower() not in {"0", "false", "no", "off", "disabled"}


__all__ = [
    "StereoCameraFrame",
    "clear_stereo_camera_frames",
    "initialize_stereo_camera_state",
    "initialize_stereo_camera_subscriptions",
    "on_stereo_camera_image",
    "set_stereo_camera_enabled",
    "set_stereo_camera_display_mode",
    "stereo_camera_topic",
    "stereo_camera_frame",
    "stereo_camera_status",
]
