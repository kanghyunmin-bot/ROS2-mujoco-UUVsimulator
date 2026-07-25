"""YOLO buoy detection overlay for the web GUI camera feed."""

from __future__ import annotations

import os
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import numpy as np

from .vision_buoy_pipeline import (
    create_vision_buoy_pipeline,
    draw_vision_guidance,
    preprocess_underwater_rgb,
)


DEFAULT_MODEL_NAME = "best.pt"
DEFAULT_CONFIDENCE = 0.35
DEFAULT_IOU = 0.45
DEFAULT_IMG_SIZE = 768
DEFAULT_MAX_DETECTIONS = 24
DEFAULT_MAX_HZ = 4.0


@dataclass(frozen=True)
class BuoyDetection:
    label: str
    confidence: float
    xyxy: tuple[int, int, int, int]


def create_yolo_buoy_detector() -> "YoloBuoyDetector":
    model_path = _resolve_model_path()
    # Mission vision owns inference and publishes the annotated stream. Running
    # a second Ultralytics model in the web process starves MuJoCo on CPU-only
    # systems; operators can still opt in explicitly for standalone preview.
    mode = os.environ.get("UUV_GUI_YOLO_ENABLE", "off").strip().lower()
    model_found = model_path.is_file()
    enabled = mode not in {"0", "false", "no", "off", "disabled"}
    if mode in {"auto", ""}:
        enabled = model_found
    return YoloBuoyDetector(
        enabled=enabled,
        model_path=model_path,
        model_found=model_found,
        confidence=_env_float("UUV_GUI_YOLO_CONF", DEFAULT_CONFIDENCE, 0.01, 0.99),
        iou=_env_float("UUV_GUI_YOLO_IOU", DEFAULT_IOU, 0.01, 0.99),
        image_size=_env_int("UUV_GUI_YOLO_IMG_SIZE", DEFAULT_IMG_SIZE, 64, 1920),
        max_detections=_env_int("UUV_GUI_YOLO_MAX_DETECTIONS", DEFAULT_MAX_DETECTIONS, 1, 200),
        max_hz=_env_float("UUV_GUI_YOLO_MAX_HZ", DEFAULT_MAX_HZ, 0.1, 60.0),
        device=os.environ.get("UUV_GUI_YOLO_DEVICE", "").strip(),
        allowed_classes=_class_filter_tokens(os.environ.get("UUV_GUI_YOLO_CLASSES", "")),
    )


class YoloBuoyDetector:
    def __init__(
        self,
        *,
        enabled: bool,
        model_path: Path,
        model_found: bool,
        confidence: float,
        iou: float,
        image_size: int,
        max_detections: int,
        max_hz: float,
        device: str,
        allowed_classes: set[str],
    ) -> None:
        self.enabled = bool(enabled)
        self.model_path = model_path
        self.model_found = bool(model_found)
        self.confidence = float(confidence)
        self.iou = float(iou)
        self.image_size = int(image_size)
        self.max_detections = int(max_detections)
        self.max_hz = float(max_hz)
        self.device = device
        self.allowed_classes = allowed_classes
        self._lock = threading.Lock()
        self._model: Any | None = None
        self._load_attempted = False
        self._error = "" if self.model_found or not self.enabled else f"YOLO model not found: {self.model_path}"
        self._last_detections: list[BuoyDetection] = []
        self._last_frame_size: tuple[int, int] = (0, 0)
        self._last_inference_wall = 0.0
        self._last_inference_ms = 0.0
        self._runtime = ""
        self._pipeline = create_vision_buoy_pipeline()

    def process_rgb(self, rgb: np.ndarray) -> tuple[np.ndarray, dict[str, Any]]:
        if not self.enabled:
            return rgb, self.status_payload()
        if not self.model_found:
            return rgb, self.status_payload()

        now = time.monotonic()
        period = 1.0 / max(self.max_hz, 0.1)
        with self._lock:
            cached = list(self._last_detections)
            recently_processed = self._last_inference_wall > 0.0 and now - self._last_inference_wall < period
        if recently_processed:
            vision = self._pipeline.process(rgb, cached, ran_yolo=False)
            drawn = draw_vision_guidance(draw_buoy_detections(rgb, cached), vision)
            return drawn, self.status_payload()

        if not self._ensure_model():
            return rgb, self.status_payload()

        start = time.perf_counter()
        detections: list[BuoyDetection] = []
        try:
            model_rgb = preprocess_underwater_rgb(rgb, self._pipeline.config)
            detections = self._predict(model_rgb)
            vision = self._pipeline.process(rgb, detections, ran_yolo=True)
            drawn = draw_vision_guidance(draw_buoy_detections(rgb, detections), vision)
        except Exception as exc:
            elapsed_ms = (time.perf_counter() - start) * 1000.0
            with self._lock:
                self._error = str(exc)
                self._last_detections = []
                self._last_frame_size = _frame_size(rgb)
                self._last_inference_wall = time.monotonic()
                self._last_inference_ms = elapsed_ms
            return rgb, self.status_payload()

        elapsed_ms = (time.perf_counter() - start) * 1000.0
        with self._lock:
            self._last_detections = detections
            self._last_frame_size = _frame_size(rgb)
            self._last_inference_wall = time.monotonic()
            self._last_inference_ms = elapsed_ms
            self._error = ""
        return drawn, self.status_payload()

    def status_payload(self) -> dict[str, Any]:
        with self._lock:
            count = len(self._last_detections)
            labels = [det.label for det in self._last_detections[:8]]
            detections = list(self._last_detections[:20])
            frame_width, frame_height = self._last_frame_size
            error = self._error
            last_inference_ms = self._last_inference_ms
            runtime = self._runtime
        return {
            "enabled": self.enabled,
            "active": bool(self.enabled and self.model_found and not error and runtime),
            "model_found": self.model_found,
            "model_path": str(self.model_path),
            "runtime": runtime,
            "error": error,
            "count": count,
            "labels": labels,
            "frame_width": frame_width,
            "frame_height": frame_height,
            "detections": [_detection_payload(det, frame_width, frame_height) for det in detections],
            "confidence": self.confidence,
            "iou": self.iou,
            "image_size": self.image_size,
            "max_hz": self.max_hz,
            "last_inference_ms": last_inference_ms,
            "vision": self._pipeline.status_payload(),
        }

    def _ensure_model(self) -> bool:
        with self._lock:
            if self._model is not None:
                return True
            if self._load_attempted:
                return False
            self._load_attempted = True
        try:
            from ultralytics import YOLO  # type: ignore

            model = YOLO(str(self.model_path))
        except Exception as exc:
            with self._lock:
                self._error = f"ultralytics unavailable: {exc}"
            return False
        with self._lock:
            self._model = model
            self._runtime = "ultralytics"
            self._error = ""
        return True

    def _predict(self, rgb: np.ndarray) -> list[BuoyDetection]:
        model = self._model
        if model is None:
            return []
        bgr = rgb[..., ::-1]
        kwargs: dict[str, Any] = {
            "source": bgr,
            "conf": self.confidence,
            "iou": self.iou,
            "imgsz": self.image_size,
            "max_det": self.max_detections,
            "verbose": False,
        }
        if self.device:
            kwargs["device"] = self.device
        results = model.predict(**kwargs)
        if not results:
            return []
        result = results[0]
        names = getattr(result, "names", None) or getattr(model, "names", {}) or {}
        boxes = getattr(result, "boxes", None)
        if boxes is None:
            return []
        xyxy = _to_numpy(getattr(boxes, "xyxy", []))
        confs = _to_numpy(getattr(boxes, "conf", []))
        class_ids = _to_numpy(getattr(boxes, "cls", []))
        detections: list[BuoyDetection] = []
        height, width = rgb.shape[:2]
        for index, coords in enumerate(xyxy):
            confidence = float(confs[index]) if index < len(confs) else 0.0
            class_id = int(class_ids[index]) if index < len(class_ids) else -1
            label = _class_label(names, class_id)
            if not _class_allowed(self.allowed_classes, class_id, label):
                continue
            x1, y1, x2, y2 = _clip_box(coords, width, height)
            if x2 <= x1 or y2 <= y1:
                continue
            detections.append(BuoyDetection(label=label, confidence=confidence, xyxy=(x1, y1, x2, y2)))
        return detections


def draw_buoy_detections(rgb: np.ndarray, detections: Iterable[BuoyDetection]) -> np.ndarray:
    detections = list(detections)
    if not detections:
        return rgb
    try:
        import cv2  # type: ignore
    except Exception:
        return rgb
    out = np.ascontiguousarray(rgb.copy())
    height, width = out.shape[:2]
    line = max(2, round(min(width, height) / 360))
    font_scale = max(0.45, min(width, height) / 900)
    for det in detections:
        x1, y1, x2, y2 = det.xyxy
        label = f"{det.label} {det.confidence:.2f}"
        color = _label_color(det.label)
        cv2.rectangle(out, (x1, y1), (x2, y2), color, line)
        (text_w, text_h), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, line)
        label_y1 = max(0, y1 - text_h - baseline - 6)
        label_y2 = min(height - 1, label_y1 + text_h + baseline + 6)
        label_x2 = min(width - 1, x1 + text_w + 10)
        cv2.rectangle(out, (x1, label_y1), (label_x2, label_y2), color, -1)
        cv2.putText(
            out,
            label,
            (x1 + 5, label_y2 - baseline - 3),
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            (7, 18, 28),
            max(1, line - 1),
            cv2.LINE_AA,
        )
    return out


def _frame_size(rgb: np.ndarray) -> tuple[int, int]:
    height, width = rgb.shape[:2]
    return int(width), int(height)


def _detection_payload(det: BuoyDetection, frame_width: int, frame_height: int) -> dict[str, Any]:
    x1, y1, x2, y2 = det.xyxy
    width = max(0, x2 - x1)
    height = max(0, y2 - y1)
    cx = x1 + width / 2.0
    cy = y1 + height / 2.0
    payload: dict[str, Any] = {
        "label": det.label,
        "confidence": det.confidence,
        "xyxy": [x1, y1, x2, y2],
        "center_px": [round(cx, 1), round(cy, 1)],
        "width_px": width,
        "height_px": height,
        "area_px": width * height,
    }
    if frame_width > 0 and frame_height > 0:
        payload["xyxy_norm"] = [
            x1 / frame_width,
            y1 / frame_height,
            x2 / frame_width,
            y2 / frame_height,
        ]
        payload["center_norm"] = [cx / frame_width, cy / frame_height]
    return payload


def _resolve_model_path() -> Path:
    explicit = os.environ.get("UUV_GUI_YOLO_MODEL", "").strip()
    workspace = Path(os.environ.get("WORKSPACE_DIR", "")).expanduser() if os.environ.get("WORKSPACE_DIR") else None
    candidates: list[Path] = []
    if explicit:
        path = Path(explicit).expanduser()
        candidates.append(path if path.is_absolute() else Path.cwd() / path)
        if workspace is not None:
            candidates.append(workspace / path)
    if workspace is not None:
        candidates.append(workspace / "sim" / "current" / "assets" / "yolo" / DEFAULT_MODEL_NAME)
    candidates.append(Path(__file__).resolve().parents[1] / "assets" / "yolo" / DEFAULT_MODEL_NAME)
    candidates.append(Path.cwd() / "assets" / "yolo" / DEFAULT_MODEL_NAME)
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    return candidates[0].resolve() if candidates else Path(DEFAULT_MODEL_NAME).resolve()


def _to_numpy(value: Any) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy"):
        return np.asarray(value.numpy())
    return np.asarray(value)


def _clip_box(coords: Any, width: int, height: int) -> tuple[int, int, int, int]:
    x1, y1, x2, y2 = [float(v) for v in coords[:4]]
    return (
        max(0, min(width - 1, int(round(x1)))),
        max(0, min(height - 1, int(round(y1)))),
        max(0, min(width - 1, int(round(x2)))),
        max(0, min(height - 1, int(round(y2)))),
    )


def _class_label(names: Any, class_id: int) -> str:
    if isinstance(names, dict):
        return str(names.get(class_id, names.get(str(class_id), f"buoy_{class_id}")))
    if isinstance(names, (list, tuple)) and 0 <= class_id < len(names):
        return str(names[class_id])
    return "buoy" if class_id < 0 else f"buoy_{class_id}"


def _class_filter_tokens(value: str) -> set[str]:
    return {part.strip().lower() for part in value.replace(";", ",").split(",") if part.strip()}


def _class_allowed(filters: set[str], class_id: int, label: str) -> bool:
    if not filters:
        return True
    return str(class_id) in filters or label.lower() in filters


def _label_color(label: str) -> tuple[int, int, int]:
    lower = label.lower()
    if "red" in lower:
        return (255, 91, 91)
    if "yellow" in lower:
        return (255, 218, 72)
    if "orange" in lower:
        return (255, 154, 46)
    if "green" in lower:
        return (84, 220, 120)
    return (71, 196, 255)


def _env_int(name: str, default: int, minimum: int, maximum: int) -> int:
    try:
        value = int(float(os.environ.get(name, str(default))))
    except (TypeError, ValueError):
        value = int(default)
    return max(int(minimum), min(int(maximum), value))


def _env_float(name: str, default: float, minimum: float, maximum: float) -> float:
    try:
        value = float(os.environ.get(name, str(default)))
    except (TypeError, ValueError):
        value = float(default)
    return max(float(minimum), min(float(maximum), value))


__all__ = [
    "BuoyDetection",
    "YoloBuoyDetector",
    "create_yolo_buoy_detector",
    "draw_buoy_detections",
]
