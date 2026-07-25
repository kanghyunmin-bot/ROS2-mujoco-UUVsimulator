"""Vision-state pipeline for buoy search, approach, fine alignment, and capture."""

from __future__ import annotations

import os
import time
from dataclasses import dataclass
from typing import Any, Iterable

import numpy as np


VISION_SEARCH = "SEARCH"
VISION_APPROACH = "APPROACH"
VISION_SEGMENTATION = "SEGMENTATION"
VISION_FINE_ALIGN = "FINE_ALIGN"
VISION_CAPTURE = "CAPTURE"


@dataclass(frozen=True)
class VisionCommand:
    forward: float = 0.0
    sway: float = 0.0
    heave: float = 0.0
    yaw: float = 0.0
    capture: bool = False


@dataclass
class VisionPipelineConfig:
    enabled: bool = True
    white_balance: bool = True
    clahe: bool = True
    search_yaw: float = 0.16
    lost_timeout_s: float = 0.75
    approach_close_height_ratio: float = 0.30
    approach_close_area_ratio: float = 0.075
    fine_align_tolerance_x: float = 0.055
    fine_align_tolerance_y: float = 0.075
    capture_confirm_s: float = 0.35
    capture_duration_s: float = 1.2
    collector_x_norm: float = 0.50
    collector_y_norm: float = 0.56
    max_forward: float = 0.90
    max_yaw: float = 0.45
    max_heave: float = 0.38


@dataclass
class SegmentationResult:
    valid: bool
    center_px: tuple[float, float] | None = None
    bottom_px: tuple[float, float] | None = None
    area_px: int = 0
    bbox_xyxy: tuple[int, int, int, int] | None = None


class Kalman2D:
    """Constant-velocity Kalman filter for image-space target center."""

    def __init__(self) -> None:
        self.initialized = False
        self.x = np.zeros((4, 1), dtype=np.float64)
        self.p = np.eye(4, dtype=np.float64) * 100.0
        self.q = np.diag([4.0, 4.0, 40.0, 40.0]).astype(np.float64)
        self.r = np.eye(2, dtype=np.float64) * 16.0
        self.last_t = 0.0

    def reset(self) -> None:
        self.initialized = False
        self.x[:] = 0.0
        self.p[:] = np.eye(4, dtype=np.float64) * 100.0
        self.last_t = 0.0

    def predict(self, now: float) -> tuple[float, float] | None:
        if not self.initialized:
            return None
        dt = max(1.0e-3, min(0.25, now - self.last_t))
        f = np.array(
            [[1.0, 0.0, dt, 0.0], [0.0, 1.0, 0.0, dt], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        self.x = f @ self.x
        self.p = f @ self.p @ f.T + self.q * dt
        self.last_t = now
        return float(self.x[0, 0]), float(self.x[1, 0])

    def update(self, center: tuple[float, float], now: float) -> tuple[float, float]:
        if not self.initialized:
            self.x = np.array([[center[0]], [center[1]], [0.0], [0.0]], dtype=np.float64)
            self.p = np.eye(4, dtype=np.float64) * 25.0
            self.initialized = True
            self.last_t = now
            return center
        self.predict(now)
        z = np.array([[center[0]], [center[1]]], dtype=np.float64)
        h = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]], dtype=np.float64)
        innovation = z - h @ self.x
        s = h @ self.p @ h.T + self.r
        k = self.p @ h.T @ np.linalg.inv(s)
        self.x = self.x + k @ innovation
        self.p = (np.eye(4, dtype=np.float64) - k @ h) @ self.p
        return float(self.x[0, 0]), float(self.x[1, 0])


class VisionBuoyPipeline:
    def __init__(self, config: VisionPipelineConfig | None = None) -> None:
        self.config = config or VisionPipelineConfig()
        self.state = VISION_SEARCH
        self._kalman = Kalman2D()
        self._last_gray: np.ndarray | None = None
        self._last_center: tuple[float, float] | None = None
        self._last_detection_t = 0.0
        self._capture_started_t = 0.0
        self._aligned_since_t = 0.0
        self._target_label = ""
        self._last_status: dict[str, Any] = self._empty_status()

    def process(
        self,
        rgb: np.ndarray,
        detections: Iterable[Any],
        *,
        ran_yolo: bool,
        depth_m: float | None = None,
        now: float | None = None,
    ) -> dict[str, Any]:
        now = time.monotonic() if now is None else float(now)
        frame_h, frame_w = rgb.shape[:2]
        detections = list(detections)
        target = select_largest_detection(detections)
        flow_center, flow_ok = self._track_with_optical_flow(rgb, now)
        measured_center = detection_center(target) if target is not None else flow_center

        if measured_center is not None:
            stable_center = self._kalman.update(measured_center, now)
            self._last_center = stable_center
            if target is not None:
                self._last_detection_t = now
                self._target_label = str(getattr(target, "label", "buoy"))
        else:
            stable_center = self._kalman.predict(now)
            self._last_center = stable_center

        target_present = stable_center is not None and (now - self._last_detection_t <= self.config.lost_timeout_s)
        segmentation = segment_buoy(rgb, target) if target is not None and close_to_buoy(target, frame_w, frame_h, self.config) else SegmentationResult(False)

        if not target_present:
            self.state = VISION_SEARCH
            self._kalman.reset()
            self._aligned_since_t = 0.0
            command = VisionCommand(yaw=self.config.search_yaw)
        elif self.state == VISION_CAPTURE:
            if now - self._capture_started_t >= self.config.capture_duration_s:
                self.state = VISION_SEARCH
                self._kalman.reset()
                self._aligned_since_t = 0.0
                command = VisionCommand(yaw=self.config.search_yaw)
            else:
                command = VisionCommand(forward=0.24, heave=-0.36, capture=True)
        elif target is not None and close_to_buoy(target, frame_w, frame_h, self.config):
            self.state = VISION_FINE_ALIGN if segmentation.valid else VISION_SEGMENTATION
            align_point = segmentation.center_px or stable_center
            command = self._fine_align_command(align_point, frame_w, frame_h)
            if self._aligned(align_point, frame_w, frame_h):
                if self._aligned_since_t <= 0.0:
                    self._aligned_since_t = now
                if now - self._aligned_since_t >= self.config.capture_confirm_s:
                    self.state = VISION_CAPTURE
                    self._capture_started_t = now
                    command = VisionCommand(forward=0.24, heave=-0.36, capture=True)
            else:
                self._aligned_since_t = 0.0
        else:
            self.state = VISION_APPROACH
            self._aligned_since_t = 0.0
            command = self._approach_command(stable_center, target, frame_w, frame_h, depth_m)

        self._last_gray = _gray(rgb)
        if stable_center is not None:
            self._last_center = stable_center
        self._last_status = self._status(
            command=command,
            target=target,
            stable_center=stable_center,
            frame_w=frame_w,
            frame_h=frame_h,
            ran_yolo=ran_yolo,
            flow_ok=flow_ok,
            segmentation=segmentation,
        )
        return dict(self._last_status)

    def status_payload(self) -> dict[str, Any]:
        return dict(self._last_status)

    def _approach_command(
        self,
        center: tuple[float, float] | None,
        target: Any | None,
        frame_w: int,
        frame_h: int,
        depth_m: float | None,
    ) -> VisionCommand:
        if center is None:
            return VisionCommand(yaw=self.config.search_yaw)
        ex, ey = normalized_error(center, frame_w, frame_h, self.config)
        size = bbox_height_ratio(target, frame_h) if target is not None else 0.0
        forward = clamp(self.config.max_forward * (1.0 - 1.7 * size), 0.18, self.config.max_forward)
        if abs(ex) > 0.42:
            forward = min(forward, 0.22)
        elif abs(ex) > 0.22:
            forward = min(forward, 0.45)
        if depth_m is not None and depth_m < 0.5:
            forward = min(forward, 0.35)
        return VisionCommand(
            forward=forward,
            heave=clamp(0.55 * ey, -self.config.max_heave, self.config.max_heave),
            yaw=clamp(0.62 * ex, -self.config.max_yaw, self.config.max_yaw),
        )

    def _fine_align_command(self, center: tuple[float, float] | None, frame_w: int, frame_h: int) -> VisionCommand:
        if center is None:
            return VisionCommand(yaw=self.config.search_yaw)
        ex, ey = normalized_error(center, frame_w, frame_h, self.config)
        return VisionCommand(
            forward=0.16 if abs(ex) < 0.12 and abs(ey) < 0.14 else 0.04,
            heave=clamp(0.42 * ey, -0.24, 0.24),
            yaw=clamp(0.72 * ex, -0.32, 0.32),
        )

    def _aligned(self, center: tuple[float, float] | None, frame_w: int, frame_h: int) -> bool:
        if center is None:
            return False
        ex, ey = normalized_error(center, frame_w, frame_h, self.config)
        return abs(ex) <= self.config.fine_align_tolerance_x and abs(ey) <= self.config.fine_align_tolerance_y

    def _track_with_optical_flow(self, rgb: np.ndarray, now: float) -> tuple[tuple[float, float] | None, bool]:
        del now
        if self._last_gray is None or self._last_center is None:
            return None, False
        try:
            import cv2  # type: ignore
        except Exception:
            return None, False
        gray = _gray(rgb)
        p0 = np.array([[[self._last_center[0], self._last_center[1]]]], dtype=np.float32)
        p1, st, _err = cv2.calcOpticalFlowPyrLK(self._last_gray, gray, p0, None, winSize=(31, 31), maxLevel=3)
        if p1 is None or st is None or int(st.reshape(-1)[0]) != 1:
            return None, False
        x, y = p1.reshape(-1, 2)[0]
        h, w = rgb.shape[:2]
        if not (0.0 <= x < w and 0.0 <= y < h):
            return None, False
        return (float(x), float(y)), True

    def _status(
        self,
        *,
        command: VisionCommand,
        target: Any | None,
        stable_center: tuple[float, float] | None,
        frame_w: int,
        frame_h: int,
        ran_yolo: bool,
        flow_ok: bool,
        segmentation: SegmentationResult,
    ) -> dict[str, Any]:
        target_payload: dict[str, Any] | None = None
        if target is not None:
            x1, y1, x2, y2 = detection_xyxy(target)
            target_payload = {
                "label": str(getattr(target, "label", "buoy")),
                "confidence": float(getattr(target, "confidence", 0.0)),
                "xyxy": [x1, y1, x2, y2],
                "height_ratio": bbox_height_ratio(target, frame_h),
                "area_ratio": bbox_area_ratio(target, frame_w, frame_h),
            }
        stable_payload = None
        if stable_center is not None and frame_w > 0 and frame_h > 0:
            ex, ey = normalized_error(stable_center, frame_w, frame_h, self.config)
            stable_payload = {
                "center_px": [round(stable_center[0], 1), round(stable_center[1], 1)],
                "center_norm": [stable_center[0] / frame_w, stable_center[1] / frame_h],
                "error_norm": [ex, ey],
            }
        return {
            "enabled": self.config.enabled,
            "state": self.state,
            "target_label": self._target_label,
            "ran_yolo": bool(ran_yolo),
            "flow_tracking": bool(flow_ok),
            "target": target_payload,
            "stable_target": stable_payload,
            "segmentation": {
                "valid": segmentation.valid,
                "center_px": list(segmentation.center_px) if segmentation.center_px else None,
                "bottom_px": list(segmentation.bottom_px) if segmentation.bottom_px else None,
                "area_px": segmentation.area_px,
                "bbox_xyxy": list(segmentation.bbox_xyxy) if segmentation.bbox_xyxy else None,
            },
            "command": {
                "forward": command.forward,
                "sway": command.sway,
                "heave": command.heave,
                "yaw": command.yaw,
                "capture": command.capture,
            },
        }

    @staticmethod
    def _empty_status() -> dict[str, Any]:
        return {
            "enabled": True,
            "state": VISION_SEARCH,
            "target_label": "",
            "ran_yolo": False,
            "flow_tracking": False,
            "target": None,
            "stable_target": None,
            "segmentation": {"valid": False, "center_px": None, "bottom_px": None, "area_px": 0, "bbox_xyxy": None},
            "command": {"forward": 0.0, "sway": 0.0, "heave": 0.0, "yaw": 0.0, "capture": False},
        }


def create_vision_buoy_pipeline() -> VisionBuoyPipeline:
    return VisionBuoyPipeline(
        VisionPipelineConfig(
            enabled=_env_bool("UUV_GUI_VISION_FSM_ENABLE", True),
            white_balance=_env_bool("UUV_GUI_VISION_WHITE_BALANCE", True),
            clahe=_env_bool("UUV_GUI_VISION_CLAHE", True),
            collector_x_norm=_env_float("UUV_GUI_VISION_COLLECTOR_X", 0.50, 0.10, 0.90),
            collector_y_norm=_env_float("UUV_GUI_VISION_COLLECTOR_Y", 0.56, 0.10, 0.90),
        )
    )


def preprocess_underwater_rgb(rgb: np.ndarray, config: VisionPipelineConfig | None = None) -> np.ndarray:
    config = config or VisionPipelineConfig()
    if not config.enabled or (not config.white_balance and not config.clahe):
        return rgb
    try:
        import cv2  # type: ignore
    except Exception:
        return rgb
    out = np.ascontiguousarray(rgb.copy())
    if config.white_balance:
        out = gray_world_white_balance(out)
    if config.clahe:
        lab = cv2.cvtColor(out, cv2.COLOR_RGB2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=2.2, tileGridSize=(8, 8))
        l = clahe.apply(l)
        out = cv2.cvtColor(cv2.merge((l, a, b)), cv2.COLOR_LAB2RGB)
    return out


def gray_world_white_balance(rgb: np.ndarray) -> np.ndarray:
    arr = rgb.astype(np.float32)
    means = arr.reshape(-1, 3).mean(axis=0)
    valid = means > 1.0e-3
    if not np.any(valid):
        return rgb.copy()
    gray = float(means[valid].mean())
    scale = np.ones(3, dtype=np.float32)
    scale[valid] = gray / means[valid]
    return np.clip(arr * scale.reshape(1, 1, 3), 0, 255).astype(np.uint8)


def draw_vision_guidance(rgb: np.ndarray, vision: dict[str, Any]) -> np.ndarray:
    try:
        import cv2  # type: ignore
    except Exception:
        return rgb
    out = np.ascontiguousarray(rgb.copy())
    h, w = out.shape[:2]
    collector = (int(round(w * 0.50)), int(round(h * 0.56)))
    cv2.drawMarker(out, collector, (255, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
    stable = vision.get("stable_target") or {}
    center = stable.get("center_px")
    if isinstance(center, list) and len(center) == 2:
        target = (int(round(center[0])), int(round(center[1])))
        cv2.circle(out, target, 5, (0, 255, 255), -1)
        cv2.line(out, collector, target, (0, 255, 255), 2)
    segmentation = vision.get("segmentation") or {}
    mask_center = segmentation.get("center_px")
    if isinstance(mask_center, list) and len(mask_center) == 2:
        cv2.circle(out, (int(mask_center[0]), int(mask_center[1])), 7, (80, 255, 80), 2)
    state = str(vision.get("state") or VISION_SEARCH)
    command = vision.get("command") or {}
    label = f"{state} f={float(command.get('forward', 0.0)):+.2f} h={float(command.get('heave', 0.0)):+.2f} y={float(command.get('yaw', 0.0)):+.2f}"
    cv2.putText(out, label, (8, max(22, h - 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (230, 245, 255), 2, cv2.LINE_AA)
    return out


def select_largest_detection(detections: Iterable[Any]) -> Any | None:
    best = None
    best_area = -1
    for det in detections:
        x1, y1, x2, y2 = detection_xyxy(det)
        area = max(0, x2 - x1) * max(0, y2 - y1)
        if area > best_area:
            best = det
            best_area = area
    return best


def detection_xyxy(det: Any) -> tuple[int, int, int, int]:
    xyxy = getattr(det, "xyxy", None)
    if xyxy is None and isinstance(det, dict):
        xyxy = det.get("xyxy")
    if xyxy is None:
        return (0, 0, 0, 0)
    x1, y1, x2, y2 = [int(round(float(v))) for v in xyxy[:4]]
    return x1, y1, x2, y2


def detection_center(det: Any) -> tuple[float, float]:
    x1, y1, x2, y2 = detection_xyxy(det)
    return (0.5 * (x1 + x2), 0.5 * (y1 + y2))


def bbox_height_ratio(det: Any | None, frame_h: int) -> float:
    if det is None or frame_h <= 0:
        return 0.0
    _x1, y1, _x2, y2 = detection_xyxy(det)
    return max(0.0, float(y2 - y1) / float(frame_h))


def bbox_area_ratio(det: Any | None, frame_w: int, frame_h: int) -> float:
    if det is None or frame_w <= 0 or frame_h <= 0:
        return 0.0
    x1, y1, x2, y2 = detection_xyxy(det)
    return max(0.0, float(max(0, x2 - x1) * max(0, y2 - y1)) / float(frame_w * frame_h))


def close_to_buoy(det: Any, frame_w: int, frame_h: int, config: VisionPipelineConfig) -> bool:
    return (
        bbox_height_ratio(det, frame_h) >= config.approach_close_height_ratio
        or bbox_area_ratio(det, frame_w, frame_h) >= config.approach_close_area_ratio
    )


def normalized_error(center: tuple[float, float], frame_w: int, frame_h: int, config: VisionPipelineConfig) -> tuple[float, float]:
    if frame_w <= 0 or frame_h <= 0:
        return 0.0, 0.0
    ex = (center[0] - config.collector_x_norm * frame_w) / max(1.0, 0.5 * frame_w)
    ey = (center[1] - config.collector_y_norm * frame_h) / max(1.0, 0.5 * frame_h)
    return clamp(ex, -1.5, 1.5), clamp(ey, -1.5, 1.5)


def segment_buoy(rgb: np.ndarray, det: Any | None) -> SegmentationResult:
    if det is None:
        return SegmentationResult(False)
    try:
        import cv2  # type: ignore
    except Exception:
        return SegmentationResult(False)
    h, w = rgb.shape[:2]
    x1, y1, x2, y2 = detection_xyxy(det)
    pad = max(4, int(0.08 * max(x2 - x1, y2 - y1)))
    x1 = max(0, x1 - pad)
    y1 = max(0, y1 - pad)
    x2 = min(w - 1, x2 + pad)
    y2 = min(h - 1, y2 + pad)
    if x2 <= x1 or y2 <= y1:
        return SegmentationResult(False)
    roi = rgb[y1:y2, x1:x2]
    hsv = cv2.cvtColor(roi, cv2.COLOR_RGB2HSV)
    label = str(getattr(det, "label", "")).lower()
    if "red" in label:
        mask = cv2.inRange(hsv, (0, 45, 35), (12, 255, 255)) | cv2.inRange(hsv, (168, 45, 35), (180, 255, 255))
    elif "yellow" in label:
        mask = cv2.inRange(hsv, (18, 35, 35), (42, 255, 255))
    elif "orange" in label:
        mask = cv2.inRange(hsv, (5, 35, 35), (27, 255, 255))
    elif "white" in label:
        mask = cv2.inRange(hsv, (0, 0, 120), (180, 70, 255))
    else:
        mask = cv2.inRange(hsv, (0, 35, 40), (180, 255, 255))
    kernel = np.ones((5, 5), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _hier = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return SegmentationResult(False)
    contour = max(contours, key=cv2.contourArea)
    area = int(cv2.contourArea(contour))
    if area < max(24, int(0.01 * roi.shape[0] * roi.shape[1])):
        return SegmentationResult(False)
    moments = cv2.moments(contour)
    if abs(moments["m00"]) < 1.0e-6:
        return SegmentationResult(False)
    cx = x1 + moments["m10"] / moments["m00"]
    cy = y1 + moments["m01"] / moments["m00"]
    pts = contour.reshape(-1, 2)
    bottom_idx = int(np.argmax(pts[:, 1]))
    bottom = pts[bottom_idx]
    bx, by, bw, bh = cv2.boundingRect(contour)
    return SegmentationResult(
        True,
        center_px=(float(cx), float(cy)),
        bottom_px=(float(x1 + bottom[0]), float(y1 + bottom[1])),
        area_px=area,
        bbox_xyxy=(int(x1 + bx), int(y1 + by), int(x1 + bx + bw), int(y1 + by + bh)),
    )


def _gray(rgb: np.ndarray) -> np.ndarray:
    try:
        import cv2  # type: ignore

        return cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    except Exception:
        return np.asarray(np.mean(rgb, axis=2), dtype=np.uint8)


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, float(value)))


def _env_bool(name: str, default: bool) -> bool:
    raw = os.environ.get(name)
    if raw is None:
        return default
    return raw.strip().lower() not in {"0", "false", "no", "off", "disabled"}


def _env_float(name: str, default: float, minimum: float, maximum: float) -> float:
    try:
        value = float(os.environ.get(name, str(default)))
    except (TypeError, ValueError):
        value = float(default)
    return max(float(minimum), min(float(maximum), value))
