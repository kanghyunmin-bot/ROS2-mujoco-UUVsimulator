#!/usr/bin/env python3
"""Contract checks for the camera buoy vision FSM pipeline."""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from gui.vision_buoy_pipeline import (  # noqa: E402
    VISION_APPROACH,
    VISION_CAPTURE,
    VISION_FINE_ALIGN,
    VISION_SEARCH,
    VisionBuoyPipeline,
    VisionPipelineConfig,
    draw_vision_guidance,
    preprocess_underwater_rgb,
    segment_buoy,
)


@dataclass(frozen=True)
class FakeDetection:
    label: str
    confidence: float
    xyxy: tuple[int, int, int, int]


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def yellow_frame(width: int = 640, height: int = 360, bbox: tuple[int, int, int, int] = (420, 120, 540, 260)) -> np.ndarray:
    image = np.zeros((height, width, 3), dtype=np.uint8)
    image[:, :] = (18, 42, 70)
    x1, y1, x2, y2 = bbox
    cy = (y1 + y2) // 2
    cx = (x1 + x2) // 2
    yy, xx = np.ogrid[:height, :width]
    rx = max(1, (x2 - x1) // 2)
    ry = max(1, (y2 - y1) // 2)
    mask = ((xx - cx) / rx) ** 2 + ((yy - cy) / ry) ** 2 <= 1.0
    image[mask] = (245, 220, 45)
    return image


def check_preprocess_preserves_shape() -> None:
    image = yellow_frame()
    processed = preprocess_underwater_rgb(image, VisionPipelineConfig())
    require(processed.shape == image.shape, "preprocessing must preserve frame shape")
    require(processed.dtype == np.uint8, "preprocessing must return uint8 RGB")


def check_search_to_approach_command() -> None:
    pipeline = VisionBuoyPipeline(VisionPipelineConfig(capture_confirm_s=0.05))
    bbox = (450, 150, 510, 210)
    image = yellow_frame(bbox=bbox)
    status = pipeline.process(image, [FakeDetection("yellow_buoy", 0.90, bbox)], ran_yolo=True, now=1.0)
    require(status["state"] == VISION_APPROACH, "small target should enter approach mode")
    require(status["command"]["yaw"] > 0.10, "right-side bbox should command positive yaw")
    require(status["command"]["forward"] > 0.10, "approach mode should keep forward motion")
    require(status["stable_target"] is not None, "Kalman target center should be reported")


def check_close_target_segments_and_captures() -> None:
    pipeline = VisionBuoyPipeline(VisionPipelineConfig(capture_confirm_s=0.05, capture_duration_s=0.2))
    bbox = (260, 92, 380, 308)
    image = yellow_frame(bbox=bbox)
    detection = FakeDetection("yellow_buoy", 0.92, bbox)
    seg = segment_buoy(image, detection)
    require(seg.valid, "close yellow buoy should segment")
    status = pipeline.process(image, [detection], ran_yolo=True, now=2.0)
    require(status["state"] == VISION_FINE_ALIGN, "close segmented target should enter fine align")
    require(status["segmentation"]["valid"], "fine align status must include segmentation")
    status = pipeline.process(image, [detection], ran_yolo=True, now=2.2)
    require(status["state"] == VISION_CAPTURE, "aligned close target should enter capture after confirm hold")
    require(status["command"]["capture"], "capture command flag should be set")


def check_lost_target_returns_search() -> None:
    pipeline = VisionBuoyPipeline(VisionPipelineConfig(lost_timeout_s=0.2))
    image = yellow_frame()
    pipeline.process(image, [FakeDetection("yellow_buoy", 0.90, (420, 120, 540, 260))], ran_yolo=True, now=3.0)
    status = pipeline.process(image, [], ran_yolo=True, now=3.5)
    require(status["state"] == VISION_SEARCH, "lost target should return to search mode")
    require(status["command"]["yaw"] != 0.0, "search mode should command scanning yaw")


def check_guidance_overlay_draws() -> None:
    pipeline = VisionBuoyPipeline(VisionPipelineConfig())
    image = yellow_frame()
    status = pipeline.process(image, [FakeDetection("yellow_buoy", 0.90, (420, 120, 540, 260))], ran_yolo=True, now=4.0)
    drawn = draw_vision_guidance(image, status)
    require(drawn.shape == image.shape, "overlay must preserve shape")
    require(np.any(drawn != image), "overlay should draw guidance pixels")


def main() -> int:
    check_preprocess_preserves_shape()
    check_search_to_approach_command()
    check_close_target_segments_and_captures()
    check_lost_target_returns_search()
    check_guidance_overlay_draws()
    print("vision_buoy_pipeline=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
