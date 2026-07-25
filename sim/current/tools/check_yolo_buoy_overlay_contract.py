#!/usr/bin/env python3
"""Static and drawing checks for the YOLO buoy camera overlay."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from gui.yolo_buoy_detector import BuoyDetection, draw_buoy_detections  # noqa: E402


def main() -> int:
    image = np.zeros((180, 320, 3), dtype=np.uint8)
    image[:, :] = (20, 40, 60)
    detections = [
        BuoyDetection(label="red_buoy", confidence=0.87, xyxy=(45, 35, 155, 145)),
        BuoyDetection(label="yellow_buoy", confidence=0.76, xyxy=(175, 45, 280, 150)),
    ]
    drawn = draw_buoy_detections(image, detections)
    if drawn.shape != image.shape:
        raise AssertionError("drawn image shape changed")
    if not np.any(drawn != image):
        raise AssertionError("YOLO overlay did not draw any pixels")
    if np.shares_memory(drawn, image):
        raise AssertionError("YOLO overlay must not mutate the source image in place")
    print("yolo_buoy_overlay_contract=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
