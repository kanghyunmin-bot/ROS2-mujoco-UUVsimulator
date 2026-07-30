#!/usr/bin/env python3
"""Regression tests for the competition course-buoy colour filter."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest

import cv2
import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DETECTOR_PATH = PACKAGE_ROOT / "scripts" / "yolo_buoy_detector.py"


def load_detector_module():
    spec = importlib.util.spec_from_file_location(
        "auv_buoy_vision_control_yolo_detector_course_filter", DETECTOR_PATH
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to import detector from {DETECTOR_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


YoloBuoyDetector = load_detector_module().YoloBuoyDetector


def detector_without_ros_init():
    detector = object.__new__(YoloBuoyDetector)
    detector.target_class_id = -1
    detector.target_class_name = ""
    detector.class_names = {0: "buoy", 1: "stick"}
    detector.area_similar_ratio = 0.15
    detector.confidence_similar_delta = 0.05
    detector.course_buoy_hue_min = 12
    detector.course_buoy_hue_max = 40
    detector.course_buoy_saturation_min = 100
    detector.course_buoy_value_min = 80
    detector.course_buoy_color_pixel_ratio_min = 0.04
    detector.course_buoy_min_confidence = 0.45
    detector.associate_stick_with_buoy = True
    detector.buoy_class_id = 0
    detector.stick_class_id = 1
    return detector


def detection(
    class_id: int,
    confidence: float,
    x1: int,
    y1: int,
    x2: int,
    y2: int,
):
    width = float(x2 - x1)
    height = float(y2 - y1)
    return (
        class_id,
        confidence,
        x1 + 0.5 * width,
        y1 + 0.5 * height,
        width,
        height,
        x1,
        y1,
        x2,
        y2,
    )


class CourseBuoyFilterTest(unittest.TestCase):
    def test_rejects_cyan_pool_false_positive(self) -> None:
        detector = detector_without_ros_init()
        hsv = np.full((120, 160, 3), (96, 220, 180), dtype=np.uint8)
        image = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        pool_box = detection(0, 0.9, 30, 20, 130, 100)

        filtered = detector._filter_course_buoy_detections(image, [pool_box])

        self.assertEqual(filtered, [])

    def test_keeps_orange_buoy_and_only_its_nearby_stick(self) -> None:
        detector = detector_without_ros_init()
        image = np.zeros((120, 200, 3), dtype=np.uint8)
        orange_bgr = cv2.cvtColor(
            np.array([[[20, 230, 220]]], dtype=np.uint8),
            cv2.COLOR_HSV2BGR,
        )[0, 0]
        image[30:65, 80:120] = orange_bgr
        buoy = detection(0, 0.8, 75, 25, 125, 70)
        nearby_stick = detection(1, 0.7, 92, 60, 108, 105)
        distant_stick = detection(1, 0.95, 160, 40, 180, 100)

        filtered = detector._filter_course_buoy_detections(
            image, [buoy, nearby_stick, distant_stick]
        )

        self.assertEqual(filtered, [buoy, nearby_stick])

    def test_rejects_low_confidence_buoy_but_keeps_low_stick_threshold(self) -> None:
        detector = detector_without_ros_init()
        image = np.zeros((120, 200, 3), dtype=np.uint8)
        orange_bgr = cv2.cvtColor(
            np.array([[[20, 230, 220]]], dtype=np.uint8),
            cv2.COLOR_HSV2BGR,
        )[0, 0]
        image[30:65, 80:120] = orange_bgr
        weak_buoy = detection(0, 0.40, 75, 25, 125, 70)
        weak_stick = detection(1, 0.36, 92, 60, 108, 105)

        filtered = detector._filter_course_buoy_detections(
            image, [weak_buoy, weak_stick]
        )

        self.assertEqual(filtered, [])

        strong_buoy = detection(0, 0.70, 75, 25, 125, 70)
        filtered = detector._filter_course_buoy_detections(
            image, [strong_buoy, weak_stick]
        )

        self.assertEqual(filtered, [strong_buoy, weak_stick])


if __name__ == "__main__":
    unittest.main()
