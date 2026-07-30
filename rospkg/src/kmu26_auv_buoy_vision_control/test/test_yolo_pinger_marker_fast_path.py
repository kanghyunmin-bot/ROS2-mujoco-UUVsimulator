#!/usr/bin/env python3
"""Regression tests for the competition-pinger marker fast path."""

from __future__ import annotations

import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest

import cv2
import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DETECTOR_PATH = PACKAGE_ROOT / "scripts" / "yolo_buoy_detector.py"


def load_detector_module():
    spec = importlib.util.spec_from_file_location(
        "auv_buoy_vision_control_yolo_detector", DETECTOR_PATH
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to import detector from {DETECTOR_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


DETECTOR_MODULE = load_detector_module()
YoloBuoyDetector = DETECTOR_MODULE.YoloBuoyDetector


class _SilentLogger:
    def info(self, *_args, **_kwargs) -> None:
        pass


def marker_image() -> tuple[np.ndarray, tuple[int, int, int, int]]:
    image = np.zeros((120, 160, 3), dtype=np.uint8)
    x, y, width, height = 70, 40, 8, 24
    # BGR(190, 95, 0) -> HSV(105, 255, 190), inside the competition
    # pinger marker range declared by _detect_pinger_marker().
    image[y : y + height, x : x + width] = (190, 95, 0)
    return image, (x, y, width, height)


def detector_without_ros_init():
    detector = object.__new__(YoloBuoyDetector)
    detector.pinger_marker_fallback = True
    detector.pinger_marker_target_id = "course_buoy_pinger_white_1_float"
    detector.publish_per_class = True
    detector.publish_annotated_image = False
    detector.show_preview = False
    detector.target_class_id = -1
    detector.target_class_name = ""
    detector.area_similar_ratio = 0.15
    detector.confidence_similar_delta = 0.05
    detector.get_logger = lambda: _SilentLogger()
    return detector


class PingerMarkerFastPathTest(unittest.TestCase):
    def test_physical_pinger_detach_switches_to_course_yolo(self) -> None:
        detector = detector_without_ros_init()

        detector._on_pinger_detached_id(SimpleNamespace(data="unrelated_buoy"))
        self.assertTrue(detector.pinger_marker_fallback)

        detector._on_pinger_detached_id(
            SimpleNamespace(data="course_buoy_pinger_white_1_float")
        )
        self.assertFalse(detector.pinger_marker_fallback)

    def test_blue_marker_bypasses_yolo_and_publishes_both_classes(self) -> None:
        detector = detector_without_ros_init()
        image, _ = marker_image()
        published: list[tuple[float, tuple[float, ...], int, int]] = []

        detector._decode_compressed_image = lambda _msg: image

        def unexpected_yolo(_image):
            raise AssertionError("_detect_targets must not run for a valid blue marker")

        detector._detect_targets = unexpected_yolo
        detector._publish_detection = (
            lambda stamp, detection, width, height: published.append(
                (stamp, tuple(detection), width, height)
            )
        )
        message = SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(sec=12, nanosec=250_000_000)
            )
        )

        YoloBuoyDetector.on_image(detector, message)

        self.assertEqual([int(row[1][0]) for row in published], [0, 1])
        self.assertEqual([(row[2], row[3]) for row in published], [(160, 120)] * 2)
        self.assertTrue(all(abs(row[0] - 12.25) < 1.0e-9 for row in published))

    def test_class_one_center_tracks_marker_and_pvc_contact_height(self) -> None:
        detector = detector_without_ros_init()
        image, (_, marker_y, _, marker_height) = marker_image()

        selected, detections = detector._detect_pinger_marker(image)

        self.assertIsNotNone(selected)
        self.assertEqual([int(row[0]) for row in detections], [0, 1])
        stick = next(row for row in detections if int(row[0]) == 1)
        marker_center_y = marker_y + 0.5 * marker_height
        self.assertAlmostEqual(stick[3], marker_center_y, delta=0.5)
        self.assertGreaterEqual(stick[3], marker_y)
        self.assertLessEqual(stick[3], marker_y + marker_height)


if __name__ == "__main__":
    unittest.main()
