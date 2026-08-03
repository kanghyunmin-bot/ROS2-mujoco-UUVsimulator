#!/usr/bin/env python3
"""상태 기반 YOLO 추론 게이트의 회귀 검사."""

from __future__ import annotations

import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DETECTOR_PATH = PACKAGE_ROOT / "scripts" / "yolo_buoy_detector.py"


def load_detector_module():
    spec = importlib.util.spec_from_file_location(
        "auv_buoy_vision_control_yolo_detector_gate", DETECTOR_PATH
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to import detector from {DETECTOR_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


YoloBuoyDetector = load_detector_module().YoloBuoyDetector


class _Logger:
    def info(self, *_args, **_kwargs) -> None:
        pass


def detector_without_ros_init():
    detector = object.__new__(YoloBuoyDetector)
    detector.inference_enabled = False
    detector._preview_prev_time = None
    detector._preview_fps = 0.0
    detector.get_logger = lambda: _Logger()
    return detector


class YoloInferenceGateTest(unittest.TestCase):
    def test_disabled_frame_skips_decode_and_inference(self) -> None:
        detector = detector_without_ros_init()
        detector._decode_compressed_image = lambda _msg: self.fail(
            "disabled detector must not decode JPEG"
        )
        detector._detect_targets = lambda _image: self.fail(
            "disabled detector must not run YOLO"
        )

        YoloBuoyDetector.on_image(detector, SimpleNamespace())

    def test_disable_pauses_without_unloading_model(self) -> None:
        detector = detector_without_ros_init()
        detector.inference_enabled = True
        model = object()
        detector.model = model

        detector._on_inference_enabled(SimpleNamespace(data=False))

        self.assertFalse(detector.inference_enabled)
        self.assertIs(detector.model, model)

    def test_enable_keeps_loaded_model_and_resumes_frames(self) -> None:
        detector = detector_without_ros_init()
        model = object()
        detector.model = model

        detector._on_inference_enabled(SimpleNamespace(data=True))

        self.assertTrue(detector.inference_enabled)
        self.assertIs(detector.model, model)


if __name__ == "__main__":
    unittest.main()
