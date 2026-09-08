#!/usr/bin/env python3
"""Focused offline tests for the deterministic underwater camera model."""

from __future__ import annotations

import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
import sys

import numpy as np


CURRENT = Path(__file__).resolve().parents[1]
if str(CURRENT) not in sys.path:
    sys.path.insert(0, str(CURRENT))

from bridge.underwater_camera_sensor_model import (  # noqa: E402
    CameraCalibration,
    CameraElectronicsConfig,
    UnderwaterCameraSensorModel,
    UnderwaterOpticsConfig,
    load_camera_calibration,
    load_underwater_camera_profile,
)


def _ideal_profile():
    profile = load_underwater_camera_profile()
    return replace(
        profile,
        optics=UnderwaterOpticsConfig(),
        electronics=CameraElectronicsConfig(),
    )


class CameraProfileTest(unittest.TestCase):
    def test_default_profile_is_explicitly_uncalibrated_and_opt_in(self) -> None:
        profile = load_underwater_camera_profile()
        payload = Path(
            CURRENT / "config/sensor_models/imx219_underwater_uncalibrated_prior.json"
        ).read_text(encoding="utf-8")

        self.assertIn("unvalidated_prior_not_measured", profile.calibration_status)
        self.assertIn('"enabled_by_default": false', payload)
        self.assertEqual(profile.calibration.distortion_model, "plumb_bob")
        self.assertFalse(profile.calibration.has_distortion)

    def test_ros_camera_info_yaml_loads_without_optional_yaml_dependency(self) -> None:
        calibration_yaml = """
image_width: 640
image_height: 480
camera_name: imx219_camera0
camera_matrix:
  rows: 3
  cols: 3
  data: [500.0, 0.0, 319.5, 0.0, 501.0, 239.5, 0.0, 0.0, 1.0]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [-0.1, 0.02, 0.001, -0.002, 0.0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [500.0, 0.0, 319.5, 0.0, 0.0, 501.0, 239.5, 0.0, 0.0, 0.0, 1.0, 0.0]
"""
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "camera.yaml"
            path.write_text(calibration_yaml, encoding="utf-8")
            calibration = load_camera_calibration(path)

        self.assertEqual((calibration.width, calibration.height), (640, 480))
        self.assertEqual(calibration.k[0], 500.0)
        self.assertEqual(calibration.d[2], 0.001)
        self.assertEqual(calibration.calibration_status, "user_supplied_calibration")

    def test_intrinsics_scale_with_runtime_resolution(self) -> None:
        calibration = load_underwater_camera_profile().calibration.scaled_to(640, 360)

        self.assertAlmostEqual(calibration.k[0], 257.06687, places=5)
        self.assertAlmostEqual(calibration.k[2], 319.75, places=5)
        self.assertAlmostEqual(calibration.k[4], 257.06687, places=5)
        self.assertAlmostEqual(calibration.k[5], 179.75, places=5)

    def test_physical_driver_ros_parameter_yaml_uses_runtime_dimensions(self) -> None:
        calibration_yaml = """
/**:
  ros__parameters:
    distortion_model: plumb_bob
    distortion_coefficients: [-0.1, 0.02, 0.001, -0.002, 0.0]
    camera_matrix: [500.0, 0.0, 639.5, 0.0, 501.0, 359.5, 0.0, 0.0, 1.0]
    rectification_matrix: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    projection_matrix: [500.0, 0.0, 639.5, 0.0, 0.0, 501.0, 359.5, 0.0, 0.0, 0.0, 1.0, 0.0]
"""
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "physical_driver.yaml"
            path.write_text(calibration_yaml, encoding="utf-8")
            calibration = load_camera_calibration(
                path,
                fallback_width=1280,
                fallback_height=720,
            )

        self.assertEqual((calibration.width, calibration.height), (1280, 720))
        self.assertEqual(calibration.k[0], 500.0)
        self.assertEqual(calibration.d[2], 0.001)

    def test_scaling_preserves_nonzero_skew_and_projection_translation(self) -> None:
        base = load_underwater_camera_profile().calibration
        custom = replace(
            base,
            k=(base.k[0], 2.0, base.k[2], 3.0, base.k[4], base.k[5], 0.0, 0.0, 1.0),
            p=(base.p[0], 2.0, base.p[2], 4.0, 3.0, base.p[5], base.p[6], 6.0, 0.0, 0.0, 1.0, 0.0),
        ).scaled_to(640, 180)

        self.assertEqual(custom.k[1], 1.0)
        self.assertEqual(custom.k[3], 0.75)
        self.assertEqual(custom.p[1], 1.0)
        self.assertEqual(custom.p[3], 2.0)
        self.assertEqual(custom.p[4], 0.75)
        self.assertEqual(custom.p[7], 1.5)


class CameraImageModelTest(unittest.TestCase):
    def test_disabled_model_is_byte_exact_legacy_parity(self) -> None:
        image = np.arange(9 * 11 * 3, dtype=np.uint8).reshape(9, 11, 3)
        model = UnderwaterCameraSensorModel(_ideal_profile(), enabled=False)

        output, diagnostics = model.process(image, sequence=4)

        np.testing.assert_array_equal(output, image)
        self.assertFalse(diagnostics.distortion_applied)
        self.assertFalse(diagnostics.noise_applied)
        self.assertIsNot(output, image)

    def test_seed_and_sequence_make_noise_reproducible(self) -> None:
        profile = replace(
            _ideal_profile(),
            electronics=CameraElectronicsConfig(
                shot_noise_electrons_per_unit=500.0,
                read_noise_electrons_rms=3.0,
            ),
        )
        image = np.full((32, 40, 3), 120, dtype=np.uint8)
        first = UnderwaterCameraSensorModel(profile, camera_name="stereo_left")
        second = UnderwaterCameraSensorModel(profile, camera_name="stereo_left")

        output_a, _ = first.process(image, sequence=7)
        output_b, _ = second.process(image, sequence=7)
        output_c, _ = second.process(image, sequence=8)

        np.testing.assert_array_equal(output_a, output_b)
        self.assertFalse(np.array_equal(output_a, output_c))

    def test_left_and_right_noise_streams_are_independent(self) -> None:
        profile = replace(
            _ideal_profile(),
            electronics=CameraElectronicsConfig(
                shot_noise_electrons_per_unit=1000.0,
                read_noise_electrons_rms=2.0,
            ),
        )
        image = np.full((20, 24, 3), 100, dtype=np.uint8)
        left, _ = UnderwaterCameraSensorModel(
            profile, camera_name="stereo_left"
        ).process(image, sequence=0)
        right, _ = UnderwaterCameraSensorModel(
            profile, camera_name="stereo_right"
        ).process(image, sequence=0)

        self.assertFalse(np.array_equal(left, right))

    def test_underwater_attenuation_creates_expected_red_loss(self) -> None:
        profile = replace(
            _ideal_profile(),
            optics=UnderwaterOpticsConfig(
                optical_path_length_m=2.0,
                attenuation_coefficients_rgb_per_m=(0.6, 0.2, 0.05),
            ),
        )
        image = np.full((12, 16, 3), 200, dtype=np.uint8)
        output, _ = UnderwaterCameraSensorModel(profile).process(image, sequence=0)
        means = output.mean(axis=(0, 1))

        self.assertLess(means[0], means[1])
        self.assertLess(means[1], means[2])

    def test_backscatter_adds_blue_green_veiling_light(self) -> None:
        profile = replace(
            _ideal_profile(),
            optics=UnderwaterOpticsConfig(
                optical_path_length_m=3.0,
                backscatter_coefficients_rgb_per_m=(0.1, 0.3, 0.5),
                veiling_light_rgb=(0.05, 0.4, 0.8),
                backscatter_strength=0.6,
            ),
        )
        output, _ = UnderwaterCameraSensorModel(profile).process(
            np.zeros((10, 10, 3), dtype=np.uint8),
            sequence=0,
        )
        means = output.mean(axis=(0, 1))

        self.assertLess(means[0], means[1])
        self.assertLess(means[1], means[2])

    def test_blur_spreads_impulse_and_vignette_darkens_corners(self) -> None:
        profile = replace(
            _ideal_profile(),
            optics=UnderwaterOpticsConfig(blur_radius_px=1, vignetting_strength=0.3),
        )
        image = np.full((21, 21, 3), 100, dtype=np.uint8)
        image[10, 10] = 255
        output, _ = UnderwaterCameraSensorModel(profile).process(image, sequence=0)

        self.assertLess(output[10, 10, 0], 255)
        self.assertGreater(output[10, 9, 0], 100)
        self.assertLess(output[0, 0, 0], output[10, 10, 0])

    def test_quantization_reduces_available_output_levels(self) -> None:
        profile = replace(
            _ideal_profile(),
            electronics=CameraElectronicsConfig(quantization_bits=3),
        )
        ramp = np.arange(256, dtype=np.uint8)[None, :, None]
        image = np.repeat(ramp, 3, axis=2)
        output, _ = UnderwaterCameraSensorModel(profile).process(image, sequence=0)

        self.assertLessEqual(np.unique(output).size, 8)

    def test_nonzero_plumb_bob_distortion_changes_checkerboard(self) -> None:
        base = _ideal_profile()
        calibration = CameraCalibration(
            width=64,
            height=48,
            distortion_model="plumb_bob",
            d=(-0.25, 0.08, 0.002, -0.001, 0.0),
            k=(45.0, 0.0, 31.5, 0.0, 45.0, 23.5, 0.0, 0.0, 1.0),
            r=(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
            p=(45.0, 0.0, 31.5, 0.0, 0.0, 45.0, 23.5, 0.0, 0.0, 0.0, 1.0, 0.0),
            calibration_status="test_calibration",
        )
        yy, xx = np.mgrid[0:48, 0:64]
        checker = (((xx // 4 + yy // 4) % 2) * 255).astype(np.uint8)
        image = np.repeat(checker[:, :, None], 3, axis=2)
        model = UnderwaterCameraSensorModel(base, calibration=calibration)

        output, diagnostics = model.process(image, sequence=0)

        self.assertTrue(diagnostics.distortion_applied)
        self.assertFalse(np.array_equal(output, image))
        self.assertEqual(output.shape, image.shape)


if __name__ == "__main__":
    unittest.main(verbosity=2)
