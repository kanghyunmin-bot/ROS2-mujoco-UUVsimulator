#!/usr/bin/env python3
"""Focused offline tests for the IMU and Bar30 measurement models."""

from __future__ import annotations

from dataclasses import replace
import json
from pathlib import Path
import sys
import unittest

import numpy as np


CURRENT = Path(__file__).resolve().parents[1]
if str(CURRENT) not in sys.path:
    sys.path.insert(0, str(CURRENT))

from bridge.bar30_sensor_model import Bar30SensorConfig, Bar30SensorModel  # noqa: E402
from bridge.imu_sensor_model import ImuSensorConfig, ImuSensorModel  # noqa: E402
from bridge.ros2_imu_bar30_sensor_runtime import (  # noqa: E402
    DEFAULT_IMU_BAR30_SENSOR_CONFIG_PATH,
)


IDENTITY = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
ZERO3 = (0.0, 0.0, 0.0)


def noise_free_imu_config(**overrides) -> ImuSensorConfig:
    values = dict(
        gyro_scale_cross_axis_matrix=IDENTITY,
        accel_scale_cross_axis_matrix=IDENTITY,
        gyro_constant_bias_rad_s=ZERO3,
        gyro_turn_on_bias_std_rad_s=ZERO3,
        gyro_bias_instability_std_rad_s=ZERO3,
        gyro_random_walk_rad_s_per_sqrt_s=ZERO3,
        gyro_white_noise_density_rad_s_sqrt_hz=ZERO3,
        gyro_saturation_rad_s=(1000.0, 1000.0, 1000.0),
        gyro_quantization_rad_s=ZERO3,
        accel_constant_bias_mps2=ZERO3,
        accel_turn_on_bias_std_mps2=ZERO3,
        accel_bias_instability_std_mps2=ZERO3,
        accel_random_walk_mps2_per_sqrt_s=ZERO3,
        accel_white_noise_density_mps2_sqrt_hz=ZERO3,
        accel_saturation_mps2=(1000.0, 1000.0, 1000.0),
        accel_quantization_mps2=ZERO3,
        attitude_turn_on_bias_std_rad=ZERO3,
        attitude_white_noise_std_rad=ZERO3,
    )
    values.update(overrides)
    return ImuSensorConfig(**values)


def noise_free_bar30_config(**overrides) -> Bar30SensorConfig:
    values = dict(
        pressure_scale_factor=1.0,
        constant_offset_pa=0.0,
        turn_on_offset_std_pa=0.0,
        bias_instability_std_pa=0.0,
        random_walk_pa_per_sqrt_s=0.0,
        white_noise_std_pa=0.0,
        temperature_coefficient_pa_per_c=0.0,
        min_pressure_pa=0.0,
        max_pressure_pa=3_000_000.0,
        quantization_pa=0.0,
    )
    values.update(overrides)
    return Bar30SensorConfig(**values)


class PriorProvenanceTest(unittest.TestCase):
    def test_default_prior_is_explicitly_uncalibrated(self) -> None:
        data = json.loads(
            DEFAULT_IMU_BAR30_SENSOR_CONFIG_PATH.read_text(encoding="utf-8")
        )
        self.assertEqual(data["schema"], "uuv_mujoco.sensor_model.imu_bar30.v1")
        self.assertEqual(data["calibration_status"], "unvalidated_prior")
        self.assertIsNone(data["evidence"]["hardware_resolution"]["fcu_imu"]["model"])
        self.assertEqual(
            data["evidence"]["hardware_resolution"]["pressure_sensor"]["model"],
            "Blue Robotics Bar30 / TE Connectivity MS5837-30BA",
        )
        self.assertIn("engineering priors", data["evidence"]["hardware_resolution"]["fcu_imu"]["basis"])
        self.assertGreaterEqual(len(data["evidence"]["calibration_required"]), 5)


class ImuSensorModelTest(unittest.TestCase):
    def test_noise_free_model_preserves_truth(self) -> None:
        model = ImuSensorModel(noise_free_imu_config())
        sample = model.sample(
            (1.0, 0.0, 0.0, 0.0),
            (0.1, -0.2, 0.3),
            (1.0, 2.0, 9.8),
            sample_time_s=0.0,
        )
        np.testing.assert_allclose(sample.orientation_wxyz, (1.0, 0.0, 0.0, 0.0))
        np.testing.assert_allclose(sample.angular_velocity_rad_s, (0.1, -0.2, 0.3))
        np.testing.assert_allclose(sample.linear_acceleration_mps2, (1.0, 2.0, 9.8))

    def test_cross_axis_matrix_couples_axes_before_noise(self) -> None:
        coupling = (1.0, 0.1, 0.0, 0.0, 1.0, 0.2, 0.3, 0.0, 1.0)
        model = ImuSensorModel(
            noise_free_imu_config(
                gyro_scale_cross_axis_matrix=coupling,
                accel_scale_cross_axis_matrix=coupling,
            )
        )
        sample = model.sample(
            (1.0, 0.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 2.0),
            sample_time_s=0.0,
        )
        np.testing.assert_allclose(sample.angular_velocity_rad_s, (0.1, 1.0, 0.0))
        np.testing.assert_allclose(sample.linear_acceleration_mps2, (0.0, 0.4, 2.0))

    def test_saturation_precedes_quantization(self) -> None:
        config = noise_free_imu_config(
            gyro_saturation_rad_s=(1.0, 1.0, 1.0),
            gyro_quantization_rad_s=(0.25, 0.25, 0.25),
            accel_saturation_mps2=(2.0, 2.0, 2.0),
            accel_quantization_mps2=(0.5, 0.5, 0.5),
        )
        sample = ImuSensorModel(config).sample(
            (1.0, 0.0, 0.0, 0.0),
            (1.4, -0.62, 0.12),
            (2.3, -1.26, 0.24),
            sample_time_s=0.0,
        )
        self.assertEqual(sample.angular_velocity_rad_s, (1.0, -0.5, 0.0))
        self.assertEqual(sample.linear_acceleration_mps2, (2.0, -1.5, 0.0))
        self.assertEqual(sample.gyro_saturated, (True, False, False))
        self.assertEqual(sample.accel_saturated, (True, False, False))

    def test_seeded_bias_white_noise_and_random_walk_replay(self) -> None:
        config = ImuSensorConfig(seed=44)
        model = ImuSensorModel(config)

        def run() -> list[object]:
            return [
                model.sample(
                    (1.0, 0.0, 0.0, 0.0),
                    (0.1, 0.2, 0.3),
                    (0.0, 0.0, 9.8),
                    sample_time_s=time_s,
                )
                for time_s in (0.0, 0.02, 0.04, 0.08)
            ]

        first = run()
        model.reset(44)
        second = run()
        self.assertEqual(first, second)
        self.assertNotEqual(first[0].gyro_bias_rad_s, first[-1].gyro_bias_rad_s)

    def test_white_noise_covariance_scales_with_sample_bandwidth(self) -> None:
        config = noise_free_imu_config(
            gyro_white_noise_density_rad_s_sqrt_hz=(0.02, 0.02, 0.02),
            accel_white_noise_density_mps2_sqrt_hz=(0.2, 0.2, 0.2),
        )
        sample = ImuSensorModel(config).sample(
            (1.0, 0.0, 0.0, 0.0), ZERO3, ZERO3, sample_time_s=0.0
        )
        self.assertAlmostEqual(sample.angular_velocity_covariance_diag_rad2_s2[0], 0.01)
        self.assertAlmostEqual(sample.linear_acceleration_covariance_diag_m2_s4[0], 1.0)

    def test_invalid_singular_coupling_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            replace(ImuSensorConfig(), gyro_scale_cross_axis_matrix=(0.0,) * 9)


class Bar30SensorModelTest(unittest.TestCase):
    def test_scale_error_applies_to_gauge_not_absolute_pressure(self) -> None:
        config = noise_free_bar30_config(
            reference_pressure_pa=100_000.0,
            pressure_scale_factor=1.01,
        )
        sample = Bar30SensorModel(config).sample(110_000.0, sample_time_s=0.0)
        self.assertEqual(sample.measured_pressure_pa, 110_100.0)

    def test_temperature_response_is_first_order_and_optional(self) -> None:
        config = noise_free_bar30_config(
            reference_temperature_c=20.0,
            default_temperature_c=20.0,
            temperature_coefficient_pa_per_c=10.0,
            temperature_time_constant_s=1.0,
            nominal_rate_hz=1.0,
        )
        sample = Bar30SensorModel(config).sample(
            100_000.0,
            sample_time_s=0.0,
            ambient_temperature_c=30.0,
        )
        expected_temperature = 20.0 + (1.0 - np.exp(-1.0)) * 10.0
        self.assertAlmostEqual(sample.sensor_temperature_c, expected_temperature)
        self.assertAlmostEqual(
            sample.measured_pressure_pa,
            100_000.0 + 10.0 * (expected_temperature - 20.0),
        )

    def test_pressure_saturation_and_resolution(self) -> None:
        config = noise_free_bar30_config(
            max_pressure_pa=1_000.0,
            quantization_pa=20.0,
        )
        model = Bar30SensorModel(config)
        high = model.sample(1_015.0, sample_time_s=0.0)
        self.assertTrue(high.saturated)
        self.assertEqual(high.measured_pressure_pa, 1_000.0)
        model.reset()
        quantized = model.sample(553.0, sample_time_s=0.0)
        self.assertFalse(quantized.saturated)
        self.assertEqual(quantized.measured_pressure_pa, 560.0)

    def test_seeded_offset_drift_and_noise_replay(self) -> None:
        model = Bar30SensorModel(Bar30SensorConfig(seed=81))

        def run() -> list[object]:
            return [
                model.sample(120_000.0, sample_time_s=time_s)
                for time_s in (0.0, 0.1, 0.2, 1.0)
            ]

        first = run()
        model.reset(81)
        self.assertEqual(first, run())
        self.assertNotEqual(first[0].total_bias_pa, first[-1].total_bias_pa)


if __name__ == "__main__":
    unittest.main(verbosity=2)
