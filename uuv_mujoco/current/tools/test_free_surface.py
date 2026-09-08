#!/usr/bin/env python3
"""Focused unit checks for the shared deterministic free-surface contract."""

from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim.physics.free_surface import (  # noqa: E402
    FreeSurface,
    MAX_COMPONENT_AMPLITUDE_M,
    MAX_FREQUENCY_HZ,
    MAX_WAVENUMBER_RAD_PER_M,
    MIN_WATER_DEPTH_M,
    MIN_WAVENUMBER_RAD_PER_M,
)


def _active_base(mode: str) -> dict:
    return {
        "mode": mode,
        "reference_height_world_m": 1.25,
        "calibration_status": "unit_test_prior",
        "provenance": "tools/test_free_surface.py analytic fixture",
    }


class FreeSurfaceConfigTests(unittest.TestCase):
    def test_missing_and_explicit_disabled_are_exact_no_effect(self) -> None:
        surfaces = (
            FreeSurface.from_profile({}),
            FreeSurface.from_mapping(
                {
                    "mode": "disabled",
                    "reference_height_world_m": 99.0,
                    "harmonics": "intentionally ignored while disabled",
                }
            ),
        )
        for surface in surfaces:
            self.assertFalse(surface.active)
            self.assertEqual(surface.mode, "disabled")
            sample = surface.sample(15.0, -8.0, -30.0, 1.0e6)
            self.assertEqual(sample.height_world_m, 0.0)
            np.testing.assert_array_equal(sample.normal_world, [0.0, 0.0, 1.0])
            np.testing.assert_array_equal(sample.orbital_velocity_world_mps, np.zeros(3))

    def test_active_modes_require_status_and_provenance(self) -> None:
        with self.assertRaisesRegex(ValueError, "calibration_status"):
            FreeSurface.from_mapping({"mode": "flat", "provenance": "fixture"})
        with self.assertRaisesRegex(ValueError, "provenance"):
            FreeSurface.from_mapping(
                {"mode": "flat", "calibration_status": "unit_test_prior"}
            )

    def test_flat_surface_is_exact_at_every_query(self) -> None:
        surface = FreeSurface.from_mapping(_active_base("flat"))
        self.assertTrue(surface.active)
        self.assertEqual(surface.mode, "flat")
        for x, y, z, time_s in ((0.0, 0.0, 0.0, 0.0), (1e4, -2e4, 99.0, 1e6)):
            sample = surface.sample(x, y, z, time_s)
            self.assertEqual(sample.height_world_m, 1.25)
            np.testing.assert_array_equal(sample.normal_world, [0.0, 0.0, 1.0])
            np.testing.assert_array_equal(sample.orbital_velocity_world_mps, np.zeros(3))

    def test_runtime_waterline_override_replaces_profile_reference(self) -> None:
        surface = FreeSurface.from_profile(
            {"free_surface": _active_base("flat")},
            reference_height_world_m=-0.37,
        )
        self.assertEqual(surface.height_world_m(0.0, 0.0, 0.0), -0.37)

    def test_flat_rejects_dead_harmonic_configuration(self) -> None:
        config = _active_base("flat")
        config["harmonics"] = [{}]
        with self.assertRaisesRegex(ValueError, "flat.*cannot define harmonics"):
            FreeSurface.from_mapping(config)

    def test_harmonic_requires_at_least_one_component(self) -> None:
        with self.assertRaisesRegex(ValueError, "at least one"):
            FreeSurface.from_mapping(_active_base("harmonic"))

    def test_component_bounds_are_enforced(self) -> None:
        cases = (
            ("amplitude_m", MAX_COMPONENT_AMPLITUDE_M + 0.01, "amplitude_m"),
            ("frequency_hz", MAX_FREQUENCY_HZ + 0.01, "frequency_hz"),
            (
                "wave_vector_rad_per_m",
                [MAX_WAVENUMBER_RAD_PER_M + 0.01, 0.0],
                "wavenumber",
            ),
        )
        for field, invalid_value, pattern in cases:
            config = _active_base("harmonic")
            harmonic = {
                "amplitude_m": 0.1,
                "frequency_hz": 0.25,
                "wave_vector_rad_per_m": [1.0, 0.0],
            }
            harmonic[field] = invalid_value
            config["harmonics"] = [harmonic]
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, pattern):
                FreeSurface.from_mapping(config)


class HarmonicFreeSurfaceTests(unittest.TestCase):
    def setUp(self) -> None:
        config = _active_base("harmonic")
        config["reference_height_world_m"] = 0.0
        config["harmonics"] = [
            {
                "amplitude_m": 0.2,
                "frequency_hz": 0.5,
                "wave_vector_rad_per_m": [math.pi / 2.0, 0.0],
                "phase_rad": 0.0,
            }
        ]
        self.surface = FreeSurface.from_mapping(config)

    def test_height_normal_and_orbital_velocity_match_linear_wave(self) -> None:
        omega = math.pi

        crest = self.surface.sample(0.0, 0.0, 0.0, 0.0)
        self.assertAlmostEqual(crest.height_world_m, 0.2, places=15)
        np.testing.assert_allclose(crest.normal_world, [0.0, 0.0, 1.0], atol=1e-15)
        np.testing.assert_allclose(
            crest.orbital_velocity_world_mps,
            [0.2 * omega, 0.0, 0.0],
            atol=1e-15,
        )

        quarter_wave = self.surface.sample(1.0, 0.0, 0.0, 0.0)
        expected_normal = np.array([0.1 * math.pi, 0.0, 1.0])
        expected_normal /= np.linalg.norm(expected_normal)
        self.assertAlmostEqual(quarter_wave.height_world_m, 0.0, places=15)
        np.testing.assert_allclose(quarter_wave.normal_world, expected_normal, atol=1e-15)
        np.testing.assert_allclose(
            quarter_wave.orbital_velocity_world_mps,
            [0.0, 0.0, 0.2 * omega],
            atol=1e-15,
        )
        self.assertAlmostEqual(float(np.linalg.norm(quarter_wave.normal_world)), 1.0, places=15)

    def test_orbital_motion_decays_with_depth_and_is_zero_in_air(self) -> None:
        surface_speed = float(np.linalg.norm(self.surface.orbital_velocity_world_mps(0, 0, 0, 0)))
        deep_speed = float(np.linalg.norm(self.surface.orbital_velocity_world_mps(0, 0, -2, 0)))
        self.assertAlmostEqual(
            deep_speed / surface_speed,
            math.exp(-math.pi),
            places=15,
        )
        np.testing.assert_array_equal(
            self.surface.orbital_velocity_world_mps(0, 0, 0.200001, 0),
            np.zeros(3),
        )

    def test_same_query_is_bitwise_deterministic(self) -> None:
        first = self.surface.sample(0.31, -0.27, -0.8, 123.5)
        second = self.surface.sample(0.31, -0.27, -0.8, 123.5)
        self.assertEqual(first.height_world_m, second.height_world_m)
        np.testing.assert_array_equal(first.normal_world, second.normal_world)
        np.testing.assert_array_equal(
            first.orbital_velocity_world_mps,
            second.orbital_velocity_world_mps,
        )

    def test_orbital_speed_is_norm_bounded(self) -> None:
        config = _active_base("harmonic")
        config["max_orbital_speed_mps"] = 0.05
        config["harmonics"] = [
            {
                "amplitude_m": 0.5,
                "frequency_hz": 1.0,
                "wave_vector_rad_per_m": [1.0, 0.0],
            }
        ]
        surface = FreeSurface.from_mapping(config)
        velocity = surface.orbital_velocity_world_mps(0.0, 0.0, 1.25, 0.0)
        self.assertAlmostEqual(float(np.linalg.norm(velocity)), 0.05, places=15)

    def test_finite_depth_gravity_mode_derives_dispersion_and_bottom_motion(self) -> None:
        config = _active_base("harmonic")
        config.update(
            {
                "reference_height_world_m": 0.0,
                "wave_kinematics": "finite_depth_gravity",
                "water_depth_m": 3.0,
                "gravity_mps2": 9.81,
                "harmonics": [
                    {
                        "amplitude_m": 0.1,
                        "wave_vector_rad_per_m": [1.0, 0.0],
                    }
                ],
            }
        )
        surface = FreeSurface.from_mapping(config)
        expected_frequency = math.sqrt(9.81 * math.tanh(3.0)) / (2.0 * math.pi)
        self.assertAlmostEqual(
            surface.config.harmonics[0].frequency_hz,
            expected_frequency,
            places=15,
        )
        quarter_time = 0.25 / expected_frequency
        bottom_velocity = surface.orbital_velocity_world_mps(
            0.0,
            0.0,
            -3.0,
            quarter_time,
        )
        self.assertAlmostEqual(bottom_velocity[2], 0.0, places=15)

    def test_finite_depth_mode_rejects_inconsistent_frequency(self) -> None:
        config = _active_base("harmonic")
        config.update(
            {
                "wave_kinematics": "finite_depth_gravity",
                "water_depth_m": 3.0,
                "harmonics": [
                    {
                        "amplitude_m": 0.1,
                        "frequency_hz": 0.1,
                        "wave_vector_rad_per_m": [1.0, 0.0],
                    }
                ],
            }
        )
        with self.assertRaisesRegex(ValueError, "violates.*dispersion"):
            FreeSurface.from_mapping(config)

    def test_finite_depth_mode_rejects_numerically_singular_geometry(self) -> None:
        for field, value, pattern in (
            ("water_depth_m", np.nextafter(0.0, 1.0), "water_depth_m"),
            (
                "wave_vector_rad_per_m",
                [0.5 * MIN_WAVENUMBER_RAD_PER_M, 0.0],
                "wavenumber",
            ),
        ):
            config = _active_base("harmonic")
            config.update(
                {
                    "wave_kinematics": "finite_depth_gravity",
                    "water_depth_m": MIN_WATER_DEPTH_M,
                    "harmonics": [
                        {
                            "amplitude_m": 0.1,
                            "wave_vector_rad_per_m": [1.0, 0.0],
                        }
                    ],
                }
            )
            if field == "water_depth_m":
                config[field] = value
            else:
                config["harmonics"][0][field] = value
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, pattern):
                FreeSurface.from_mapping(config)


if __name__ == "__main__":
    unittest.main(verbosity=2)
