#!/usr/bin/env python3
"""Focused offline tests for the deterministic four-beam A50 model."""

from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest

import numpy as np


CURRENT = Path(__file__).resolve().parents[1]
if str(CURRENT) not in sys.path:
    sys.path.insert(0, str(CURRENT))

from bridge.dvl_a50_sensor_model import (  # noqa: E402
    A50SensorConfig,
    A50SensorModel,
    beam_directions_frd,
)


def _noise_free_config(**overrides) -> A50SensorConfig:
    values = {
        "white_noise_std_mps": 0.0,
        "velocity_noise_per_meter_mps": 0.0,
        "range_noise_std_m": 0.0,
        "range_noise_fraction": 0.0,
        "nsd_std_db": 0.0,
    }
    values.update(overrides)
    return A50SensorConfig(**values)


class A50BeamGeometryTest(unittest.TestCase):
    def test_published_a50_geometry_and_range_defaults(self) -> None:
        config = A50SensorConfig()

        self.assertEqual(config.beam_tilt_deg, 22.5)
        self.assertEqual(config.min_range_m, 0.05)
        self.assertEqual(config.max_range_m, 50.0)

    def test_four_beams_are_symmetric_unit_vectors(self) -> None:
        config = A50SensorConfig()
        directions = np.asarray(beam_directions_frd(config))

        self.assertEqual(directions.shape, (4, 3))
        np.testing.assert_allclose(np.linalg.norm(directions, axis=1), 1.0, atol=1.0e-12)
        np.testing.assert_allclose(directions[:, :2].sum(axis=0), 0.0, atol=1.0e-12)
        np.testing.assert_allclose(
            directions[:, 2],
            math.cos(math.radians(config.beam_tilt_deg)),
            atol=1.0e-12,
        )
        self.assertEqual(np.linalg.matrix_rank(directions), 3)

    def test_flat_bottom_sets_slant_range_and_incidence(self) -> None:
        config = _noise_free_config()
        model = A50SensorModel(config)
        sample = model.sample(
            (0.0, 0.0, 0.0),
            4.0,
            time_of_validity_us=1_000_000,
        )
        expected_incidence = math.cos(math.radians(config.beam_tilt_deg))
        expected_range = 4.0 / expected_incidence

        for beam in sample.beams:
            self.assertAlmostEqual(beam.true_range_m, expected_range, places=12)
            self.assertAlmostEqual(beam.measured_range_m, expected_range, places=12)
            self.assertAlmostEqual(beam.incidence_cosine, expected_incidence, places=12)
        self.assertAlmostEqual(sample.altitude_estimate_m, 4.0, places=12)


class A50VelocityModelTest(unittest.TestCase):
    def test_noise_free_beam_inversion_recovers_truth(self) -> None:
        model = A50SensorModel(_noise_free_config())
        truth = (0.42, -0.17, 0.08)
        sample = model.sample(truth, 3.0, time_of_validity_us=10)

        self.assertTrue(sample.velocity_valid)
        self.assertEqual(sample.valid_beam_count, 4)
        np.testing.assert_allclose(sample.measured_velocity_frd_mps, truth, atol=1.0e-11)
        np.testing.assert_allclose(sample.velocity_error_frd_mps, 0.0, atol=1.0e-11)
        for beam, direction in zip(sample.beams, model.directions_frd):
            self.assertAlmostEqual(
                beam.true_radial_velocity_mps,
                float(np.dot(np.asarray(truth), np.asarray(direction))),
                places=12,
            )

    def test_body_bias_projects_through_beams_and_reconstructs(self) -> None:
        bias = (0.012, -0.007, 0.004)
        model = A50SensorModel(
            _noise_free_config(velocity_bias_frd_mps=bias)
        )
        truth = np.asarray((0.3, -0.1, 0.02))
        sample = model.sample(truth, 2.0, time_of_validity_us=20)

        np.testing.assert_allclose(
            sample.measured_velocity_frd_mps,
            truth + np.asarray(bias),
            atol=1.0e-11,
        )
        np.testing.assert_allclose(sample.velocity_error_frd_mps, bias, atol=1.0e-11)

    def test_equal_seeds_produce_identical_sequences(self) -> None:
        config = A50SensorConfig(
            seed=91,
            beam_dropout_probabilities=(0.15, 0.25, 0.35, 0.45),
        )
        first = A50SensorModel(config)
        second = A50SensorModel(config)

        sequence_a = [
            first.sample((0.2, -0.1, 0.03), 5.0, time_of_validity_us=100 + index)
            for index in range(5)
        ]
        sequence_b = [
            second.sample((0.2, -0.1, 0.03), 5.0, time_of_validity_us=100 + index)
            for index in range(5)
        ]
        self.assertEqual(sequence_a, sequence_b)

        first.reset()
        replay = [
            first.sample((0.2, -0.1, 0.03), 5.0, time_of_validity_us=100 + index)
            for index in range(5)
        ]
        self.assertEqual(sequence_a, replay)

    def test_weighted_solution_uses_velocity_standard_deviation(self) -> None:
        model = A50SensorModel(
            A50SensorConfig(
                seed=73,
                white_noise_std_mps=0.004,
                velocity_noise_per_meter_mps=0.0005,
                range_noise_std_m=0.0,
                range_noise_fraction=0.0,
                nsd_std_db=8.0,
            )
        )
        sample = model.sample(
            (0.31, -0.12, 0.06),
            None,
            beam_ranges_m=(0.55, 1.5, 5.0, 12.0),
            incidence_cosines=(0.92, 0.82, 0.70, 0.55),
            time_of_validity_us=30,
        )

        self.assertTrue(sample.velocity_valid)
        design = np.asarray([beam.direction_frd for beam in sample.beams])
        radial = np.asarray(
            [beam.measured_radial_velocity_mps for beam in sample.beams]
        )
        velocity_std = np.asarray(
            [beam.velocity_std_mps for beam in sample.beams]
        )
        weights = 1.0 / np.square(velocity_std)
        information = design.T @ (weights[:, None] * design)
        expected = np.linalg.solve(information, design.T @ (weights * radial))

        self.assertGreater(float(np.ptp(velocity_std)), 0.0)
        np.testing.assert_allclose(
            sample.measured_velocity_frd_mps,
            expected,
            atol=1.0e-12,
        )


class A50QualityAndDropoutTest(unittest.TestCase):
    def test_official_example_scale_uses_negative_dbm_diagnostics(self) -> None:
        config = A50SensorConfig(seed=2608)
        sample = A50SensorModel(config).sample(
            (0.0, 0.0, 0.0),
            None,
            # Beam ranges from Water Linked's published JSON example.
            beam_ranges_m=(0.5568, 0.5664, 0.5376, 0.5472),
            incidence_cosines=(0.924, 0.924, 0.924, 0.924),
            time_of_validity_us=1,
        )

        rssi_dbm = np.asarray([beam.rssi_dbm for beam in sample.beams])
        nsd_dbm = np.asarray([beam.nsd_dbm for beam in sample.beams])
        velocity_std = np.asarray(
            [beam.velocity_std_mps for beam in sample.beams]
        )

        self.assertTrue(np.all((-120.0 <= rssi_dbm) & (rssi_dbm <= -10.0)))
        self.assertTrue(np.all((-120.0 <= nsd_dbm) & (nsd_dbm <= -60.0)))
        self.assertTrue(np.all(rssi_dbm < 0.0))
        self.assertTrue(np.all(nsd_dbm < 0.0))
        self.assertTrue(np.all(velocity_std >= 0.0))
        self.assertGreater(float(np.mean(rssi_dbm)), -35.0)
        self.assertLess(float(np.mean(rssi_dbm)), -25.0)
        self.assertGreater(float(np.mean(nsd_dbm)), -100.0)
        self.assertLess(float(np.mean(nsd_dbm)), -88.0)

    def test_range_degradation_links_rssi_velocity_std_fom_and_error(self) -> None:
        config = A50SensorConfig(
            seed=44,
            white_noise_std_mps=0.003,
            velocity_noise_per_meter_mps=0.0002,
        )
        near = A50SensorModel(config).sample(
            (0.4, -0.2, 0.1),
            1.0,
            time_of_validity_us=1,
        )
        far = A50SensorModel(config).sample(
            (0.4, -0.2, 0.1),
            30.0,
            time_of_validity_us=1,
        )

        self.assertTrue(near.velocity_valid)
        self.assertTrue(far.velocity_valid)
        self.assertLess(
            sum(beam.rssi_dbm for beam in far.beams) / 4.0,
            sum(beam.rssi_dbm for beam in near.beams) / 4.0,
        )
        self.assertGreater(
            sum(beam.velocity_std_mps for beam in far.beams) / 4.0,
            sum(beam.velocity_std_mps for beam in near.beams) / 4.0,
        )
        np.testing.assert_allclose(
            [beam.nsd_dbm for beam in far.beams],
            [beam.nsd_dbm for beam in near.beams],
            atol=0.0,
        )
        self.assertGreater(far.fom_mps, near.fom_mps)
        self.assertGreater(
            np.linalg.norm(far.velocity_error_frd_mps),
            np.linalg.norm(near.velocity_error_frd_mps),
        )

    def test_explicit_incidence_controls_quality_and_validity(self) -> None:
        model = A50SensorModel(_noise_free_config(min_incidence_cosine=0.2))
        sample = model.sample(
            (0.1, 0.0, 0.0),
            None,
            beam_ranges_m=(2.0, 2.0, 2.0, 2.0),
            incidence_cosines=(0.95, 0.60, 0.30, 0.10),
            time_of_validity_us=2,
        )

        self.assertTrue(sample.velocity_valid)
        self.assertEqual(sample.valid_beam_count, 3)
        self.assertGreater(sample.beams[0].quality, sample.beams[1].quality)
        self.assertGreater(sample.beams[1].quality, sample.beams[2].quality)
        self.assertFalse(sample.beams[3].valid)
        self.assertEqual(sample.beams[3].dropout_reason, "incidence")

    def test_minimum_valid_beams_controls_solution(self) -> None:
        config = _noise_free_config(min_valid_beams=3)
        one_missing = A50SensorModel(config).sample(
            (0.2, 0.1, -0.05),
            3.0,
            forced_dropout_beams={0},
            time_of_validity_us=3,
        )
        two_missing = A50SensorModel(config).sample(
            (0.2, 0.1, -0.05),
            3.0,
            forced_dropout_beams={0, 1},
            time_of_validity_us=3,
        )

        self.assertTrue(one_missing.velocity_valid)
        self.assertEqual(one_missing.valid_beam_count, 3)
        np.testing.assert_allclose(
            one_missing.measured_velocity_frd_mps,
            (0.2, 0.1, -0.05),
            atol=1.0e-11,
        )
        self.assertFalse(two_missing.velocity_valid)
        self.assertEqual(two_missing.valid_beam_count, 2)
        self.assertIsNone(two_missing.measured_velocity_frd_mps)
        self.assertIsNone(two_missing.covariance_frd_mps2)
        self.assertTrue(math.isinf(two_missing.fom_mps))

        four_required = A50SensorModel(
            _noise_free_config(min_valid_beams=4)
        ).sample(
            (0.2, 0.1, -0.05),
            3.0,
            forced_dropout_beams={0},
            time_of_validity_us=3,
        )
        self.assertEqual(four_required.valid_beam_count, 3)
        self.assertFalse(four_required.velocity_valid)
        self.assertIsNone(four_required.measured_velocity_frd_mps)

    def test_probability_one_marks_random_dropout(self) -> None:
        config = _noise_free_config(
            beam_dropout_probabilities=(1.0, 0.0, 0.0, 0.0),
        )
        sample = A50SensorModel(config).sample(
            (0.0, 0.0, 0.0),
            2.0,
            time_of_validity_us=4,
        )
        self.assertEqual(sample.beams[0].dropout_reason, "random")
        self.assertEqual(sample.valid_beam_count, 3)
        self.assertTrue(sample.velocity_valid)


class A50TimingAndValidationTest(unittest.TestCase):
    def test_validity_and_transmission_times_are_distinct(self) -> None:
        model = A50SensorModel(_noise_free_config(transmission_delay_us=2_500))
        delayed = model.sample(
            (0.0, 0.0, 0.0),
            2.0,
            time_of_validity_us=1_000_000,
        )
        explicit = model.sample(
            (0.0, 0.0, 0.0),
            2.0,
            time_of_validity_us=2_000_000,
            time_of_transmission_us=2_009_000,
        )

        self.assertEqual(delayed.time_of_validity_us, 1_000_000)
        self.assertEqual(delayed.time_of_transmission_us, 1_002_500)
        self.assertEqual(explicit.time_of_validity_us, 2_000_000)
        self.assertEqual(explicit.time_of_transmission_us, 2_009_000)
        self.assertEqual((delayed.sample_index, explicit.sample_index), (0, 1))

    def test_invalid_time_and_config_are_rejected(self) -> None:
        model = A50SensorModel(_noise_free_config())
        with self.assertRaises(ValueError):
            model.sample(
                (0.0, 0.0, 0.0),
                1.0,
                time_of_validity_us=100,
                time_of_transmission_us=99,
            )
        with self.assertRaises(ValueError):
            A50SensorConfig(min_valid_beams=2)
        with self.assertRaises(ValueError):
            A50SensorConfig(beam_dropout_probabilities=(0.0, 0.0, 0.0, 1.1))
        with self.assertRaises(ValueError):
            A50SensorConfig(rssi_at_1m_dbm=0.0)
        with self.assertRaises(ValueError):
            A50SensorConfig(nsd_min_dbm=-50.0, nsd_max_dbm=-60.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
