#!/usr/bin/env python3
"""Focused unit checks for full 6x6 Fossen hydrodynamics."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim.physics.full_matrix_hydrodynamics import (  # noqa: E402
    FullMatrixHydrodynamics,
    added_mass_coriolis_matrix,
)


def _profile(
    *,
    added_mass: np.ndarray | None = None,
    linear_damping: np.ndarray | None = None,
    quadratic_damping: np.ndarray | None = None,
) -> dict:
    return {
        "hydrodynamic_matrices": {
            "active": True,
            "calibration_status": "unit_test_prior",
            "provenance": "synthetic unit-test coefficients",
            "added_mass_6x6": (
                np.diag([2.0, 3.0, 4.0, 0.5, 0.6, 0.7])
                if added_mass is None
                else added_mass
            ),
            "linear_damping_6x6": (
                np.diag([5.0, 6.0, 7.0, 0.8, 0.9, 1.0])
                if linear_damping is None
                else linear_damping
            ),
            "quadratic_damping_6x6": (
                np.diag([8.0, 9.0, 10.0, 1.1, 1.2, 1.3])
                if quadratic_damping is None
                else quadratic_damping
            ),
        }
    }


class FullMatrixHydrodynamicsTests(unittest.TestCase):
    def test_missing_or_disabled_model_returns_exact_zero(self) -> None:
        profiles = (
            {},
            {
                "hydrodynamic_matrices": {
                    "active": False,
                    "added_mass_6x6": "ignored while disabled",
                }
            },
        )
        for profile in profiles:
            model = FullMatrixHydrodynamics.from_profile(profile)
            self.assertFalse(model.active)
            np.testing.assert_array_equal(
                model.wrench_body([float("nan")], [float("nan")]),
                np.zeros(6),
            )
            np.testing.assert_array_equal(
                model.added_mass_coriolis_body([float("nan")]),
                np.zeros((6, 6)),
            )

    def test_diagonal_model_agrees_with_explicit_fossen_terms(self) -> None:
        model = FullMatrixHydrodynamics.from_profile(_profile())
        velocity = np.array([0.4, -0.3, 0.2, 0.1, -0.05, 0.08])
        acceleration = np.array([0.7, -0.2, 0.4, -0.1, 0.06, 0.03])
        cfg = model.config

        momentum = cfg.added_mass_6x6 @ velocity
        skew_linear = _skew(momentum[:3])
        skew_angular = _skew(momentum[3:])
        expected_coriolis = np.block(
            [
                [np.zeros((3, 3)), -skew_linear],
                [-skew_linear, -skew_angular],
            ]
        )
        expected = -(
            cfg.added_mass_6x6 @ acceleration
            + expected_coriolis @ velocity
            + cfg.linear_damping_6x6 @ velocity
            + cfg.quadratic_damping_6x6 @ (np.abs(velocity) * velocity)
        )

        np.testing.assert_allclose(
            model.added_mass_coriolis_body(velocity),
            expected_coriolis,
            atol=1.0e-14,
        )
        np.testing.assert_allclose(
            model.wrench_body(velocity, acceleration),
            expected,
            atol=1.0e-14,
        )

    def test_coriolis_is_skew_symmetric_and_does_zero_power(self) -> None:
        generator = np.array(
            [
                [1.2, -0.2, 0.1, 0.0, 0.3, 0.0],
                [0.1, 1.1, 0.2, -0.1, 0.0, 0.1],
                [0.0, 0.2, 1.3, 0.1, -0.2, 0.0],
                [0.1, 0.0, 0.2, 0.8, 0.1, -0.1],
                [-0.2, 0.1, 0.0, 0.2, 0.9, 0.1],
                [0.0, -0.1, 0.1, 0.0, 0.2, 0.7],
            ]
        )
        added_mass = generator.T @ generator
        velocity = np.array([0.8, -0.5, 0.3, -0.2, 0.4, 0.1])

        coriolis = added_mass_coriolis_matrix(added_mass, velocity)

        np.testing.assert_allclose(coriolis + coriolis.T, np.zeros((6, 6)), atol=1.0e-14)
        self.assertAlmostEqual(float(velocity @ coriolis @ velocity), 0.0, places=14)

    def test_cross_coupled_matrices_create_off_axis_wrench(self) -> None:
        added_mass = np.eye(6)
        added_mass[0, 4] = added_mass[4, 0] = 0.25
        linear = np.eye(6)
        linear[0, 1] = linear[1, 0] = 0.2
        quadratic = np.eye(6)
        quadratic[2, 5] = quadratic[5, 2] = 0.3
        model = FullMatrixHydrodynamics.from_profile(
            _profile(
                added_mass=added_mass,
                linear_damping=linear,
                quadratic_damping=quadratic,
            )
        )

        acceleration_only = model.wrench_body(
            np.zeros(6),
            np.array([0.0, 0.0, 0.0, 0.0, 2.0, 0.0]),
        )
        self.assertAlmostEqual(acceleration_only[0], -0.5)
        self.assertAlmostEqual(acceleration_only[4], -2.0)

        damping_only = model.wrench_body(
            np.array([0.0, 2.0, 0.0, 0.0, 0.0, 0.0]),
            np.zeros(6),
        )
        self.assertAlmostEqual(damping_only[0], -0.4)
        self.assertAlmostEqual(damping_only[1], -6.0)

        quadratic_velocity = np.array([0.0, 0.0, 2.0, 0.0, 0.0, 0.5])
        quadratic_only = model.wrench_body(
            quadratic_velocity,
            np.zeros(6),
        )
        eigenvalues, eigenvectors = np.linalg.eigh(quadratic)
        modal_velocity = eigenvectors.T @ quadratic_velocity
        expected_quadratic = -(
            model.added_mass_coriolis_body(quadratic_velocity) @ quadratic_velocity
            + linear @ quadratic_velocity
            + eigenvectors
            @ (np.maximum(eigenvalues, 0.0) * np.abs(modal_velocity) * modal_velocity)
        )
        np.testing.assert_allclose(quadratic_only, expected_quadratic)

        pure_heave = model.wrench_body(
            np.array([0.0, 0.0, 2.0, 0.0, 0.0, 0.0]),
            np.zeros(6),
        )
        self.assertNotEqual(pure_heave[5], 0.0)

    def test_rejects_nonfinite_nonsymmetric_and_nonpassive_matrices(self) -> None:
        nonfinite = np.eye(6)
        nonfinite[2, 2] = np.inf
        with self.assertRaisesRegex(ValueError, "finite 6x6"):
            FullMatrixHydrodynamics.from_profile(_profile(added_mass=nonfinite))

        nonsymmetric = np.eye(6)
        nonsymmetric[0, 1] = 0.1
        with self.assertRaisesRegex(ValueError, "symmetric"):
            FullMatrixHydrodynamics.from_profile(_profile(added_mass=nonsymmetric))

        indefinite = np.eye(6)
        indefinite[3, 3] = -0.01
        with self.assertRaisesRegex(ValueError, "positive semidefinite"):
            FullMatrixHydrodynamics.from_profile(_profile(added_mass=indefinite))

        indefinite_damping = np.eye(6)
        indefinite_damping[0, 1] = indefinite_damping[1, 0] = 1.1
        with self.assertRaisesRegex(ValueError, "positive semidefinite"):
            FullMatrixHydrodynamics.from_profile(
                _profile(linear_damping=indefinite_damping)
            )

    def test_passive_damping_allows_signed_off_diagonal_coupling(self) -> None:
        damping = np.eye(6)
        damping[0, 1] = damping[1, 0] = -0.4

        model = FullMatrixHydrodynamics.from_profile(
            _profile(linear_damping=damping)
        )

        np.testing.assert_array_equal(model.config.linear_damping_6x6, damping)
        velocity = np.array([0.7, -0.2, 0.0, 0.0, 0.0, 0.0])
        dissipated_power = float(
            velocity @ model.config.linear_damping_6x6 @ velocity
        )
        self.assertGreaterEqual(dissipated_power, 0.0)

        quadratic_model = FullMatrixHydrodynamics.from_profile(
            _profile(quadratic_damping=damping)
        )
        rng = np.random.default_rng(260901)
        for _index in range(100):
            velocity = rng.normal(size=6)
            wrench = quadratic_model.wrench_body(velocity, np.zeros(6))
            coriolis = quadratic_model.added_mass_coriolis_body(velocity)
            damping_wrench = wrench + coriolis @ velocity
            self.assertLessEqual(float(velocity @ damping_wrench), 1.0e-12)

    def test_rejects_huge_finite_matrices_and_state_components(self) -> None:
        huge = np.eye(6) * 1.0e300
        with self.assertRaisesRegex(ValueError, "safe absolute-entry bound"):
            FullMatrixHydrodynamics.from_profile(_profile(added_mass=huge))

        model = FullMatrixHydrodynamics.from_profile(_profile())
        with self.assertRaisesRegex(ValueError, "safe component bound"):
            model.wrench_body(
                np.array([1.0e100, 0.0, 0.0, 0.0, 0.0, 0.0]),
                np.zeros(6),
            )

    def test_wrench_safety_bound_is_finite(self) -> None:
        profile = _profile()
        profile["hydrodynamic_matrices"]["max_force_n"] = 20.0
        profile["hydrodynamic_matrices"]["max_torque_nm"] = 10.0
        model = FullMatrixHydrodynamics.from_profile(profile)
        wrench = model.wrench_body(
            np.full(6, 1000.0),
            np.full(6, 1000.0),
        )
        self.assertTrue(np.all(np.isfinite(wrench)))
        self.assertLessEqual(float(np.linalg.norm(wrench[:3])), 20.0 + 1.0e-12)
        self.assertLessEqual(float(np.linalg.norm(wrench[3:])), 10.0 + 1.0e-12)


def _skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = vector
    return np.array(
        [
            [0.0, -z, y],
            [z, 0.0, -x],
            [-y, x, 0.0],
        ]
    )


if __name__ == "__main__":
    unittest.main(verbosity=2)
