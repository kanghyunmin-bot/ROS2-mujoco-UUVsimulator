#!/usr/bin/env python3
"""Focused unit checks for research-pool current and coefficient models."""

from __future__ import annotations

from contextlib import redirect_stdout
import io
import json
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim.physics.current_field import DeterministicCurrentField  # noqa: E402
from sim.physics.current_field_runtime import CurrentFieldRuntime  # noqa: E402
from sim.physics.hydrodynamic_state_scaling import (  # noqa: E402
    FluidcoefStateScalingRuntime,
    HydrodynamicStateScaler,
)
from physics.sim_profile_helpers import load_sim_profiles  # noqa: E402
from sim.physics.fluid_contract import configure_fluid_model_contract  # noqa: E402


PROFILE = ROOT / "config" / "sim_profiles.json"


def _profile() -> dict:
    return json.loads(PROFILE.read_text())["research_pool"]


class CurrentFieldTests(unittest.TestCase):
    def test_research_profile_inherits_current_without_mutating_parent(self) -> None:
        profiles, warning = load_sim_profiles(PROFILE)
        self.assertIsNone(warning)
        current = profiles["current"]
        research = profiles["research_pool"]
        self.assertNotIn("current_field", current)
        self.assertIn("current_field", research)
        self.assertEqual(research["thruster_force_max"], current["thruster_force_max"])
        research["fossen_residual_hydro"]["active"] = False
        self.assertTrue(current["fossen_residual_hydro"]["active"])

    def test_missing_config_preserves_exact_static_current(self) -> None:
        fallback = np.array([0.2, -0.1, 0.03], dtype=np.float64)
        field = DeterministicCurrentField.from_profile(
            {},
            fallback_velocity_world_mps=fallback,
        )
        self.assertFalse(field.active)
        np.testing.assert_array_equal(field.velocity_world([100.0, -30.0, 2.0], 999.0), fallback)

    def test_pool_prior_is_reproducible_spatial_and_time_varying(self) -> None:
        fallback = np.zeros(3, dtype=np.float64)
        left = DeterministicCurrentField.from_profile(
            _profile(),
            fallback_velocity_world_mps=fallback,
        )
        right = DeterministicCurrentField.from_profile(
            _profile(),
            fallback_velocity_world_mps=fallback,
        )
        sample = left.velocity_world([-5.0, 1.0, -1.5], 12.5)
        np.testing.assert_array_equal(sample, right.velocity_world([-5.0, 1.0, -1.5], 12.5))
        self.assertFalse(np.allclose(sample, left.velocity_world([5.0, -1.0, -2.0], 12.5)))
        self.assertFalse(np.allclose(sample, left.velocity_world([-5.0, 1.0, -1.5], 17.5)))
        self.assertLessEqual(float(np.linalg.norm(sample)), 0.08 + 1.0e-12)

    def test_norm_bound_limits_large_gradient(self) -> None:
        field = DeterministicCurrentField.from_profile(
            {
                "current_field": {
                    "active": True,
                    "calibration_status": "unit_test",
                    "gradient_per_s": np.eye(3).tolist(),
                    "max_speed_mps": 0.2,
                }
            },
            fallback_velocity_world_mps=np.zeros(3),
        )
        sample = field.velocity_world([10.0, 10.0, 10.0], 0.0)
        self.assertAlmostEqual(float(np.linalg.norm(sample)), 0.2, places=12)

    def test_huge_finite_current_parameters_and_queries_are_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "gradient_per_s exceeds"):
            DeterministicCurrentField.from_profile(
                {
                    "current_field": {
                        "active": True,
                        "calibration_status": "unit_test",
                        "gradient_per_s": np.eye(3) * 1.0e300,
                    }
                },
                fallback_velocity_world_mps=np.zeros(3),
            )

        field = DeterministicCurrentField.from_profile(
            _profile(),
            fallback_velocity_world_mps=np.zeros(3),
        )
        with self.assertRaisesRegex(ValueError, "safe component bound"):
            field.velocity_world([1.0e300, 0.0, 0.0], 0.0)

    def test_turbulence_rejects_non_low_frequency_mode(self) -> None:
        with self.assertRaisesRegex(ValueError, "frequency_hz"):
            DeterministicCurrentField.from_profile(
                {
                    "current_field": {
                        "active": True,
                        "calibration_status": "unit_test",
                        "turbulence": {
                            "active": True,
                            "modes": [
                                {
                                    "amplitude_mps": [0.1, 0.0, 0.0],
                                    "frequency_hz": 2.0,
                                }
                            ],
                        },
                    }
                },
                fallback_velocity_world_mps=np.zeros(3),
            )

    def test_runtime_updates_builtin_wind_and_shared_array(self) -> None:
        shared = np.zeros(3, dtype=np.float64)
        model = SimpleNamespace(opt=SimpleNamespace(wind=np.zeros(3, dtype=np.float64)))
        field = DeterministicCurrentField.from_profile(
            _profile(),
            fallback_velocity_world_mps=shared,
        )
        runtime = CurrentFieldRuntime(
            model=model,
            field=field,
            water_current_world=shared,
            use_custom_hydrodynamics=False,
            log=lambda _message: None,
        )
        expected = field.velocity_world([1.0, 2.0, -1.0], 5.0)
        actual = runtime.update(np.array([1.0, 2.0, -1.0]), 5.0)
        self.assertIs(actual, shared)
        np.testing.assert_array_equal(shared, expected)
        np.testing.assert_array_equal(model.opt.wind, expected)


class FluidContractTests(unittest.TestCase):
    def test_compiled_fluid_proxy_avoids_filename_false_warning(self) -> None:
        model = SimpleNamespace(
            opt=SimpleNamespace(density=0.0, viscosity=0.0),
            geom_fluid=np.array([[1.0] + [0.0] * 11], dtype=np.float64),
        )
        output = io.StringIO()
        with redirect_stdout(output):
            uses_custom = configure_fluid_model_contract(
                model=model,
                fluid_model="current",
                scene_path="research_pool_slam_scene.xml",
                scene_fluid_density=1000.0,
                scene_fluid_viscosity=0.001,
            )
        self.assertFalse(uses_custom)
        self.assertNotIn("warning:", output.getvalue())


class HydrodynamicStateScalingTests(unittest.TestCase):
    def test_missing_config_returns_exact_identity(self) -> None:
        scaler = HydrodynamicStateScaler.from_profile({})
        scales = scaler.evaluate(
            relative_speed_mps=5.0,
            depth_m=20.0,
            base_rotation_world=np.eye(3),
        )
        np.testing.assert_array_equal(scales.added_mass_diag, np.ones(6))
        np.testing.assert_array_equal(scales.linear_damping_diag, np.ones(6))
        np.testing.assert_array_equal(scales.quadratic_damping_diag, np.ones(6))
        np.testing.assert_array_equal(scales.mujoco_fluidcoef, np.ones(5))

    def test_speed_depth_and_ellipsoid_tilt_have_bounded_effects(self) -> None:
        scaler = HydrodynamicStateScaler.from_profile(_profile())
        upright = scaler.evaluate(
            relative_speed_mps=0.5,
            depth_m=3.0,
            base_rotation_world=np.eye(3),
        )
        pitched = np.array(
            [[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]],
            dtype=np.float64,
        )
        tilted = scaler.evaluate(
            relative_speed_mps=0.5,
            depth_m=3.0,
            base_rotation_world=pitched,
        )
        self.assertTrue(np.all(upright.mujoco_fluidcoef > 1.0))
        self.assertTrue(np.all(tilted.mujoco_fluidcoef >= upright.mujoco_fluidcoef))
        saturated = scaler.evaluate(
            relative_speed_mps=100.0,
            depth_m=100.0,
            base_rotation_world=pitched,
        )
        self.assertTrue(np.all(saturated.mujoco_fluidcoef <= 1.35))
        self.assertTrue(np.all(saturated.quadratic_damping_diag <= 1.35))

    def test_fluidcoef_wrapper_does_not_compound(self) -> None:
        base = np.array([[0.0, 2.0, 4.0, 6.0, 8.0, 10.0]], dtype=np.float64)
        model = SimpleNamespace(geom_fluid=base.copy())

        def restore(_linear: np.ndarray, _angular: np.ndarray) -> None:
            model.geom_fluid[:] = base

        runtime = FluidcoefStateScalingRuntime(
            model=model,
            fluid_geom_ids=[0],
            enabled=True,
            update_unscaled=restore,
        )
        runtime.set_scale(np.array([1.1, 1.2, 1.3, 1.4, 1.5]))
        zero = np.zeros(3, dtype=np.float64)
        runtime.update(zero, zero)
        first = model.geom_fluid.copy()
        runtime.update(zero, zero)
        np.testing.assert_array_equal(model.geom_fluid, first)
        np.testing.assert_allclose(first[0, 1:6], [2.2, 4.8, 7.8, 11.2, 15.0])


if __name__ == "__main__":
    unittest.main(verbosity=2)
