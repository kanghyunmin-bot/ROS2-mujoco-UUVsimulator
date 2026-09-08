#!/usr/bin/env python3
"""Focused unit checks for distributed hull hydrodynamics."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim.physics.distributed_hydrodynamics import (  # noqa: E402
    DistributedHullHydrodynamics,
)


def _profile(*, patches: list[dict] | None = None, **overrides) -> dict:
    payload = {
        "active": True,
        "calibration_status": "uncalibrated_prior",
        "coefficient_provenance": "unit-test geometry prior; not measured",
        "fluid_density_kg_m3": 1000.0,
        "gravity_mps2": 10.0,
        "normal_drag_coefficient": 1.0,
        "skin_drag_coefficient": 0.1,
        "max_relative_speed_mps": 10.0,
        "max_patch_force_n": 1.0e6,
        "max_total_force_n": 1.0e6,
        "max_total_torque_nm": 1.0e6,
        "patches": patches
        or [
            {
                "name": "center",
                "position_body_m": [0.0, 0.0, 0.0],
                "normal_body": [1.0, 0.0, 0.0],
                "area_m2": 2.0,
                "volume_share_m3": 0.01,
                "half_height_m": 0.1,
            }
        ],
    }
    payload.update(overrides)
    return {"distributed_hydrodynamics": payload}


def _evaluate(model: DistributedHullHydrodynamics, **overrides):
    values = {
        "body_position_world_m": np.zeros(3),
        "rotation_world_from_body": np.eye(3),
        "linear_velocity_world_mps": np.zeros(3),
        "angular_velocity_world_radps": np.zeros(3),
        "current_world_mps": np.zeros(3),
        "surface_height_world_m": 1.0,
        "time_s": 0.0,
    }
    values.update(overrides)
    return model.evaluate(**values)


class DistributedHydrodynamicsConfigTests(unittest.TestCase):
    def test_missing_or_disabled_config_is_exact_zero_without_input_validation(self) -> None:
        for profile in ({}, {"distributed_hydrodynamics": {"active": False, "patches": "bad"}}):
            model = DistributedHullHydrodynamics.from_profile(profile)
            self.assertFalse(model.active)
            result = model.evaluate(
                body_position_world_m=[float("nan")],
                rotation_world_from_body=[float("nan")],
                linear_velocity_world_mps=[float("nan")],
                angular_velocity_world_radps=[float("nan")],
                current_world_mps=[float("nan")],
                surface_height_world_m=float("nan"),
            )
            np.testing.assert_array_equal(result.force_world_n, np.zeros(3))
            np.testing.assert_array_equal(result.torque_world_nm, np.zeros(3))
            self.assertEqual(result.patch_forces_world_n.shape, (0, 3))
            self.assertEqual(result.safety_scale, 1.0)

    def test_active_model_requires_status_and_provenance(self) -> None:
        missing = _profile()
        del missing["distributed_hydrodynamics"]["coefficient_provenance"]
        with self.assertRaisesRegex(ValueError, "coefficient_provenance"):
            DistributedHullHydrodynamics.from_profile(missing)

        empty_status = _profile(calibration_status="  ")
        with self.assertRaisesRegex(ValueError, "calibration_status"):
            DistributedHullHydrodynamics.from_profile(empty_status)

        calibrated = DistributedHullHydrodynamics.from_profile(
            _profile(calibration_status="calibrated_tow_and_decay_2026_09")
        )
        self.assertEqual(
            calibrated.config.calibration_status,
            "calibrated_tow_and_decay_2026_09",
        )

    def test_calibration_overlay_updates_metadata_and_validates_scales(self) -> None:
        profile = _profile()
        profile["distributed_hydrodynamics_calibration"] = {
            "calibration_status": "hybrid_prior",
            "coefficient_provenance": "two-model virtual measurements",
            "normal_drag_axis_scale": [2.0, 1.0, 1.0],
        }
        model = DistributedHullHydrodynamics.from_profile(profile)

        self.assertEqual(model.config.calibration_status, "hybrid_prior")
        np.testing.assert_allclose(model.config.normal_drag_coefficients, [2.0])

        profile["distributed_hydrodynamics_calibration"]["normal_drag_axis_scale"] = [
            -1.0,
            1.0,
            1.0,
        ]
        with self.assertRaisesRegex(ValueError, "normal_drag_axis_scale"):
            DistributedHullHydrodynamics.from_profile(profile)

        profile["distributed_hydrodynamics_calibration"] = {"patches": []}
        with self.assertRaisesRegex(ValueError, "unsupported keys: patches"):
            DistributedHullHydrodynamics.from_profile(profile)

    def test_rejects_non_unit_normal_and_nonfinite_patch(self) -> None:
        bad_normal = _profile()
        bad_normal["distributed_hydrodynamics"]["patches"][0]["normal_body"] = [2.0, 0.0, 0.0]
        with self.assertRaisesRegex(ValueError, "unit vector"):
            DistributedHullHydrodynamics.from_profile(bad_normal)

        bad_area = _profile()
        bad_area["distributed_hydrodynamics"]["patches"][0]["area_m2"] = float("inf")
        with self.assertRaisesRegex(ValueError, "finite"):
            DistributedHullHydrodynamics.from_profile(bad_area)

    def test_tangent_quadrature_expands_area_and_volume_without_changing_totals(self) -> None:
        profile = _profile()
        profile["distributed_hydrodynamics"]["patches"][0][
            "quadrature_spans_body_m"
        ] = [[0.0, 1.0, 0.0], [0.0, 0.0, 0.5]]
        model = DistributedHullHydrodynamics.from_profile(profile)

        self.assertEqual(model.config.patch_count, 4)
        self.assertAlmostEqual(float(np.sum(model.config.areas_m2)), 2.0)
        self.assertAlmostEqual(float(np.sum(model.config.volume_shares_m3)), 0.01)
        self.assertTrue(all("__q" in name for name in model.config.patch_names))

        result = _evaluate(
            model,
            angular_velocity_world_radps=np.array([0.0, 0.0, 1.0]),
        )
        self.assertLess(result.torque_world_nm[2], 0.0)


class DistributedHydrodynamicsForceTests(unittest.TestCase):
    def test_submerged_fraction_and_buoyancy_are_distributed_at_patch_points(self) -> None:
        patches = [
            {
                "name": "lower",
                "position_body_m": [1.0, 0.0, -0.1],
                "normal_body": [1.0, 0.0, 0.0],
                "area_m2": 1.0,
                "volume_share_m3": 0.01,
                "half_height_m": 0.1,
            },
            {
                "name": "waterline",
                "position_body_m": [-1.0, 0.0, 0.0],
                "normal_body": [-1.0, 0.0, 0.0],
                "area_m2": 1.0,
                "volume_share_m3": 0.02,
                "half_height_m": 0.1,
            },
            {
                "name": "upper",
                "position_body_m": [0.0, 0.0, 0.2],
                "normal_body": [0.0, 0.0, 1.0],
                "area_m2": 1.0,
                "volume_share_m3": 0.03,
                "half_height_m": 0.1,
            },
        ]
        model = DistributedHullHydrodynamics.from_profile(_profile(patches=patches))
        result = _evaluate(model, surface_height_world_m=0.0)

        np.testing.assert_allclose(result.submerged_fractions, [1.0, 0.5, 0.0])
        np.testing.assert_allclose(result.buoyancy_forces_world_n[:, 2], [100.0, 100.0, 0.0])
        np.testing.assert_allclose(result.force_world_n, [0.0, 0.0, 200.0])
        np.testing.assert_allclose(result.patch_torques_world_nm[:, 1], [-100.0, 100.0, 0.0])
        np.testing.assert_allclose(result.torque_world_nm, np.zeros(3))

    def test_buoyancy_offset_moves_only_volume_patches(self) -> None:
        patches = [
            {
                "name": "buoyant",
                "position_body_m": [0.0, 0.0, 0.0],
                "normal_body": [1.0, 0.0, 0.0],
                "area_m2": 0.0,
                "volume_share_m3": 0.01,
                "half_height_m": 0.1,
            },
            {
                "name": "drag_only",
                "position_body_m": [0.0, 0.0, 0.0],
                "normal_body": [1.0, 0.0, 0.0],
                "area_m2": 1.0,
                "volume_share_m3": 0.0,
                "half_height_m": 0.1,
            },
        ]
        model = DistributedHullHydrodynamics.from_profile(
            _profile(
                patches=patches,
                buoyancy_position_offset_body_m=[0.0, 0.0, 0.2],
            )
        )

        np.testing.assert_allclose(model.config.positions_body_m[0], [0.0, 0.0, 0.2])
        np.testing.assert_allclose(model.config.positions_body_m[1], [0.0, 0.0, 0.0])

    def test_residual_restoring_adds_righting_torque(self) -> None:
        model = DistributedHullHydrodynamics.from_profile(
            _profile(
                residual_restoring={
                    "active": True,
                    "roll_stiffness_nm_per_rad": 40.0,
                    "pitch_stiffness_nm_per_rad": 100.0,
                }
            )
        )
        angle = 0.1
        rotation = np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, np.cos(angle), -np.sin(angle)],
                [0.0, np.sin(angle), np.cos(angle)],
            ]
        )
        result = _evaluate(model, rotation_world_from_body=rotation)

        np.testing.assert_allclose(
            result.residual_restoring_torque_world_nm,
            rotation @ np.array([-4.0, 0.0, 0.0]),
        )
        np.testing.assert_allclose(
            result.torque_world_nm,
            result.patch_torques_world_nm.sum(axis=0)
            + result.residual_restoring_torque_world_nm,
        )

    def test_profile_buoyancy_scale_changes_only_hydrostatic_force(self) -> None:
        profile = _profile()
        profile["buoyancy_scale"] = 0.98
        model = DistributedHullHydrodynamics.from_profile(profile)
        result = _evaluate(model, linear_velocity_world_mps=np.array([1.0, 0.0, 0.0]))

        self.assertEqual(model.config.buoyancy_scale, 0.98)
        np.testing.assert_allclose(result.buoyancy_forces_world_n[:, 2], [98.0])
        np.testing.assert_allclose(result.form_drag_forces_world_n[:, 0], [-1000.0])

    def test_local_rotation_velocity_and_per_patch_current_drive_drag(self) -> None:
        patches = [
            {
                "name": "plus_x",
                "position_body_m": [1.0, 0.0, 0.0],
                "normal_body": [0.0, 1.0, 0.0],
                "area_m2": 1.0,
                "volume_share_m3": 0.0,
                "half_height_m": 0.1,
                "skin_drag_coefficient": 0.0,
            },
            {
                "name": "minus_x",
                "position_body_m": [-1.0, 0.0, 0.0],
                "normal_body": [0.0, 1.0, 0.0],
                "area_m2": 1.0,
                "volume_share_m3": 0.0,
                "half_height_m": 0.1,
                "skin_drag_coefficient": 0.0,
            },
        ]
        model = DistributedHullHydrodynamics.from_profile(_profile(patches=patches))
        sampled_positions: list[np.ndarray] = []

        def current(position: np.ndarray, _time_s: float) -> np.ndarray:
            sampled_positions.append(position)
            return np.array([0.0, 0.5 * position[0], 0.0])

        result = _evaluate(
            model,
            angular_velocity_world_radps=np.array([0.0, 0.0, 1.0]),
            current_world_mps=current,
        )

        self.assertEqual(len(sampled_positions), 2)
        np.testing.assert_allclose(result.point_velocities_world_mps[:, 1], [1.0, -1.0])
        np.testing.assert_allclose(result.currents_world_mps[:, 1], [0.5, -0.5])
        np.testing.assert_allclose(result.form_drag_forces_world_n[:, 1], [-125.0, 125.0])
        np.testing.assert_allclose(result.torque_world_nm, [0.0, 0.0, -250.0])

    def test_form_and_skin_drag_oppose_normal_and_tangential_motion(self) -> None:
        model = DistributedHullHydrodynamics.from_profile(_profile())
        result = _evaluate(
            model,
            linear_velocity_world_mps=np.array([2.0, 3.0, 0.0]),
        )
        np.testing.assert_allclose(result.form_drag_forces_world_n, [[-4000.0, 0.0, 0.0]])
        np.testing.assert_allclose(result.skin_drag_forces_world_n, [[0.0, -900.0, 0.0]])
        self.assertLess(float(result.form_drag_forces_world_n[0] @ np.array([2.0, 3.0, 0.0])), 0.0)
        self.assertLess(float(result.skin_drag_forces_world_n[0] @ np.array([2.0, 3.0, 0.0])), 0.0)

    def test_force_and_torque_outputs_transform_to_body_frame(self) -> None:
        model = DistributedHullHydrodynamics.from_profile(_profile())
        rotation = np.array(
            [
                [0.0, -1.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0],
            ]
        )
        result = _evaluate(
            model,
            rotation_world_from_body=rotation,
            linear_velocity_world_mps=np.array([0.0, 2.0, 0.0]),
        )
        np.testing.assert_allclose(result.force_body_n, rotation.T @ result.force_world_n)
        np.testing.assert_allclose(result.torque_body_nm, rotation.T @ result.torque_world_nm)

    def test_wrench_reference_changes_torque_without_changing_force(self) -> None:
        model = DistributedHullHydrodynamics.from_profile(_profile())
        origin_result = _evaluate(model)
        reference_result = _evaluate(
            model,
            wrench_reference_position_body_m=np.array([1.0, 0.0, 0.0]),
        )

        np.testing.assert_allclose(reference_result.force_world_n, origin_result.force_world_n)
        np.testing.assert_allclose(
            reference_result.wrench_reference_position_world_m,
            [1.0, 0.0, 0.0],
        )
        np.testing.assert_allclose(reference_result.torque_world_nm, [0.0, 100.0, 0.0])

    def test_speed_and_wrench_bounds_stay_finite_and_consistent(self) -> None:
        patches = [
            {
                "name": "lever",
                "position_body_m": [10.0, 0.0, 0.0],
                "normal_body": [0.0, 1.0, 0.0],
                "area_m2": 100.0,
                "volume_share_m3": 0.0,
                "half_height_m": 0.1,
            }
        ]
        model = DistributedHullHydrodynamics.from_profile(
            _profile(
                patches=patches,
                max_relative_speed_mps=2.0,
                max_patch_force_n=100.0,
                max_total_force_n=50.0,
                max_total_torque_nm=20.0,
            )
        )
        result = _evaluate(
            model,
            linear_velocity_world_mps=np.array([0.0, 1.0e100, 0.0]),
        )

        self.assertAlmostEqual(float(np.linalg.norm(result.relative_velocities_world_mps[0])), 2.0)
        self.assertLessEqual(float(np.linalg.norm(result.force_world_n)), 50.0 + 1.0e-12)
        self.assertLessEqual(float(np.linalg.norm(result.torque_world_nm)), 20.0 + 1.0e-12)
        np.testing.assert_allclose(result.patch_forces_world_n.sum(axis=0), result.force_world_n)
        np.testing.assert_allclose(
            result.patch_torques_world_nm.sum(axis=0),
            result.torque_world_nm,
        )
        self.assertTrue(np.all(np.isfinite(result.patch_forces_world_n)))
        self.assertLess(result.safety_scale, 1.0)

    def test_drag_safety_limits_do_not_change_buoyancy(self) -> None:
        model = DistributedHullHydrodynamics.from_profile(
            _profile(
                max_patch_force_n=8.0,
                max_total_force_n=5.0,
                max_total_torque_nm=5.0,
            )
        )
        still = _evaluate(model)
        fast = _evaluate(
            model,
            linear_velocity_world_mps=np.array([10.0, 0.0, 0.0]),
        )

        np.testing.assert_allclose(
            fast.buoyancy_forces_world_n,
            still.buoyancy_forces_world_n,
        )
        np.testing.assert_allclose(fast.buoyancy_forces_world_n[:, 2], [100.0])
        self.assertLessEqual(
            float(np.linalg.norm(fast.form_drag_forces_world_n.sum(axis=0))),
            5.0 + 1.0e-12,
        )
        self.assertLess(fast.safety_scale, 1.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
