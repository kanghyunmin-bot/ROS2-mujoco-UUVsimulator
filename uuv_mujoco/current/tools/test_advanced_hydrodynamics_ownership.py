#!/usr/bin/env python3
"""Checks that advanced UUV physics has one explicit force owner."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
import sys
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from physics.sim_profile_helpers import build_sim_profile, load_sim_profiles  # noqa: E402
from sim.physics.distributed_hydrodynamics import DistributedHullHydrodynamics  # noqa: E402
from sim.physics.full_matrix_hydrodynamics import FullMatrixHydrodynamics  # noqa: E402
from sim.physics.fluid_model import validate_requested_fluid_model_profile  # noqa: E402
from sim.physics.hydrodynamic_state_scaling import HydrodynamicStateScaler  # noqa: E402
from sim.runtime.hydrodynamics_runtime_ownership import (  # noqa: E402
    validate_advanced_hydrodynamics_ownership,
)


class AdvancedHydrodynamicsOwnershipTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        profiles, warning = load_sim_profiles(ROOT / "config" / "sim_profiles.json")
        if warning:
            raise AssertionError(warning)
        cls.profile = build_sim_profile(profiles, "research_pool_distributed")

    def _arguments(self) -> dict:
        return {
            "use_custom_hydrodynamics": True,
            "hydro_cfg": SimpleNamespace(hydrostatic_restoring_active=False),
            "values": SimpleNamespace(
                added_mass_diag=np.zeros(6),
                linear_damping_diag=np.zeros(6),
                quadratic_damping_diag=np.zeros(6),
                surface_heave_damping=0.0,
                buoyancy_scale=1.0,
                hydro_pitch_moment_coeff=0.0,
                hydro_vertical_lift_coeff=0.0,
                hydro_yawrate_heave_pos_coeff=0.0,
                hydro_yawrate_heave_neg_coeff=0.0,
                heave_extra_damping_n_per_mps=0.0,
            ),
            "wrenches": SimpleNamespace(
                fossen_residual_added_mass_active=False,
                fossen_residual_requested_active=False,
                fossen_residual_requested_added_mass_active=False,
                cfd_dynamic_wrench_enabled=False,
                cfd_dynamic_wrench_axes={},
            ),
            "state_coefficient_scaler": HydrodynamicStateScaler.from_profile(self.profile),
            "distributed_hydrodynamics": DistributedHullHydrodynamics.from_profile(self.profile),
            "full_matrix_hydrodynamics": FullMatrixHydrodynamics.from_profile(self.profile),
            "neutral_volume_m3": 0.015008,
            "rho_kg_m3": 1000.0,
            "gravity_world_mps2": np.array([0.0, 0.0, -9.81]),
        }

    def test_shipped_distributed_profile_has_one_owner(self) -> None:
        validate_advanced_hydrodynamics_ownership(**self._arguments())
        validate_requested_fluid_model_profile("distributed", self.profile)

    def test_distributed_alias_requires_an_active_patch_profile(self) -> None:
        for alias in ("distributed", "patch"):
            with self.subTest(alias=alias), self.assertRaisesRegex(
                ValueError,
                "distributed_hydrodynamics.active=true",
            ):
                validate_requested_fluid_model_profile(alias, {"name": "legacy"})
        validate_requested_fluid_model_profile("legacy", {"name": "legacy"})

    def test_builtin_mujoco_fluid_cannot_remain_enabled(self) -> None:
        arguments = self._arguments()
        arguments["use_custom_hydrodynamics"] = False
        with self.assertRaisesRegex(ValueError, "--fluid-model distributed"):
            validate_advanced_hydrodynamics_ownership(**arguments)

    def test_patch_and_legacy_drag_cannot_overlap(self) -> None:
        arguments = self._arguments()
        arguments["values"].linear_damping_diag[0] = 1.0
        with self.assertRaisesRegex(ValueError, "owns hull drag"):
            validate_advanced_hydrodynamics_ownership(**arguments)

    def test_patch_and_cfd_table_drag_cannot_overlap(self) -> None:
        arguments = self._arguments()
        arguments["wrenches"].cfd_dynamic_wrench_enabled = True
        arguments["wrenches"].cfd_dynamic_wrench_axes = {"x": {}}
        with self.assertRaisesRegex(ValueError, "disable cfd_dynamic_wrench"):
            validate_advanced_hydrodynamics_ownership(**arguments)

    def test_cfd_total_force_cannot_layer_on_mujoco_ellipsoid(self) -> None:
        arguments = self._arguments()
        arguments["distributed_hydrodynamics"] = DistributedHullHydrodynamics.from_profile({})
        arguments["full_matrix_hydrodynamics"] = FullMatrixHydrodynamics.from_profile({})
        arguments["use_custom_hydrodynamics"] = False
        arguments["wrenches"].cfd_dynamic_wrench_enabled = True
        arguments["wrenches"].cfd_dynamic_wrench_axes = {"x": {}}
        with self.assertRaisesRegex(ValueError, "cannot be layered on MuJoCo ellipsoid"):
            validate_advanced_hydrodynamics_ownership(**arguments)

    def test_cfd_total_force_requires_zero_legacy_damping_on_owned_axes(self) -> None:
        arguments = self._arguments()
        arguments["distributed_hydrodynamics"] = DistributedHullHydrodynamics.from_profile({})
        arguments["full_matrix_hydrodynamics"] = FullMatrixHydrodynamics.from_profile({})
        arguments["wrenches"].cfd_dynamic_wrench_enabled = True
        arguments["wrenches"].cfd_dynamic_wrench_axes = {"x": {}}
        arguments["values"].linear_damping_diag[0] = 2.0
        with self.assertRaisesRegex(ValueError, "overlaps legacy diagonal damping"):
            validate_advanced_hydrodynamics_ownership(**arguments)

        arguments["values"].linear_damping_diag[0] = 0.0
        validate_advanced_hydrodynamics_ownership(**arguments)

    def test_cfd_heave_total_force_rejects_other_heave_terms(self) -> None:
        arguments = self._arguments()
        arguments["distributed_hydrodynamics"] = DistributedHullHydrodynamics.from_profile({})
        arguments["full_matrix_hydrodynamics"] = FullMatrixHydrodynamics.from_profile({})
        arguments["wrenches"].cfd_dynamic_wrench_enabled = True
        arguments["wrenches"].cfd_dynamic_wrench_axes = {"z": {}}
        arguments["values"].surface_heave_damping = 3.0
        with self.assertRaisesRegex(ValueError, "owns heave total force"):
            validate_advanced_hydrodynamics_ownership(**arguments)

    def test_patch_and_empirical_heave_damping_cannot_overlap(self) -> None:
        arguments = self._arguments()
        arguments["values"].heave_extra_damping_n_per_mps = 50.0
        with self.assertRaisesRegex(ValueError, "disable empirical"):
            validate_advanced_hydrodynamics_ownership(**arguments)

    def test_matrix_and_legacy_added_mass_cannot_overlap(self) -> None:
        arguments = self._arguments()
        arguments["values"].added_mass_diag[2] = 1.0
        with self.assertRaisesRegex(ValueError, "owns added mass"):
            validate_advanced_hydrodynamics_ownership(**arguments)

    def test_distributed_rejects_fossen_damping_without_added_mass(self):
        arguments = self._arguments()
        arguments["wrenches"].fossen_residual_requested_active = True
        arguments["wrenches"].fossen_residual_requested_added_mass_active = False
        with self.assertRaisesRegex(ValueError, "owns damping; disable fossen_residual_hydro"):
            validate_advanced_hydrodynamics_ownership(**arguments)

    def test_matrix_rejects_requested_fossen_mass_even_if_runtime_suppressed_it(self) -> None:
        arguments = self._arguments()
        arguments["wrenches"].fossen_residual_requested_active = True
        arguments["wrenches"].fossen_residual_requested_added_mass_active = True
        with self.assertRaisesRegex(ValueError, "disable fossen_residual_hydro.added_mass"):
            validate_advanced_hydrodynamics_ownership(**arguments)

    def test_matrix_and_legacy_damping_cannot_overlap_without_patches(self) -> None:
        arguments = self._arguments()
        arguments["distributed_hydrodynamics"] = DistributedHullHydrodynamics.from_profile({})
        arguments["values"].quadratic_damping_diag[4] = 1.0
        with self.assertRaisesRegex(ValueError, "owns matrix damping"):
            validate_advanced_hydrodynamics_ownership(**arguments)

    def test_matrix_and_cfd_drag_reject_only_shared_output_axes(self) -> None:
        matrix_mapping = {
            "active": True,
            "calibration_status": "test",
            "provenance": "unit test",
            "reference_point": "center_of_mass",
            "added_mass_6x6": np.zeros((6, 6)).tolist(),
            "linear_damping_6x6": np.diag([2.0, 0.0, 0.0, 0.0, 0.0, 0.0]).tolist(),
            "quadratic_damping_6x6": np.zeros((6, 6)).tolist(),
        }
        arguments = self._arguments()
        arguments["distributed_hydrodynamics"] = DistributedHullHydrodynamics.from_profile({})
        arguments["full_matrix_hydrodynamics"] = FullMatrixHydrodynamics.from_mapping(
            matrix_mapping
        )
        arguments["wrenches"].cfd_dynamic_wrench_enabled = True
        arguments["wrenches"].cfd_dynamic_wrench_axes = {"y": {}}
        validate_advanced_hydrodynamics_ownership(**arguments)

        arguments["wrenches"].cfd_dynamic_wrench_axes = {"x": {}}
        with self.assertRaisesRegex(ValueError, "same translational output axis"):
            validate_advanced_hydrodynamics_ownership(**arguments)

    def test_matrix_rejects_unapplied_state_scaler(self) -> None:
        arguments = self._arguments()
        arguments["distributed_hydrodynamics"] = DistributedHullHydrodynamics.from_profile({})
        arguments["state_coefficient_scaler"] = HydrodynamicStateScaler.from_profile(
            {
                "hydrodynamic_state_scaling": {
                    "active": True,
                    "calibration_status": "unit_test",
                }
            }
        )
        with self.assertRaisesRegex(ValueError, "not applied to full 6x6 matrices"):
            validate_advanced_hydrodynamics_ownership(**arguments)

    def test_matrix_and_empirical_heave_reject_only_shared_output_axis(self) -> None:
        matrix_mapping = {
            "active": True,
            "calibration_status": "test",
            "provenance": "unit test",
            "reference_point": "center_of_mass",
            "added_mass_6x6": np.zeros((6, 6)).tolist(),
            "linear_damping_6x6": np.diag([2.0, 0.0, 0.0, 0.0, 0.0, 0.0]).tolist(),
            "quadratic_damping_6x6": np.zeros((6, 6)).tolist(),
        }
        arguments = self._arguments()
        arguments["distributed_hydrodynamics"] = DistributedHullHydrodynamics.from_profile({})
        arguments["full_matrix_hydrodynamics"] = FullMatrixHydrodynamics.from_mapping(
            matrix_mapping
        )
        arguments["values"].heave_extra_damping_n_per_mps = 4.0
        validate_advanced_hydrodynamics_ownership(**arguments)

        matrix_mapping["linear_damping_6x6"] = np.diag(
            [0.0, 0.0, 2.0, 0.0, 0.0, 0.0]
        ).tolist()
        arguments["full_matrix_hydrodynamics"] = FullMatrixHydrodynamics.from_mapping(
            matrix_mapping
        )
        with self.assertRaisesRegex(ValueError, "same heave/pitch output axis"):
            validate_advanced_hydrodynamics_ownership(**arguments)

    def test_distributed_model_rejects_non_vertical_or_upward_gravity(self) -> None:
        for gravity in (
            np.array([0.1, 0.0, -9.81]),
            np.array([0.0, 0.0, 9.81]),
        ):
            arguments = self._arguments()
            arguments["gravity_world_mps2"] = gravity
            with self.subTest(gravity=gravity), self.assertRaisesRegex(
                ValueError,
                "vertical gravity",
            ):
                validate_advanced_hydrodynamics_ownership(**arguments)


if __name__ == "__main__":
    unittest.main(verbosity=2)
