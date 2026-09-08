#!/usr/bin/env python3
"""Focused checks for opt-in thruster inflow and reaction torque."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
import sys
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim.physics.thruster_inflow import (  # noqa: E402
    ThrusterInflowConfig,
    apply_thruster_inflow,
    thruster_inflow_profile_overrides,
)
from sim.physics.thruster_param_defaults import default_thruster_global_params  # noqa: E402
from sim.physics.thruster_param_global import apply_global_params  # noqa: E402
from sim.runtime.thruster_actuator_inflow import force_with_local_inflow  # noqa: E402
from sim.runtime.thruster_actuator_forces import update_thruster_forces  # noqa: E402
from sim.runtime.thruster_actuator_wrench import (  # noqa: E402
    accumulate_thruster_wrench,
    reaction_torque_world_for_rotation,
)


class ThrusterInflowModelTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = ThrusterInflowConfig(
            enabled=True,
            reference_speed_mps=1.0,
            minimum_reference_speed_mps=0.1,
            command_exponent=0.0,
            gain_per_advance_ratio=0.5,
            minimum_multiplier=0.4,
            maximum_multiplier=1.2,
        )

    def test_disabled_model_preserves_force_bit_for_bit(self) -> None:
        force = float(np.nextafter(12.0, 13.0))
        result = apply_thruster_inflow(
            force,
            command_fraction=0.8,
            thrust_axis_world=np.array([1.0, 0.0, 0.0]),
            site_velocity_world_mps=np.array([999.0, 0.0, 0.0]),
            water_velocity_world_mps=np.array([-999.0, 0.0, 0.0]),
            config=ThrusterInflowConfig(enabled=False),
        )
        self.assertEqual(result.force_n, force)
        self.assertEqual(result.multiplier, 1.0)

    def test_advance_reduces_thrust_and_opposing_current_increases_it(self) -> None:
        advancing = apply_thruster_inflow(
            10.0,
            command_fraction=1.0,
            thrust_axis_world=np.array([1.0, 0.0, 0.0]),
            site_velocity_world_mps=np.array([0.5, 0.0, 0.0]),
            water_velocity_world_mps=np.zeros(3),
            config=self.config,
        )
        opposing = apply_thruster_inflow(
            10.0,
            command_fraction=1.0,
            thrust_axis_world=np.array([1.0, 0.0, 0.0]),
            site_velocity_world_mps=np.zeros(3),
            water_velocity_world_mps=np.array([0.5, 0.0, 0.0]),
            config=self.config,
        )
        self.assertAlmostEqual(advancing.force_n, 7.5)
        self.assertAlmostEqual(advancing.axial_advance_speed_mps, 0.5)
        self.assertAlmostEqual(opposing.force_n, 12.0)
        self.assertAlmostEqual(opposing.multiplier, self.config.maximum_multiplier)

    def test_reverse_thrust_uses_reverse_command_axis(self) -> None:
        result = apply_thruster_inflow(
            -10.0,
            command_fraction=-1.0,
            thrust_axis_world=np.array([1.0, 0.0, 0.0]),
            site_velocity_world_mps=np.array([-0.5, 0.0, 0.0]),
            water_velocity_world_mps=np.zeros(3),
            config=self.config,
        )
        self.assertAlmostEqual(result.force_n, -7.5)
        self.assertAlmostEqual(result.axial_advance_speed_mps, 0.5)

    def test_multiplier_is_bounded_at_high_advance_speed(self) -> None:
        result = apply_thruster_inflow(
            10.0,
            command_fraction=1.0,
            thrust_axis_world=np.array([1.0, 0.0, 0.0]),
            site_velocity_world_mps=np.array([20.0, 0.0, 0.0]),
            water_velocity_world_mps=np.zeros(3),
            config=self.config,
        )
        self.assertAlmostEqual(result.force_n, 4.0)
        self.assertAlmostEqual(result.multiplier, self.config.minimum_multiplier)

    def test_profile_override_requires_provenance_and_strict_bounds(self) -> None:
        profile = {
            "thruster_inflow": {
                "active": True,
                "calibration_status": "uncalibrated_prior",
                "provenance": "unit test",
                "reference_speed_mps": 0.9,
                "minimum_reference_speed_mps": 0.2,
            }
        }
        values, status, provenance = thruster_inflow_profile_overrides(profile)
        self.assertIs(values["inflow_enabled"], True)
        self.assertEqual(values["inflow_reference_speed_mps"], 0.9)
        self.assertEqual(status, "uncalibrated_prior")
        self.assertEqual(provenance, "unit test")

        with self.assertRaisesRegex(ValueError, "provenance"):
            thruster_inflow_profile_overrides(
                {"thruster_inflow": {"active": True, "calibration_status": "prior"}}
            )
        invalid = profile.copy()
        invalid["thruster_inflow"] = dict(profile["thruster_inflow"])
        invalid["thruster_inflow"]["minimum_multiplier"] = -0.1
        with self.assertRaisesRegex(ValueError, "minimum_multiplier"):
            thruster_inflow_profile_overrides(invalid)

    def test_global_json_boolean_remains_boolean(self) -> None:
        target = default_thruster_global_params()
        apply_global_params({"inflow_enabled": True}, target)
        self.assertIs(target["inflow_enabled"], True)


class _FakeMujoco:
    class mjtObj:
        mjOBJ_SITE = 6
        mjOBJ_BODY = 1

    @staticmethod
    def mj_objectVelocity(_model, data, _object_type, object_id, result, _local) -> None:
        result[:] = 0.0
        result[3:6] = data.object_linear_velocity[int(object_id)]


class ThrusterInflowRuntimeTests(unittest.TestCase):
    def test_runtime_uses_site_velocity_and_local_current_sampler(self) -> None:
        sampled_positions: list[np.ndarray] = []

        def sample_current(position_world: np.ndarray, _time_s: float) -> np.ndarray:
            sampled_positions.append(position_world)
            return np.array([0.1, 0.0, 0.0], dtype=np.float64)

        runtime = SimpleNamespace(
            model=SimpleNamespace(
                actuator_gear=np.array([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0]]),
                opt=SimpleNamespace(wind=np.zeros(3)),
            ),
            data=SimpleNamespace(
                site_xpos=np.array([[2.0, 3.0, -1.0]], dtype=np.float64),
                site_xmat=np.array([np.eye(3).reshape(9)], dtype=np.float64),
                xpos=np.array([[0.0, 0.0, 0.0]], dtype=np.float64),
                object_linear_velocity={0: np.array([0.5, 0.0, 0.0])},
                time=4.0,
            ),
            mujoco_module=_FakeMujoco,
            current_velocity_sampler=sample_current,
            last_inflow_multiplier={"thr": 1.0},
            last_axial_advance_speed_mps={"thr": 0.0},
        )
        config = ThrusterInflowConfig(
            enabled=True,
            reference_speed_mps=1.0,
            minimum_reference_speed_mps=0.1,
            command_exponent=0.0,
            gain_per_advance_ratio=0.5,
            minimum_multiplier=0.4,
            maximum_multiplier=1.2,
        )
        result = force_with_local_inflow(
            runtime,
            name="thr",
            aid=0,
            sid=0,
            base_id=0,
            base_rot=np.eye(3),
            static_force_n=10.0,
            command_fraction=1.0,
            config=config,
        )
        self.assertAlmostEqual(result, 8.0)
        self.assertAlmostEqual(runtime.last_axial_advance_speed_mps["thr"], 0.4)
        np.testing.assert_array_equal(sampled_positions[0], [2.0, 3.0, -1.0])

    def test_runtime_projects_inflow_on_rotated_site_axis(self) -> None:
        site_rotation = np.array(
            [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        runtime = SimpleNamespace(
            model=SimpleNamespace(
                actuator_gear=np.array([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0]]),
                opt=SimpleNamespace(wind=np.zeros(3)),
            ),
            data=SimpleNamespace(
                site_xpos=np.zeros((1, 3)),
                site_xmat=np.array([site_rotation.reshape(9)]),
                xpos=np.zeros((1, 3)),
                object_linear_velocity={0: np.array([0.0, 0.5, 0.0])},
                time=0.0,
            ),
            mujoco_module=_FakeMujoco,
            current_velocity_sampler=lambda _position, _time: np.zeros(3),
            last_inflow_multiplier={"thr": 1.0},
            last_axial_advance_speed_mps={"thr": 0.0},
        )
        result = force_with_local_inflow(
            runtime,
            name="thr",
            aid=0,
            sid=0,
            base_id=0,
            base_rot=np.eye(3),
            static_force_n=10.0,
            command_fraction=1.0,
            config=ThrusterInflowConfig(
                enabled=True,
                reference_speed_mps=1.0,
                minimum_reference_speed_mps=0.1,
                command_exponent=0.0,
                gain_per_advance_ratio=0.5,
                minimum_multiplier=0.4,
                maximum_multiplier=1.2,
            ),
        )
        self.assertAlmostEqual(result, 7.5)

    def test_disabled_runtime_does_not_require_mujoco_velocity_api(self) -> None:
        runtime = SimpleNamespace(
            last_inflow_multiplier={"thr": 0.0},
            last_axial_advance_speed_mps={"thr": 99.0},
        )
        force = float(np.nextafter(-8.0, -9.0))
        result = force_with_local_inflow(
            runtime,
            name="thr",
            aid=0,
            sid=0,
            base_id=0,
            base_rot=np.eye(3),
            static_force_n=force,
            command_fraction=-0.5,
            config=ThrusterInflowConfig(enabled=False),
        )
        self.assertEqual(result, force)
        self.assertEqual(runtime.last_inflow_multiplier["thr"], 1.0)

    def test_force_update_passes_base_id_to_local_inflow(self) -> None:
        global_params = default_thruster_global_params()
        global_params["inflow_enabled"] = True
        runtime = SimpleNamespace(
            model=SimpleNamespace(
                actuator_gear=np.array([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0]]),
                body_ipos=np.array([[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]]),
                site_pos=np.array([[0.2, 0.0, -0.1]]),
                opt=SimpleNamespace(wind=np.zeros(3)),
            ),
            data=SimpleNamespace(
                xmat=np.tile(np.eye(3).reshape(1, 9), (2, 1)),
                site_xpos=np.array([[0.2, 0.0, -0.1]]),
                site_xmat=np.array([np.eye(3).reshape(9)]),
                xpos=np.zeros((2, 3)),
                xipos=np.zeros((2, 3)),
                object_linear_velocity={0: np.zeros(3)},
                time=1.0,
                ctrl=np.zeros(1),
            ),
            mujoco_module=_FakeMujoco,
            actuator_ids={"thr": 0},
            ctrlrange=np.array([[-100.0, 100.0]]),
            all_thruster_names=["thr"],
            site_ids={"thr": 0},
            state={"thr": 0.0},
            target={"thr": 0.5},
            force_cmd={"thr": 0.0},
            thruster_global=global_params,
            thruster_scale={"thr": 1.0},
            thruster_direct_scale={"thr": 1.0},
            thruster_reverse_asymmetry={"thr": None},
            thruster_tau_up={"thr": None},
            thruster_tau_down={"thr": None},
            perf_cfg={"active": False, "direct": False},
            thruster_force_max=100.0,
            water_surface_z=0.0,
            thruster_air_force_scale=0.0,
            thruster_immersion_half_height_m=0.05,
            buoyancy_model="ellipsoid",
            surface_height_sampler=None,
            current_velocity_sampler=lambda _position, _time: np.zeros(3),
            last_inflow_multiplier={"thr": 1.0},
            last_axial_advance_speed_mps={"thr": 0.0},
            last_reaction_torque_world=np.zeros(3),
            last_force_body=np.zeros(3),
            last_torque_body=np.zeros(3),
            prop_spin_sign={"thr": 1.0},
            yaw_torque_thruster_scales={},
            yaw_torque_scale=1.0,
            yaw_thrusters=[],
        )
        update_thruster_forces(runtime, 0.02, base_id=1)
        self.assertTrue(np.isfinite(runtime.force_cmd["thr"]))
        self.assertNotEqual(runtime.force_cmd["thr"], 0.0)


class ReactionTorqueTests(unittest.TestCase):
    def _runtime(self) -> SimpleNamespace:
        return SimpleNamespace(
            model=SimpleNamespace(
                actuator_gear=np.array([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0]]),
                site_pos=np.array([[0.0, 0.2, 0.0]]),
            ),
            data=SimpleNamespace(
                site_xpos=np.array([[0.0, 0.2, 0.0]]),
                site_xmat=np.array([np.eye(3).reshape(9)]),
                xipos=np.zeros((1, 3)),
            ),
            prop_spin_sign={"thr": -1.0},
            last_force_body=np.zeros(3),
            last_torque_body=np.zeros(3),
            last_reaction_torque_world=np.zeros(3),
            yaw_torque_thruster_scales={},
            yaw_torque_scale=1.0,
            yaw_thrusters=[],
        )

    def test_reaction_torque_is_zero_at_legacy_default_and_configurable(self) -> None:
        disabled = self._runtime()
        accumulate_thruster_wrench(
            disabled,
            name="thr",
            aid=0,
            sid=0,
            force=10.0,
            base_id=0,
            base_rot=np.eye(3),
            com_body=np.zeros(3),
            reaction_torque_gain=0.0,
        )
        np.testing.assert_array_equal(disabled.last_reaction_torque_world, np.zeros(3))

        enabled = self._runtime()
        accumulate_thruster_wrench(
            enabled,
            name="thr",
            aid=0,
            sid=0,
            force=10.0,
            base_id=0,
            base_rot=np.eye(3),
            com_body=np.zeros(3),
            reaction_torque_gain=0.02,
        )
        np.testing.assert_allclose(enabled.last_reaction_torque_world, [0.2, 0.0, 0.0])
        np.testing.assert_allclose(enabled.last_reaction_torque_body, [0.2, 0.0, 0.0])
        quarter_turn = np.array(
            [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        np.testing.assert_allclose(
            reaction_torque_world_for_rotation(enabled, quarter_turn),
            [0.0, 0.2, 0.0],
            atol=1.0e-12,
        )

    def test_wrench_uses_site_world_frame_and_actual_nested_position(self) -> None:
        runtime = self._runtime()
        site_rotation = np.array(
            [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        runtime.data.site_xmat[0] = site_rotation.reshape(9)
        runtime.data.site_xpos[0] = [1.0, 0.0, 0.0]
        runtime.model.site_pos[0] = [99.0, 99.0, 99.0]
        accumulate_thruster_wrench(
            runtime,
            name="thr",
            aid=0,
            sid=0,
            force=10.0,
            base_id=0,
            base_rot=np.eye(3),
            com_body=np.zeros(3),
            reaction_torque_gain=0.0,
        )
        np.testing.assert_allclose(runtime.last_force_body, [0.0, 10.0, 0.0])
        np.testing.assert_allclose(runtime.last_torque_body, [0.0, 0.0, 10.0])


if __name__ == "__main__":
    unittest.main(verbosity=2)
