"""Compare batched water queries with the original scalar physical model."""

from dataclasses import fields, replace
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import Mock

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from physics.sim_profile_helpers import build_sim_profile, load_sim_profiles
from sim.physics.current_field import DeterministicCurrentField
from sim.physics.distributed_hydrodynamics import DistributedHullHydrodynamics
from sim.physics.free_surface import FreeSurface
from sim.runtime.water_environment_runtime import WaterEnvironmentRuntime
from sim.runtime.underwater_distributed_hydrodynamics import (
    apply_distributed_hydrodynamics,
)


class BatchSamplingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.profiles, _ = load_sim_profiles(ROOT / "config/sim_profiles.json")

    def environment(self, name):
        profile = build_sim_profile(self.profiles, name)
        current = DeterministicCurrentField.from_profile(
            profile, fallback_velocity_world_mps=np.zeros(3)
        )
        surface = FreeSurface.from_profile(profile, reference_height_world_m=0.0)
        return profile, WaterEnvironmentRuntime(current, surface, 0.0)

    def test_current_matches_scalar_including_clipping_and_disabled_field(self):
        _, env = self.environment("research_pool_distributed")
        rng = np.random.default_rng(529)
        points = rng.uniform(-20, 20, (257, 3))
        for active in (True, False):
            for limit in (0.0001, 0.08, 20):
                current = DeterministicCurrentField(
                    replace(
                        env.current_field.config, active=active, max_speed_mps=limit
                    )
                )
                for time in (-10000, 0, 1.7, 10000):
                    expected = np.asarray(
                        [current.velocity_world(p, time) for p in points]
                    )
                    np.testing.assert_allclose(
                        current.velocity_world_batch(points, time),
                        expected,
                        atol=2e-14,
                        rtol=2e-13,
                    )
                self.assertEqual(
                    current.velocity_world_batch(np.empty((0, 3)), 0).shape, (0, 3)
                )

    def test_all_distributed_outputs_match_scalar_across_surface_and_motion(self):
        rng = np.random.default_rng(615)
        for name in (
            "research_pool_distributed",
            "research_pool_distributed_hybrid",
            "research_pool_distributed_waves",
        ):
            profile, env = self.environment(name)
            model = DistributedHullHydrodynamics.from_profile(profile)
            for i in range(30):
                rotation, _ = np.linalg.qr(rng.normal(size=(3, 3)))
                if np.linalg.det(rotation) < 0:
                    rotation[:, 0] *= -1
                kwargs = dict(
                    body_position_world_m=np.array([1.2, -0.8, (-3, -0.1, 0.2)[i % 3]]),
                    rotation_world_from_body=rotation,
                    linear_velocity_world_mps=rng.normal(size=3)
                    * (100 if i % 4 == 0 else 1),
                    angular_velocity_world_radps=rng.normal(size=3),
                    current_world_mps=env.velocity_world,
                    surface_height_world_m=env.surface_height_world_m,
                    wrench_reference_position_body_m=np.array([0.03, -0.02, 0.01]),
                    time_s=i * 1.13,
                )
                scalar = model.evaluate(**kwargs)
                batch = model.evaluate(
                    **kwargs,
                    current_batch_sampler=env.velocity_world_batch,
                    surface_batch_sampler=env.surface_height_world_m_batch,
                )
                for f in fields(scalar):
                    np.testing.assert_allclose(
                        getattr(batch, f.name),
                        getattr(scalar, f.name),
                        atol=1e-10,
                        rtol=1e-12,
                        err_msg=f"{name}: {i}: {f.name}",
                    )

    def test_runtime_uses_one_batch_query_for_all_105_points(self):
        profile, original = self.environment("research_pool_distributed")
        env = SimpleNamespace(
            **{
                name: Mock(wraps=getattr(original, name))
                for name in (
                    "velocity_world",
                    "surface_height_world_m",
                    "velocity_world_batch",
                    "surface_height_world_m_batch",
                )
            }
        )
        runtime = SimpleNamespace(
            hydrodynamics=SimpleNamespace(
                distributed_hydrodynamics=DistributedHullHydrodynamics.from_profile(
                    profile
                ),
                water_environment_runtime=env,
            ),
            data=SimpleNamespace(time=2.0, xfrc_applied=np.zeros((1, 6))),
            base_id=0,
        )
        apply_distributed_hydrodynamics(
            runtime,
            base_rot=np.eye(3),
            base_origin=np.array([0.0, 0.0, -2.0]),
            com=np.array([0.02, 0.0, -2.0]),
            lin_vel_com_body=np.array([0.2, 0.1, -0.1]),
            ang_vel_body=np.array([0.0, 0.1, 0.2]),
        )
        self.assertEqual(env.velocity_world_batch.call_count, 1)
        self.assertEqual(env.surface_height_world_m_batch.call_count, 1)
        self.assertLessEqual(
            env.velocity_world.call_count, 1
        )  # residual reference only
        self.assertEqual(
            runtime.last_distributed_hydrodynamics_result.positions_world_m.shape,
            (105, 3),
        )

    def test_batch_rejects_invalid_positions_and_time(self):
        _, env = self.environment("research_pool_distributed")
        for value in ([1, 2, 3], [[0, 0]], [[np.nan, 0, 0]], [[np.inf, 0, 0]]):
            for method in (env.velocity_world_batch, env.surface_height_world_m_batch):
                with self.assertRaises(ValueError):
                    method(value, 0)
        for value in (np.nan, np.inf, 1e13):
            with self.assertRaises(ValueError):
                env.velocity_world_batch(np.zeros((2, 3)), value)
        with self.assertRaises(ValueError):
            env.velocity_world_batch(np.array([[1e7, 0, 0]]), 0)


if __name__ == "__main__":
    unittest.main()
