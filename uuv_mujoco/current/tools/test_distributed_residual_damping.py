"""Physical invariants for residual damping and acceleration sample timing."""
import unittest
from types import SimpleNamespace
import numpy as np
from uuv_mujoco.current.tools.test_distributed_hydrodynamics import _profile, _evaluate
from sim.physics.distributed_hydrodynamics import DistributedHullHydrodynamics
from sim.runtime.underwater_relative_acceleration import update_relative_acceleration


class ResidualDampingTests(unittest.TestCase):
    def model(self, **kw):
        return DistributedHullHydrodynamics.from_profile(_profile(
            residual_linear_damping=[2,3,4,5,6,7],
            residual_quadratic_damping=[1,2,3,4,5,6], **kw))

    def test_passive_symmetric_six_axes_and_rest(self):
        model = self.model()
        for axis in range(6):
            for speed in (0., .02, .05, .1, .2, .35, .5):
                values = []
                for sign in (-1,1):
                    nu = np.zeros(6); nu[axis] = sign * speed
                    result = _evaluate(model, linear_velocity_world_mps=nu[:3], angular_velocity_world_radps=nu[3:])
                    self.assertLessEqual(float(nu @ result.residual_damping_wrench_body), 0.)
                    values.append(result.residual_damping_wrench_body)
                np.testing.assert_array_equal(values[0], -values[1])

    def test_reference_point_current_rotation_and_immersion(self):
        model = self.model()
        rotation = np.array([[0.,-1,0],[1,0,0],[0,0,1]])
        omega = np.array([0.,0.,.2]); reference = np.array([.4,.1,.0])
        current = np.array([.3,-.2,0.])
        origin_velocity = current - np.cross(omega, rotation @ reference)
        result = _evaluate(model, rotation_world_from_body=rotation,
            wrench_reference_position_body_m=reference,
            linear_velocity_world_mps=origin_velocity, angular_velocity_world_radps=omega,
            current_world_mps=lambda p,t: current, surface_height_world_m=0.)
        np.testing.assert_allclose(result.residual_damping_wrench_body[:3], 0., atol=1e-15)
        self.assertAlmostEqual(result.residual_damping_wrench_body[5], -.5*(7*.2+6*.2**2))
        dry = _evaluate(model, linear_velocity_world_mps=np.ones(3), surface_height_world_m=-1.)
        np.testing.assert_array_equal(dry.residual_damping_wrench_body, np.zeros(6))

    def test_joint_limit_preserves_buoyancy_and_component_accounting(self):
        model = self.model(max_total_force_n=.1, max_total_torque_nm=.1)
        result = _evaluate(model, linear_velocity_world_mps=np.ones(3), angular_velocity_world_radps=np.ones(3))
        dynamic = result.force_world_n-result.buoyancy_forces_world_n.sum(axis=0)
        self.assertLessEqual(np.linalg.norm(dynamic), .1+1e-12)
        np.testing.assert_allclose(result.force_world_n, result.patch_forces_world_n.sum(axis=0)+result.residual_damping_wrench_body[:3])
        self.assertEqual(result.buoyancy_forces_world_n[0,2],100.)

    def test_matching_uniform_current_has_exact_zero_residual(self):
        velocity = np.array([.1, -.2, .3])
        result = _evaluate(self.model(), linear_velocity_world_mps=velocity, current_world_mps=velocity)
        np.testing.assert_array_equal(result.residual_damping_wrench_body, np.zeros(6))

    def test_bad_coefficients_rejected(self):
        for bad in ([1]*5, [-1]*6, [float('nan')]*6):
            with self.assertRaises(ValueError):
                DistributedHullHydrodynamics.from_profile(_profile(residual_linear_damping=bad))

    def test_acceleration_uses_elapsed_sample_time(self):
        runtime = SimpleNamespace(use_custom_hydrodynamics=True,
            hydrodynamics=SimpleNamespace(fossen_residual_added_mass_active=False),
            data=SimpleNamespace(time=1.01), prev_rel_sample_time_s=1.,
            prev_rel_nu_valid=True, prev_rel_nu_body=np.zeros(6))
        np.testing.assert_allclose(update_relative_acceleration(runtime,np.ones(6)*.02,.005),np.ones(6)*2.)

if __name__ == '__main__': unittest.main()
