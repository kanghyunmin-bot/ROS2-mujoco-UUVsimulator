"""Check arming-time selection, observation frames and pressure support."""

from __future__ import annotations

import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from armed_replay_initial_state import build_armed_initial_state, sample_at_time


class ArmedReplayInitialStateTests(unittest.TestCase):
    def resolve(self, imu, pressure=None, mount=None, lever=None, arm_time=2.):
        return build_armed_initial_state(
            [[0., True, False, "MANUAL"], [arm_time, True, True, "ALT_HOLD"]],
            np.asarray(imu, float),
            np.array([[0., 101640.], [2., 103601.33]]) if pressure is None else pressure,
            np.eye(4) if mount is None else mount,
            np.zeros(3) if lever is None else lever,
            101640.,
        )

    def test_uses_armed_pose_and_rate_instead_of_first_boot_sample(self):
        q = Rotation.from_euler("z", [0., 90.], degrees=True).as_quat()
        imu = [[0., *q[0], 0., 0., 0.], [2., *q[1], .1, .2, .3]]
        actual = self.resolve(imu)
        self.assertAlmostEqual(actual["rpy_rad"][2], np.pi / 2)
        self.assertAlmostEqual(actual["base_depth_m"], .2)
        np.testing.assert_allclose(actual["angular_velocity_body_radps"], [.1, .2, .3])

    def test_imu_rotation_is_removed_from_pose_and_applied_to_body_rate(self):
        mount = np.eye(4)
        mount[:3, :3] = Rotation.from_euler("z", 90., degrees=True).as_matrix()
        q = Rotation.from_euler("z", 90., degrees=True).as_quat()
        imu = [[0., *q, 1., 0., 0.], [2., *q, 1., 0., 0.]]
        actual = self.resolve(imu, mount=mount)
        np.testing.assert_allclose(actual["rpy_rad"], np.zeros(3), atol=1e-12)
        np.testing.assert_allclose(actual["angular_velocity_body_radps"], [0, 1, 0], atol=1e-12)

    def test_tilted_pressure_lever_sets_body_depth_not_sensor_depth(self):
        q = Rotation.from_euler("y", 90., degrees=True).as_quat()
        imu = [[0., *q, 0., 0., 0.], [2., *q, 0., 0., 0.]]
        with self.assertWarns(UserWarning):
            actual = self.resolve(imu, lever=np.array([.1, 0, 0]))
        self.assertAlmostEqual(actual["pressure_depth_m"], .2)
        self.assertAlmostEqual(actual["base_depth_m"], .1)

    def test_quaternion_sign_change_does_not_create_rotation(self):
        q = np.array([0., 0., 0., 1.])
        value, _ = sample_at_time(np.array([[1., *q], [1.1, *(-q)]]), 1.05, .25,
                                  quaternion=True)
        self.assertAlmostEqual(Rotation.from_quat(value).magnitude(), 0.)

    def test_large_pressure_gap_is_visible_as_coarse_support(self):
        q = [0., 0., 0., 1.]
        actual = self.resolve([[.9, *q, 0., 0., 0.], [1.1, *q, 0., 0., 0.]], arm_time=1.)
        self.assertTrue(actual["pressure_coarse_support"])
        self.assertEqual(actual["pressure_bracket_s"], [0., 2.])

    def test_unsupported_gap_is_rejected_without_extrapolation(self):
        with self.assertRaises(ValueError):
            sample_at_time(np.array([[0., 1.], [3., 2.]]), 1., 2.1)

    def test_missing_armed_state_is_rejected(self):
        with self.assertRaises(ValueError):
            build_armed_initial_state([[0, True, False, "MANUAL"]], np.zeros((1, 8)),
                                      np.zeros((1, 2)), np.eye(4), np.zeros(3), 101640.)


if __name__ == "__main__":
    unittest.main()
