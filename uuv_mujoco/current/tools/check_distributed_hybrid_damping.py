#!/usr/bin/env python3
"""Report the retired geometry-based linear-damping candidate.

This is NOT a Stonefish fit or a replay of measured SERVO telemetry.
The crossover speeds were engineering assumptions, not identified data.  The
candidate remains reproducible for audit, but must stay disabled in profiles.
"""
from pathlib import Path
import sys
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from gui.sim_launch_preset import _load_resolved_profiles
from sim.physics.distributed_hydrodynamics import DistributedHullHydrodynamics

PROFILE = ROOT / 'config/sim_profiles.json'
CROSSOVER = np.array([0.10, 0.10, 0.10, 0.30, 0.30, 0.30])


def evaluate(model, velocity):
    return model.evaluate(body_position_world_m=np.array([0., 0., -2.]),
        rotation_world_from_body=np.eye(3), linear_velocity_world_mps=velocity[:3],
        angular_velocity_world_radps=velocity[3:], current_world_mps=np.zeros(3),
        surface_height_world_m=0., wrench_reference_position_body_m=np.array([-0.0044, 0., -0.0452]))


def derive(profile):
    model = DistributedHullHydrodynamics.from_profile(profile)
    quadratic = []
    for axis in range(6):
        velocity = np.zeros(6); velocity[axis] = 1.
        # Convert reference-point velocity into origin velocity.
        velocity[:3] -= np.cross(velocity[3:], np.array([-0.0044, 0., -0.0452]))
        result = evaluate(model, velocity)
        forces = result.form_drag_forces_world_n + result.skin_drag_forces_world_n
        torque = np.cross(result.positions_world_m - result.wrench_reference_position_world_m, forces).sum(axis=0)
        quadratic.append(-np.r_[forces.sum(axis=0), torque][axis])
    return np.asarray(quadratic) * CROSSOVER


def release(model, dt, *, braking=True):
    # Reduced pure-yaw plant with combined inertia. Identical imposed torque
    # pulse isolates damping; it does not model the Stabilize feedback loop.
    inertia = 0.8948 + 0.1092618388
    rate = 0.5; angle = 0.; minimum = 0.; crossing_angle = None
    for step in range(round(8. / dt)):
        t = step * dt
        torque = -0.5 if braking and t < 1.5 else 0.
        velocity = np.array([0., 0., 0., 0., 0., rate])
        velocity[:3] -= np.cross(velocity[3:], np.array([-0.0044, 0., -0.0452]))
        result = evaluate(model, velocity)
        drag = (result.form_drag_forces_world_n + result.skin_drag_forces_world_n)
        tau = np.cross(result.positions_world_m-result.wrench_reference_position_world_m, drag).sum(axis=0)[2]
        tau += result.residual_damping_wrench_body[5]
        rate += dt * (torque + tau) / inertia
        angle += dt * rate
        if rate <= 0. and crossing_angle is None:
            crossing_angle = angle
        if crossing_angle is not None:
            minimum = min(minimum, angle-crossing_angle)
    return -minimum


def main():
    profiles = _load_resolved_profiles(PROFILE)
    candidate = np.round(derive(profiles['research_pool_distributed_hybrid']), 10)
    cfg = profiles['research_pool_distributed_hybrid']['distributed_hydrodynamics_calibration']
    np.testing.assert_array_equal(cfg['residual_linear_damping'], np.zeros(6))
    np.testing.assert_array_equal(cfg['residual_quadratic_damping'], np.zeros(6))
    baseline = DistributedHullHydrodynamics.from_profile(profiles['research_pool_distributed'])
    hybrid = DistributedHullHydrodynamics.from_profile(profiles['research_pool_distributed_hybrid'])
    for dt in (0.01, 0.005):
        assert release(baseline, dt, braking=False) == 0.
        assert release(hybrid, dt, braking=False) == 0.
    print('PASS: residual linear/quadratic damping remains disabled.')
    print('Retired crossover candidate [u,v,w,p,q,r]:', candidate.tolist())
    print('Candidate is audit-only; recorded final-SERVO replay rejected it.')

if __name__ == '__main__': main()
