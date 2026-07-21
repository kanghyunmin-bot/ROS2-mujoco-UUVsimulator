"""Ellipsoid damping roll-stability sweep candidates."""

from __future__ import annotations

from roll_stability_candidate_types import Candidate


def ellipsoid_candidates() -> list[Candidate]:
    return [
        Candidate(
            "ellipsoid_roll_damping_p50",
            fluid_angular_scale=1.50,
            note="increase MuJoCo ellipsoid angular damping only",
        ),
        Candidate(
            "ellipsoid_roll_damping_p100",
            fluid_angular_scale=2.00,
            note="stronger MuJoCo ellipsoid angular damping only",
        ),
        Candidate(
            "ellipsoid_angular_p300",
            fluid_angular_scale=3.00,
            note="increase MuJoCo ellipsoid angular damping 3x",
        ),
        Candidate(
            "ellipsoid_angular_p500",
            fluid_angular_scale=5.00,
            note="increase MuJoCo ellipsoid angular damping 5x",
        ),
        Candidate(
            "near_neutral_ellipsoid_p50",
            {
                "buoyancy_scale": 1.0005,
                "cob_torque_scale": 0.50,
                "cob_z_offset": 0.012,
            },
            fluid_angular_scale=1.50,
            note="near-neutral hydrostatics plus moderate ellipsoid angular damping",
        ),
        Candidate(
            "near_neutral_ellipsoid_p100",
            {
                "buoyancy_scale": 1.0005,
                "cob_torque_scale": 0.50,
                "cob_z_offset": 0.012,
            },
            fluid_angular_scale=2.00,
            note="near-neutral hydrostatics plus strong ellipsoid angular damping",
        ),
    ]


__all__ = ["ellipsoid_candidates"]
