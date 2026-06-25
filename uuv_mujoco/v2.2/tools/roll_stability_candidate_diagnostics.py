"""Diagnostic roll-stability sweep candidates."""

from __future__ import annotations

from roll_stability_candidate_types import Candidate


def diagnostic_candidates() -> list[Candidate]:
    return [
        Candidate(
            "earlier_prefix_physics",
            {
                "buoyancy_scale": 1.005,
                "cob_torque_scale": 0.75,
                "cob_z_offset": 0.018,
            },
            fluid_angular_scale=2.0 / 3.0,
            note="diagnostic: earlier stronger hydrostatics and earlier ellipsoid angular damping",
        ),
        Candidate(
            "earlier_hydro_current_ellipsoid",
            {
                "buoyancy_scale": 1.005,
                "cob_torque_scale": 0.75,
                "cob_z_offset": 0.018,
            },
            note="diagnostic: earlier stronger hydrostatics with current ellipsoid angular damping",
        ),
        Candidate(
            "current_hydro_old_ellipsoid",
            fluid_angular_scale=2.0 / 3.0,
            note="diagnostic: current hydrostatics with weaker earlier ellipsoid angular damping",
        ),
        Candidate(
            "current_hydro_stronger_ellipsoid",
            fluid_angular_scale=4.0 / 3.0,
            note="diagnostic: current hydrostatics with stronger ellipsoid angular damping",
        ),
        Candidate(
            "too_soft_hydro",
            {
                "buoyancy_scale": 1.0000,
                "cob_torque_scale": 0.25,
                "cob_z_offset": 0.006,
            },
            note="diagnostic: under-restored hydrostatics",
        ),
    ]


__all__ = ["diagnostic_candidates"]
