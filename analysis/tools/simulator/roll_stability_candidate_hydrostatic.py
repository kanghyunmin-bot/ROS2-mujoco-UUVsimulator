"""Hydrostatic roll-stability sweep candidates."""

from __future__ import annotations

from roll_stability_candidate_types import Candidate


def hydrostatic_candidates() -> list[Candidate]:
    return [
        Candidate("baseline_current", note="current profile as-is"),
        Candidate(
            "restore_too_strong_check",
            {
                "buoyancy_scale": 1.015,
                "cob_torque_scale": 1.05,
                "cob_z_offset": 0.026,
            },
            note="previous stronger restoring candidate",
        ),
        Candidate(
            "restore_soft",
            {
                "buoyancy_scale": 1.003,
                "cob_torque_scale": 0.50,
                "cob_z_offset": 0.012,
            },
            note="lower GM/restoring stiffness",
        ),
        Candidate(
            "near_neutral_soft",
            {
                "buoyancy_scale": 1.0005,
                "cob_torque_scale": 0.50,
                "cob_z_offset": 0.012,
            },
            note="almost neutral buoyancy with soft restoring",
        ),
    ]


__all__ = ["hydrostatic_candidates"]
