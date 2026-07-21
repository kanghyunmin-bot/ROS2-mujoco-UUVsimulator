"""Candidate catalog for roll stability sweeps."""

from __future__ import annotations

from roll_stability_candidate_diagnostics import diagnostic_candidates
from roll_stability_candidate_ellipsoid import ellipsoid_candidates
from roll_stability_candidate_hydrostatic import hydrostatic_candidates
from roll_stability_candidate_signs import sign_check_candidates
from roll_stability_candidate_types import Candidate


def base_candidates() -> list[Candidate]:
    return [
        *hydrostatic_candidates(),
        *ellipsoid_candidates(),
        *diagnostic_candidates(),
    ]


def default_candidates(include_sign_checks: bool) -> list[Candidate]:
    candidates = base_candidates()
    if include_sign_checks:
        candidates.extend(sign_check_candidates())
    return candidates


__all__ = ["base_candidates", "default_candidates", "sign_check_candidates"]
