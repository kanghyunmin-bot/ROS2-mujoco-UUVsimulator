"""Servo sign diagnostic roll-stability sweep candidates."""

from __future__ import annotations

from roll_stability_candidate_types import Candidate


def sign_check_candidates() -> list[Candidate]:
    return [
        Candidate(
            "vertical_signs_inverted_check",
            servo_signs=(-1, -1, 1, 1, 1, -1, -1, 1),
            note="diagnostic only: invert vertical PWM-to-force signs",
        ),
        Candidate(
            "qgc_reverse_removed_check",
            servo_signs=(-1, -1, -1, -1, 1, 1, 1, 1),
            note="diagnostic only: remove simulated QGC reverse compensation for motors 3/4/5/8",
        ),
        Candidate(
            "horizontal_yaw_signs_inverted_check",
            servo_signs=(1, 1, -1, -1, -1, 1, 1, -1),
            note="diagnostic only: invert horizontal/yaw PWM-to-force signs",
        ),
    ]


__all__ = ["sign_check_candidates"]
