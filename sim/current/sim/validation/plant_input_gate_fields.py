"""CSV field selection helpers for plant-input validation gates."""

from __future__ import annotations

from typing import Iterable


TIME_FIELD_NAMES = {"t", "t_s", "time", "time_s", "source_t_s"}
PWM_FIELD_PREFIXES = ("servo", "pwm", "rcout", "rc_out", "ch")


def candidate_pwm_fields(fieldnames: Iterable[str]) -> list[str]:
    out: list[str] = []
    for field in fieldnames:
        name = field.lower()
        if name in TIME_FIELD_NAMES:
            continue
        if name.startswith(PWM_FIELD_PREFIXES):
            out.append(field)
    return out


__all__ = ["PWM_FIELD_PREFIXES", "TIME_FIELD_NAMES", "candidate_pwm_fields"]
