"""Profile direct-gain override application."""

from __future__ import annotations

from typing import Any, Callable, MutableMapping, Optional

from .thruster_direct_gain_values import parse_direct_gain_scale
from .thruster_param_common import is_number, log_optional


def apply_profile_direct_gain_scales(
    profile_direct_gain_scales: Any,
    thruster_direct_scale: MutableMapping[str, float],
    *,
    log: Optional[Callable[[str], None]] = None,
) -> None:
    if not isinstance(profile_direct_gain_scales, dict):
        return

    applied: list[str] = []
    for thr_name, raw_value in profile_direct_gain_scales.items():
        if thr_name not in thruster_direct_scale or not is_number(raw_value):
            continue
        value = parse_direct_gain_scale(raw_value)
        if value is None:
            continue
        thruster_direct_scale[thr_name] = value
        applied.append(f"{thr_name}={value:.3f}")

    if applied:
        log_optional(log, "[thruster] profile direct gain scales: " + ", ".join(applied))


__all__ = ["apply_profile_direct_gain_scales"]
