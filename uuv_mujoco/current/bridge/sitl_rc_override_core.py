"""Core RC override normalization and MAVLink send helpers."""

from __future__ import annotations

from sim.contracts import normalize_ardusub_rc_override


def _normalize_rc_override_values(
    self,
    pwm_values: list[int],
) -> list[int]:
    return normalize_ardusub_rc_override(pwm_values)


def _send_rc_channels_override(
    self,
    mav,
    target_sys: int,
    target_comp: int,
    values: list[int],
) -> int:
    link = self._command_link_for_mav(mav)
    if link is None:
        return 0
    return link.send_rc_channels_override(target_sys, target_comp, values)


__all__ = ["_normalize_rc_override_values", "_send_rc_channels_override"]
