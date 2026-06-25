"""GUI RC axis mapping helpers."""

from __future__ import annotations

from .gui_axis_normalization import normalize_axes


def gui_rc_to_override_axes(
    *,
    forward: float,
    lateral: float,
    heave: float,
    yaw: float,
) -> tuple[float, float, float, float]:
    """Map GUI stick axes directly to RC override axes."""
    return normalize_axes(
        forward=forward,
        lateral=lateral,
        heave=heave,
        yaw=yaw,
    )


__all__ = ["gui_rc_to_override_axes"]
