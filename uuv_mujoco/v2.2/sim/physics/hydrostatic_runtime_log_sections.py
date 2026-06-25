"""Hydrostatic runtime log sections."""

from __future__ import annotations

from typing import Any, Callable

from .hydrostatic_runtime_types import HydrostaticRuntimeValues


def log_hydrostatic_application(
    *,
    active_body_components: tuple[Any, ...],
    active_buoyancy_points: tuple[Any, ...],
    hydrostatic_source_used: str,
    log: Callable[[str], None],
) -> None:
    if active_buoyancy_points:
        log(
            f"[physics] buoyancy application: distributed 4-point model "
            f"({len(active_buoyancy_points)} points, source={hydrostatic_source_used})"
        )
    elif active_body_components:
        log(
            f"[physics] buoyancy application: distributed component model "
            f"({len(active_body_components)} components, source={hydrostatic_source_used})"
        )
    else:
        log("[physics] buoyancy application: single-point CoB/center model")


def log_hydrostatic_restoring(
    *,
    values: HydrostaticRuntimeValues,
    real_start_required: bool,
    log: Callable[[str], None],
) -> None:
    if not values.hydrostatic_restoring_active:
        return

    log(
        "[physics] hydrostatic restoring stiffness: "
        f"roll={values.hydrostatic_restoring_roll_stiffness:.3f} "
        f"pitch={values.hydrostatic_restoring_pitch_stiffness:.3f} N*m/rad "
        f"trim_roll={values.hydrostatic_restoring_roll_trim_rad:+.4f}rad "
        f"trim_pitch={values.hydrostatic_restoring_pitch_trim_rad:+.4f}rad "
        "(separate from buoyancy force position)"
    )
    if values.hydrostatic_restoring_release_trim_blend_s > 1.0e-9 and real_start_required:
        log(
            "[physics] hydrostatic release trim blend: "
            f"{values.hydrostatic_restoring_release_trim_blend_s:.3f}s "
            f"from real_start roll={values.real_start_restoring_roll_trim_rad:+.4f}rad "
            f"pitch={values.real_start_restoring_pitch_trim_rad:+.4f}rad "
            f"to profile roll={values.hydrostatic_restoring_profile_roll_trim_rad:+.4f}rad "
            f"pitch={values.hydrostatic_restoring_profile_pitch_trim_rad:+.4f}rad"
        )


__all__ = ["log_hydrostatic_application", "log_hydrostatic_restoring"]
