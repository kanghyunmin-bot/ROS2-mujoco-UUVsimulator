"""Hydrostatic runtime model mutation and logging."""

from __future__ import annotations

from typing import Any, Callable

from .hydrostatic_cob_reporting import align_cob_site, log_cob_runtime_override
from .hydrostatic_runtime_types import HydrostaticRuntimeValues
from .hydrostatic_runtime_log_sections import log_hydrostatic_application, log_hydrostatic_restoring


def log_hydrostatic_runtime(
    *,
    values: HydrostaticRuntimeValues,
    active_body_components: tuple[Any, ...],
    active_buoyancy_points: tuple[Any, ...],
    hydrostatic_source_used: str,
    real_start_required: bool,
    log: Callable[[str], None],
) -> None:
    log_hydrostatic_application(
        active_body_components=active_body_components,
        active_buoyancy_points=active_buoyancy_points,
        hydrostatic_source_used=hydrostatic_source_used,
        log=log,
    )
    log_hydrostatic_restoring(values=values, real_start_required=real_start_required, log=log)


__all__ = [
    "log_cob_runtime_override",
    "align_cob_site",
    "log_hydrostatic_application",
    "log_hydrostatic_restoring",
    "log_hydrostatic_runtime",
]
