"""Hydrodynamics profile config helpers for runtime physics setup."""

from __future__ import annotations

from typing import Any

from physics.sim_profile_helpers import build_hydrodynamics_config


def performance_force_max(np_module: Any, perf_cfg: dict[str, Any]) -> float | None:
    if perf_cfg.get("active") and perf_cfg.get("force").size > 0:
        perf_max = float(np_module.max(np_module.abs(perf_cfg["force"])))
        if perf_max > 0.0:
            return perf_max
    return None


def build_runtime_hydrodynamics_config(
    *,
    np_module: Any,
    sim_profile: dict[str, Any],
    perf_cfg: dict[str, Any],
    scene_fluid_density: float,
) -> Any:
    return build_hydrodynamics_config(
        sim_profile,
        perf_force_max=performance_force_max(np_module, perf_cfg),
        fluid_density=scene_fluid_density,
    )


__all__ = ["build_runtime_hydrodynamics_config", "performance_force_max"]
