"""Array allocation helpers for dynamic MuJoCo fluid coefficients."""

from __future__ import annotations

import numpy as np

from sim.physics.dynamic_fluidcoef_types import DynamicFluidcoefSetup


def allocate_dynamic_fluidcoef_arrays(model) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    base = model.geom_fluid[:, 1:6].copy()
    current = base.copy()
    reference = base.copy()
    weights = np.zeros_like(base)
    axis_weights = np.zeros((model.ngeom, 5, 3), dtype=np.float64)
    angular_axis_weights = np.zeros((model.ngeom, 5, 3), dtype=np.float64)
    return base, current, reference, weights, axis_weights, angular_axis_weights


def build_dynamic_fluidcoef_setup_result(
    *,
    cfg: dict,
    enabled: bool,
    base: np.ndarray,
    current: np.ndarray,
    reference: np.ndarray,
    weights: np.ndarray,
    axis_weights: np.ndarray,
    angular_axis_weights: np.ndarray,
    active_geom_ids: set[int],
) -> DynamicFluidcoefSetup:
    return DynamicFluidcoefSetup(
        cfg=cfg,
        enabled=enabled,
        base=base,
        current=current,
        reference=reference,
        weights=weights,
        axis_weights=axis_weights,
        angular_axis_weights=angular_axis_weights,
        active_geom_ids=active_geom_ids,
    )


__all__ = ["allocate_dynamic_fluidcoef_arrays", "build_dynamic_fluidcoef_setup_result"]
