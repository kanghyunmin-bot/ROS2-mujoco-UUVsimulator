"""Shared local water velocity and free-surface runtime contract."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class WaterEnvironmentRuntime:
    """Sample one current field and one free surface at arbitrary world points."""

    current_field: object
    free_surface: object
    fallback_surface_height_world_m: float

    def velocity_world(self, position_world_m: np.ndarray, time_s: float) -> np.ndarray:
        """Return ambient plus wave-orbital water velocity [m/s]."""

        position = np.asarray(position_world_m, dtype=np.float64)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            raise ValueError("position_world_m must contain three finite values")
        velocity = np.asarray(
            self.current_field.velocity_world(position, float(time_s)),
            dtype=np.float64,
        ).copy()
        if self.free_surface.active:
            velocity += self.free_surface.orbital_velocity_world_mps(
                float(position[0]),
                float(position[1]),
                float(position[2]),
                float(time_s),
            )
        if velocity.shape != (3,) or not np.all(np.isfinite(velocity)):
            raise FloatingPointError("water environment returned a non-finite velocity")
        return velocity

    def surface_height_world_m(self, position_world_m: np.ndarray, time_s: float) -> float:
        """Return the shared air-water boundary world-z height [m]."""

        position = np.asarray(position_world_m, dtype=np.float64)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            raise ValueError("position_world_m must contain three finite values")
        if not self.free_surface.active:
            return float(self.fallback_surface_height_world_m)
        return float(
            self.free_surface.height_world_m(
                float(position[0]),
                float(position[1]),
                float(time_s),
            )
        )


__all__ = ["WaterEnvironmentRuntime"]
