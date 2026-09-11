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

    def velocity_world_batch(
        self, positions_world_m: np.ndarray, time_s: float
    ) -> np.ndarray:
        """Return ambient plus wave velocities [m/s] at (N, 3) world positions [m]."""
        positions = self._positions_batch(positions_world_m)
        batch = getattr(self.current_field, "velocity_world_batch", None)
        velocity = (
            batch(positions, time_s)
            if batch is not None
            else np.asarray(
                [self.current_field.velocity_world(p, time_s) for p in positions]
            ).reshape(-1, 3)
        )
        # Harmonic waves retain their existing dry-point and finite-depth rules.
        # Flat/disabled water has exactly zero orbital flow and needs no point loop.
        if self.free_surface.active and self.free_surface.mode != "flat":
            velocity = velocity + np.asarray(
                [
                    self.free_surface.orbital_velocity_world_mps(*p, float(time_s))
                    for p in positions
                ]
            ).reshape(-1, 3)
        if velocity.shape != positions.shape or not np.all(np.isfinite(velocity)):
            raise FloatingPointError("water environment returned a non-finite velocity")
        return velocity

    def surface_height_world_m_batch(
        self, positions_world_m: np.ndarray, time_s: float
    ) -> np.ndarray:
        """Return boundary heights [m] at (N, 3) world positions [m]."""
        positions = self._positions_batch(positions_world_m)
        if not self.free_surface.active:
            return np.full(len(positions), self.fallback_surface_height_world_m)
        if self.free_surface.mode == "flat":
            # Call once to preserve the free-surface query validation contract.
            height = self.free_surface.height_world_m(0.0, 0.0, float(time_s))
            return np.full(len(positions), height)
        return np.asarray(
            [
                self.free_surface.height_world_m(
                    float(p[0]), float(p[1]), float(time_s)
                )
                for p in positions
            ]
        )

    @staticmethod
    def _positions_batch(positions_world_m: np.ndarray) -> np.ndarray:
        positions = np.asarray(positions_world_m, dtype=np.float64)
        if (
            positions.ndim != 2
            or positions.shape[1] != 3
            or not np.all(np.isfinite(positions))
        ):
            raise ValueError(
                "positions_world_m must have shape (N, 3) with finite values"
            )
        return positions


__all__ = ["WaterEnvironmentRuntime"]
