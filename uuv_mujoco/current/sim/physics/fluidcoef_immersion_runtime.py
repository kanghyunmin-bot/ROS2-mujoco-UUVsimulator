"""Waterline scaling for MuJoCo's built-in ellipsoid fluid coefficients."""

from __future__ import annotations

from collections.abc import Callable

import numpy as np

from physics.hydrodynamics_helpers import submerged_fraction


# Stable ``mjtGeom`` values from MuJoCo.  Keeping these local avoids importing
# the rendering/runtime extension in dependency-light contract tests.
_GEOM_SPHERE = 2
_GEOM_CAPSULE = 3
_GEOM_ELLIPSOID = 4
_GEOM_CYLINDER = 5
_GEOM_BOX = 6


class FluidcoefImmersionRuntime:
    """Keep built-in fluid forces continuous when proxy geoms cross the surface.

    MuJoCo's passive ellipsoid fluid force otherwise remains active in air.
    The wrapper restores unscaled coefficients before an optional dynamic
    fluidcoef update, captures that update, then applies a per-geom immersed
    fraction exactly once for the next ``mj_step``.
    """

    def __init__(
        self,
        *,
        model,
        data,
        fluid_geom_ids,
        water_surface_z: float,
        enabled: bool,
        update_unscaled: Callable[[np.ndarray, np.ndarray], None],
    ) -> None:
        self.model = model
        self.data = data
        self.geom_ids = np.asarray(tuple(int(item) for item in fluid_geom_ids), dtype=np.int32)
        self.water_surface_z = float(water_surface_z)
        self.enabled = bool(enabled) and self.geom_ids.size > 0
        self.update_unscaled = update_unscaled
        self.surface_height_sampler: Callable[[np.ndarray, float], float] | None = None
        self.unscaled = (
            np.asarray(model.geom_fluid[self.geom_ids, 1:6], dtype=np.float64).copy()
            if self.geom_ids.size
            else np.zeros((0, 5), dtype=np.float64)
        )
        self.last_fractions = np.ones(self.geom_ids.size, dtype=np.float64)

    def set_surface_height_sampler(
        self,
        sampler: Callable[[np.ndarray, float], float] | None,
    ) -> None:
        """Bind the shared free-surface height sampler."""

        self.surface_height_sampler = sampler

    def update(self, rel_lin_vel_body: np.ndarray, ang_vel_body: np.ndarray) -> None:
        if not self.enabled:
            self.update_unscaled(rel_lin_vel_body, ang_vel_body)
            return
        # Undo our previous waterline scale so dynamic-fluidcoef logic always
        # sees its own coefficient domain and never compounds the fraction.
        self.model.geom_fluid[self.geom_ids, 1:6] = self.unscaled
        self.update_unscaled(rel_lin_vel_body, ang_vel_body)
        self.unscaled[:] = self.model.geom_fluid[self.geom_ids, 1:6]
        fractions = self._fractions()
        self.model.geom_fluid[self.geom_ids, 1:6] = self.unscaled * fractions[:, None]
        self.last_fractions[:] = fractions

    def _fractions(self) -> np.ndarray:
        fractions = np.empty(self.geom_ids.size, dtype=np.float64)
        for index, geom_id in enumerate(self.geom_ids):
            position_world = np.asarray(
                self.data.geom_xpos[int(geom_id)],
                dtype=np.float64,
            )
            center_z = float(position_world[2])
            half_height = self._vertical_half_extent(int(geom_id))
            if self.surface_height_sampler is None:
                surface_height = self.water_surface_z
            else:
                surface_height = float(
                    self.surface_height_sampler(
                        position_world.copy(),
                        float(self.data.time),
                    )
                )
                if not np.isfinite(surface_height):
                    raise ValueError("fluidcoef surface sampler must return a finite height")
            depth = surface_height - center_z
            fractions[index] = submerged_fraction(depth, half_height, "ellipsoid")
        return fractions

    def _vertical_half_extent(self, geom_id: int) -> float:
        size = np.abs(np.asarray(self.model.geom_size[geom_id, :3], dtype=np.float64))
        rotation = np.asarray(self.data.geom_xmat[geom_id], dtype=np.float64).reshape(3, 3)
        # World-Z expressed in the geom frame. MuJoCo's ``geom_size`` does not
        # use generic XYZ half-axes for cylinder/capsule geoms: size[0] is the
        # radius and size[1] is the half-length along local Z. Use each real
        # geom's support function so a pitched long body does not collapse to
        # the size[2] == 0 placeholder at the waterline.
        vertical_local = np.abs(rotation[2, :])
        geom_types = getattr(self.model, "geom_type", None)
        geom_type = int(geom_types[geom_id]) if geom_types is not None else -1
        if geom_type == _GEOM_SPHERE:
            extent = float(size[0])
        elif geom_type == _GEOM_ELLIPSOID:
            extent = float(np.linalg.norm(vertical_local * size))
        elif geom_type == _GEOM_CYLINDER:
            radial_projection = float(np.linalg.norm(vertical_local[:2]))
            extent = float(size[0] * radial_projection + size[1] * vertical_local[2])
        elif geom_type == _GEOM_CAPSULE:
            extent = float(size[0] + size[1] * vertical_local[2])
        elif geom_type == _GEOM_BOX:
            extent = float(np.dot(vertical_local, size))
        else:
            # Unknown fluid geom: a rotation-invariant bounding radius is a
            # conservative, continuous fallback. Dependency-light fakes may
            # not expose geom_rbound, in which case use the largest size.
            rbound = getattr(self.model, "geom_rbound", None)
            extent = float(rbound[geom_id]) if rbound is not None else float(np.max(size))
        return max(extent, 1.0e-4)


__all__ = ["FluidcoefImmersionRuntime"]
