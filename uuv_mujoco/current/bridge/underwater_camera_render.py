"""Bounded water-path projection for ideal MuJoCo RGB/depth captures.

This is a homogeneous-water approximation. It models neither refraction nor
volumetric lights and does not change scene geometry, contacts, or viewer state.
"""

from __future__ import annotations

import math

import numpy as np


class CameraOpticalPathProjector:
    """Cache pinhole rays and project metric z-depth to water path lengths [m]."""

    def __init__(self, *, width: int, height: int, fovy_degrees: float) -> None:
        if width < 1 or height < 1:
            raise ValueError("image dimensions must be positive")
        if not math.isfinite(fovy_degrees) or not 0.0 < fovy_degrees < 180.0:
            raise ValueError("camera vertical FOV must be in (0, 180) degrees")
        focal = 0.5 * height / math.tan(math.radians(fovy_degrees) / 2.0)
        self._x = ((np.arange(width, dtype=np.float32) - (width - 1) / 2) / focal)[None, :]
        self._y = (-(np.arange(height, dtype=np.float32) - (height - 1) / 2) / focal)[:, None]
        self._ray_length = np.sqrt(1.0 + self._x * self._x + self._y * self._y)

    def project(
        self,
        depth_m: np.ndarray,
        *,
        camera_position_z_m: float,
        camera_rotation: np.ndarray,
        water_surface_z_m: float,
        max_path_length_m: float,
    ) -> np.ndarray:
        """Return aligned underwater ray-segment lengths [m], capped at a limit.

        MuJoCo depth is axial z-depth [m], not ray distance. Camera local axes
        are right, up, backward. Only the part below the horizontal surface
        contributes, so above-water pixels do not receive underwater haze.
        """
        depth = np.asarray(depth_m, dtype=np.float32)
        if depth.shape != self._ray_length.shape:
            raise ValueError("depth dimensions must match the cached camera rays")
        if not np.all(np.isfinite(depth)) or np.any(depth < 0):
            raise ValueError("depth must be finite and non-negative")
        if not math.isfinite(max_path_length_m) or max_path_length_m <= 0:
            raise ValueError("max_path_length_m must be finite and positive")
        if not math.isfinite(camera_position_z_m) or not math.isfinite(water_surface_z_m):
            raise ValueError("camera and surface heights must be finite")
        rotation = np.asarray(camera_rotation, dtype=np.float32).reshape(3, 3)
        if not np.all(np.isfinite(rotation)):
            raise ValueError("camera rotation must be finite")
        # dz is the world-height change per metre of axial depth. Intersect
        # in axial coordinates before converting to ray length; clipping depth
        # first would discard water reached by a camera high above the surface.
        dz = self._x * rotation[2, 0] + self._y * rotation[2, 1] - rotation[2, 2]
        height_to_surface = float(water_surface_z_m - camera_position_z_m)
        crossing = np.zeros_like(dz)
        np.divide(height_to_surface, dz, out=crossing, where=np.abs(dz) > 1e-12)
        if height_to_surface >= 0.0:
            water_depth = np.where(dz > 1e-12, np.minimum(depth, np.maximum(crossing, 0)), depth)
        else:
            water_depth = np.where(dz < -1e-12, np.maximum(depth - crossing, 0), 0.0)
        return np.ascontiguousarray(
            np.minimum(water_depth * self._ray_length, max_path_length_m), dtype=np.float32
        )
