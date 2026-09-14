"""Deterministic, bounded pool lighting approximations (no reflected scene pass)."""

from __future__ import annotations

import cv2
import numpy as np


def wave_field(
    x: np.ndarray, y: np.ndarray, time_s: float
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return surface displacement [m] and two dimensionless slopes."""
    height = np.zeros_like(x + y, dtype=np.float32)
    dx = np.zeros_like(height)
    dy = np.zeros_like(height)
    for amplitude, kx, ky, speed in (
        (0.009, 5.1, 1.8, 1.7),
        (0.005, -2.7, 8.2, 2.1),
        (0.002, 17.0, 11.0, 3.2),
    ):
        phase = kx * x + ky * y - speed * time_s
        height += amplitude * np.sin(phase)
        slope = amplitude * np.cos(phase)
        dx += kx * slope
        dy += ky * slope
    return height, dx, dy


class PoolWaterLighting:
    """Project world-anchored waves, highlights and depth lighting onto RGB."""

    def __init__(self, width: int, height: int, fovy_degrees: float) -> None:
        focal = float(height / (2 * np.tan(np.deg2rad(fovy_degrees) / 2)))
        self.x = ((np.arange(width, dtype=np.float32) - (width - 1) / 2) / focal)[
            None, :
        ]
        self.y = (-(np.arange(height, dtype=np.float32) - (height - 1) / 2) / focal)[
            :, None
        ]

    def apply(
        self,
        rgb: np.ndarray,
        depth: np.ndarray,
        *,
        position: np.ndarray,
        rotation: np.ndarray,
        time_s: float,
        surface_z: float,
        center_xy: np.ndarray,
        half_size_xy: np.ndarray,
    ) -> np.ndarray:
        """Shade aligned RGB/depth [m] without changing simulation or depth buffers.

        This is an artistic homogeneous-water prior: no refracted rays, reflected
        objects, volumetric ray marching or light-source occlusion are evaluated.
        """
        position = np.asarray(position, dtype=np.float32)
        depth = np.asarray(depth, dtype=np.float32)
        rot = np.asarray(rotation, dtype=np.float32).reshape(3, 3)
        rays = [self.x * rot[i, 0] + self.y * rot[i, 1] - rot[i, 2] for i in range(3)]
        points = [position[i] + depth * rays[i] for i in range(3)]
        inside = (np.abs(points[0] - center_xy[0]) < half_size_xy[0] + 0.01) & (
            np.abs(points[1] - center_xy[1]) < half_size_xy[1] + 0.01
        )
        immersion = np.clip(surface_z - points[2], 0, 20)
        wet = inside & (immersion > 0)
        # Incoming light diminishes separately from the camera-path attenuation.
        transmission = np.exp(-0.32 * immersion)
        # Only smooth lighting fields use half resolution; RGB and occlusion
        # retain their original resolution, including thin ropes and silhouettes.
        px, py = points[0][::2, ::2], points[1][::2, ::2]
        phase_a = 5.1 * px + 1.8 * py - 1.7 * time_s
        phase_b = -2.7 * px + 8.2 * py - 2.1 * time_s
        caustic = np.maximum(0.001, np.cos(phase_a + 0.6 * np.sin(phase_b))) ** 12
        caustic = cv2.resize(
            caustic, (rgb.shape[1], rgb.shape[0]), interpolation=cv2.INTER_LINEAR
        )
        gain = np.where(
            wet, 0.70 + 0.30 * transmission + 0.22 * caustic * transmission, 1
        )
        out = rgb.astype(np.float32) * gain[..., None]
        # Intersect only visible water within the pool; foreground stays intact.
        crossing = np.full_like(depth, -1)
        np.divide(
            surface_z - position[2], rays[2], out=crossing, where=np.abs(rays[2]) > 1e-5
        )
        sx = position[0] + crossing * rays[0]
        sy = position[1] + crossing * rays[1]
        valid = (
            (crossing > 0)
            & (crossing < depth)
            & (np.abs(sx - center_xy[0]) < half_size_xy[0])
            & (np.abs(sy - center_xy[1]) < half_size_xy[1])
        )
        if np.any(valid):
            _, dx, dy = wave_field(
                np.where(valid, sx, 0)[::2, ::2],
                np.where(valid, sy, 0)[::2, ::2],
                time_s,
            )
            dx = cv2.resize(
                dx, (rgb.shape[1], rgb.shape[0]), interpolation=cv2.INTER_LINEAR
            )
            dy = cv2.resize(
                dy, (rgb.shape[1], rgb.shape[0]), interpolation=cv2.INTER_LINEAR
            )
            ray_norm = np.sqrt(sum(ray * ray for ray in rays))
            # A broad overhead source avoids subpixel glitter and temporal aliasing.
            hx, hy = -0.15 - rays[0] / ray_norm, -0.10 - rays[1] / ray_norm
            hz = 1.0 + np.abs(rays[2]) / ray_norm
            normal_half = (-dx * hx - dy * hy + hz) / np.sqrt(
                (1 + dx * dx + dy * dy) * (hx * hx + hy * hy + hz * hz)
            )
            sparkle = np.clip(normal_half, 0.5, 1) ** 80
            grazing = (1 - np.abs(rays[2]) / ray_norm) ** 5
            alpha = np.where(valid, 0.06 + 0.16 * grazing, 0)
            out = out * (1 - alpha[..., None]) + alpha[..., None] * np.array(
                [55, 135, 155], np.float32
            )
            out += (valid * sparkle * 85)[..., None] * np.array(
                [1.0, 0.97, 0.86], np.float32
            )
        return np.ascontiguousarray(np.clip(out, 0, 255), dtype=np.uint8)
