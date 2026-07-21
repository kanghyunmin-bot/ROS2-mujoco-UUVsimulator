"""Ping360 local beam direction generation."""

from __future__ import annotations

import math

import numpy as np

from .ping360_types import Ping360Config


def beam_directions_local(config: Ping360Config, angle_rad: float) -> list[tuple[np.ndarray, float]]:
    h_count = max(1, int(config.horizontal_ray_count))
    v_count = max(1, int(config.vertical_ray_count))
    h_span = math.radians(float(config.horizontal_beamwidth_deg))
    v_span = math.radians(float(config.vertical_beamwidth_deg))
    h_offsets = np.linspace(-0.5 * h_span, 0.5 * h_span, h_count)
    v_offsets = np.linspace(-0.5 * v_span, 0.5 * v_span, v_count)
    rays = [_beam_ray(angle_rad, float(h), float(v)) for h in h_offsets for v in v_offsets]
    return _normalize_ray_weights(rays)


def _beam_ray(angle_rad: float, h_offset: float, v_offset: float) -> tuple[np.ndarray, float]:
    yaw = float(angle_rad) + h_offset
    cv = math.cos(v_offset)
    local = np.array([math.cos(yaw) * cv, math.sin(yaw) * cv, math.sin(v_offset)], dtype=np.float64)
    weight = math.cos(h_offset) ** 2 * math.cos(v_offset) ** 2
    return local / max(np.linalg.norm(local), 1.0e-12), float(max(weight, 0.05))


def _normalize_ray_weights(rays: list[tuple[np.ndarray, float]]) -> list[tuple[np.ndarray, float]]:
    total = sum(weight for _, weight in rays)
    if total <= 1.0e-12:
        return rays
    return [(direction, weight / total) for direction, weight in rays]


__all__ = ["beam_directions_local"]
