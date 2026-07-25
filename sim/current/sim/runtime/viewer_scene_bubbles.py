"""Bubble-stream visual effect for MuJoCo viewer thrusters."""

from __future__ import annotations

import numpy as np


def bubble_stream_basis(normalize, exhaust_dir) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    d = normalize(exhaust_dir)
    up = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    if abs(float(np.dot(d, up))) > 0.9:
        up = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    s1 = normalize(np.cross(d, up))
    s2 = normalize(np.cross(d, s1))
    return d, s1, s2


def bubble_stream_strength(thrust_mag: float, thrust_force_max: float) -> float:
    return min(1.0, float(thrust_mag) / max(float(thrust_force_max), 1e-6))


def add_bubble_stream_visual(
    *,
    add_sphere,
    normalize,
    start,
    exhaust_dir,
    thrust_mag: float,
    thrust_force_max: float,
    sim_time: float,
) -> None:
    if thrust_mag < 2.0:
        return
    d, s1, s2 = bubble_stream_basis(normalize, exhaust_dir)
    strength = bubble_stream_strength(thrust_mag, thrust_force_max)
    count = 3 + int(3 * strength)
    for k in range(count):
        phase = (float(sim_time) * 2.4 + k * 0.31) % 1.0
        dist = 0.04 + phase * (0.22 + 0.08 * strength)
        swirl = 0.008 * (1.0 - phase) * (0.6 + 0.4 * strength)
        wobble = np.sin(2.0 * np.pi * (phase + 0.17 * k))
        wobble2 = np.cos(2.0 * np.pi * (phase + 0.11 * k))
        pos = start + d * dist + s1 * (swirl * wobble) + s2 * (swirl * wobble2)
        radius = 0.004 + 0.003 * strength * (1.0 - 0.5 * phase)
        alpha = 0.35 * (1.0 - phase)
        add_sphere(pos, radius, (0.82, 0.93, 1.0, alpha))


__all__ = [
    "add_bubble_stream_visual",
    "bubble_stream_basis",
    "bubble_stream_strength",
]
