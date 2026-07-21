"""Polar image lookup construction for Ping360 rendering."""

from __future__ import annotations

import numpy as np

from .ping360_types import PING360_GRADS_PER_REV


def grad_distance(angle_idx: np.ndarray, grad: int) -> np.ndarray:
    forward = (angle_idx.astype(np.int16) - int(grad)) % PING360_GRADS_PER_REV
    backward = (int(grad) - angle_idx.astype(np.int16)) % PING360_GRADS_PER_REV
    return np.minimum(forward, backward)


def build_polar_lookup(image_size: int, number_of_samples: int) -> dict[str, np.ndarray]:
    size = int(np.clip(image_size, 128, 1200))
    samples = max(1, int(number_of_samples))
    yy, xx = np.indices((size, size), dtype=np.float32)
    center = (float(size) - 1.0) * 0.5
    radius_px = max(center - 1.0, 1.0)
    dx = xx - center
    dy = center - yy
    rr = np.sqrt(dx * dx + dy * dy) / radius_px
    mask = rr <= 1.0
    angle = np.mod(np.arctan2(dy, dx), 2.0 * np.pi)
    grad_float = angle * float(PING360_GRADS_PER_REV) / (2.0 * np.pi)
    angle_idx = np.floor(grad_float + 0.5).astype(np.int16) % PING360_GRADS_PER_REV
    range_idx = np.clip(np.floor(rr * float(samples - 1) + 0.5), 0, samples - 1).astype(np.int32)

    ring_width = max(1.2 / radius_px, 0.002)
    ring_mask = np.zeros_like(mask, dtype=bool)
    for frac in (0.25, 0.50, 0.75, 1.0):
        ring_mask |= mask & (np.abs(rr - frac) <= ring_width)
    spoke_dist = np.abs(((grad_float + 25.0) % 50.0) - 25.0)
    spoke_mask = mask & (rr > 0.04) & (spoke_dist <= 0.45)

    return {
        "mask": mask,
        "angle_idx": angle_idx,
        "range_idx": range_idx,
        "ring_mask": ring_mask,
        "spoke_mask": spoke_mask,
        "rr": rr,
    }


__all__ = ["build_polar_lookup", "grad_distance"]
