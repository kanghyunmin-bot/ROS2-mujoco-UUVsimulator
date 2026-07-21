"""Ping360 polar image intensity and overlay layers."""

from __future__ import annotations

import numpy as np

from .ping360_image_lookup import grad_distance
from .ping360_types import Ping360Config, Ping360Sample


def initialize_polar_canvas(size: int, mask: np.ndarray) -> np.ndarray:
    out = np.zeros((size, size), dtype=np.uint8)
    out[mask] = 5
    return out


def apply_return_intensity(
    out: np.ndarray,
    *,
    lookup: dict[str, np.ndarray],
    raw: np.ndarray,
    config: Ping360Config,
) -> None:
    mask = lookup["mask"]
    values = raw[lookup["angle_idx"][mask], lookup["range_idx"][mask]].astype(np.float32)
    noise_gate = float(config.noise_floor) + 0.75 * float(config.speckle_std)
    display_gain = float(max(config.image_display_gain, 0.0))
    returns = np.clip((values - noise_gate) * display_gain, 0.0, 255.0).astype(np.uint8)
    out[mask] = np.maximum(out[mask], returns)


def apply_reference_grid(out: np.ndarray, lookup: dict[str, np.ndarray]) -> None:
    spoke_mask = lookup["spoke_mask"]
    ring_mask = lookup["ring_mask"]
    out[spoke_mask] = np.maximum(out[spoke_mask], 18)
    out[ring_mask] = np.maximum(out[ring_mask], 30)


def apply_angle_overlays(
    out: np.ndarray,
    *,
    lookup: dict[str, np.ndarray],
    sample: Ping360Sample,
) -> None:
    mask = lookup["mask"]
    rr = lookup["rr"]
    angle_idx = lookup["angle_idx"]
    for boundary_grad in (sample.settings.start_angle_grad, sample.settings.stop_angle_grad):
        boundary_mask = mask & (rr > 0.06) & (grad_distance(angle_idx, int(boundary_grad)) <= 1)
        out[boundary_mask] = np.maximum(out[boundary_mask], 55)

    sweep_mask = mask & (rr > 0.04) & (grad_distance(angle_idx, int(sample.angle_grad)) <= 1)
    out[sweep_mask] = np.maximum(out[sweep_mask], 145)


def apply_blind_zone(
    out: np.ndarray,
    *,
    lookup: dict[str, np.ndarray],
    sample: Ping360Sample,
    raw: np.ndarray,
    size: int,
) -> None:
    mask = lookup["mask"]
    rr = lookup["rr"]
    blind_radius = float(sample.settings.blind_bins) / max(float(raw.shape[1] - 1), 1.0)
    if blind_radius <= 0.0:
        return
    out[mask & (rr <= blind_radius)] = 0
    blind_fade_width = max(4.0 / max(float(size), 1.0), 0.035)
    fade_mask = mask & (rr > blind_radius) & (rr <= blind_radius + blind_fade_width)
    if not np.any(fade_mask):
        return
    fade = (rr[fade_mask] - blind_radius) / blind_fade_width
    out[fade_mask] = np.clip(out[fade_mask].astype(np.float32) * fade, 0.0, 255.0).astype(np.uint8)


def render_ping360_polar_layers(
    *,
    raw: np.ndarray,
    lookup: dict[str, np.ndarray],
    sample: Ping360Sample,
    config: Ping360Config,
    size: int,
) -> np.ndarray:
    out = initialize_polar_canvas(size, lookup["mask"])
    apply_return_intensity(out, lookup=lookup, raw=raw, config=config)
    apply_reference_grid(out, lookup)
    apply_angle_overlays(out, lookup=lookup, sample=sample)
    apply_blind_zone(out, lookup=lookup, sample=sample, raw=raw, size=size)
    out[~lookup["mask"]] = 0
    return out


__all__ = [
    "apply_angle_overlays",
    "apply_blind_zone",
    "apply_reference_grid",
    "apply_return_intensity",
    "initialize_polar_canvas",
    "render_ping360_polar_layers",
]
