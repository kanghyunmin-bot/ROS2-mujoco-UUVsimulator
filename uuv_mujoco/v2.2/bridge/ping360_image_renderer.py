"""Ping360 polar history renderer."""

from __future__ import annotations

import numpy as np

from .ping360_image_layers import render_ping360_polar_layers
from .ping360_image_lookup import build_polar_lookup
from .ping360_types import Ping360Config, Ping360Sample


class Ping360ImageRenderer:
    """Render Ping360 polar history to a mono8 image with cached index maps."""

    def __init__(self) -> None:
        self._lookup_key: tuple[int, int] | None = None
        self._lookup: dict[str, np.ndarray] | None = None

    def reset(self) -> None:
        self._lookup_key = None
        self._lookup = None

    def polar_lookup(self, image_size: int, number_of_samples: int) -> dict[str, np.ndarray]:
        size = int(np.clip(image_size, 128, 1200))
        samples = max(1, int(number_of_samples))
        key = (size, samples)
        if self._lookup_key == key and self._lookup is not None:
            return self._lookup

        lookup = build_polar_lookup(size, samples)
        self._lookup_key = key
        self._lookup = lookup
        return lookup

    def render_polar_image(self, sample: Ping360Sample, config: Ping360Config) -> np.ndarray:
        raw = np.asarray(sample.image, dtype=np.uint8)
        size = int(np.clip(config.image_size_px, 128, 1200))
        lookup = self.polar_lookup(size, raw.shape[1])
        return render_ping360_polar_layers(
            raw=raw,
            lookup=lookup,
            sample=sample,
            config=config,
            size=size,
        )


__all__ = ["Ping360ImageRenderer"]
