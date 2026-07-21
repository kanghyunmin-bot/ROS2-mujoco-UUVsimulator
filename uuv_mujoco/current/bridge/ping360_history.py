"""Ping360 polar history buffers."""

from __future__ import annotations

import numpy as np

from .ping360_types import PING360_GRADS_PER_REV


class Ping360History:
    """Own mutable polar image/range/intensity history buffers."""

    def __init__(self, number_of_samples: int = 1) -> None:
        self.image = np.zeros((PING360_GRADS_PER_REV, 1), dtype=np.uint8)
        self.ranges = np.full(PING360_GRADS_PER_REV, np.inf, dtype=np.float32)
        self.intensities = np.zeros(PING360_GRADS_PER_REV, dtype=np.float32)
        self.resize(number_of_samples)

    def resize(self, number_of_samples: int) -> bool:
        samples = max(1, int(number_of_samples))
        if self.image.shape[1] == samples:
            return False
        self.image = np.zeros((PING360_GRADS_PER_REV, samples), dtype=np.uint8)
        self.ranges = np.full(PING360_GRADS_PER_REV, np.inf, dtype=np.float32)
        self.intensities = np.zeros(PING360_GRADS_PER_REV, dtype=np.float32)
        return True

    def record(self, *, angle_grad: int, profile: np.ndarray, nearest_range: float | None, peak_intensity: float) -> None:
        self.image[int(angle_grad), :] = profile
        self.ranges[int(angle_grad)] = nearest_range if nearest_range is not None else np.inf
        self.intensities[int(angle_grad)] = float(peak_intensity)

    def image_copy(self) -> np.ndarray:
        return self.image.copy()

    def ranges_copy(self) -> np.ndarray:
        return self.ranges.copy()

    def intensities_copy(self) -> np.ndarray:
        return self.intensities.copy()


__all__ = ["Ping360History"]
