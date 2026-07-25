"""Shared type aliases for static ROS2 TF specs."""

from __future__ import annotations

import numpy as np

TfSpec = tuple[str, str, np.ndarray, np.ndarray]


__all__ = ["TfSpec"]
