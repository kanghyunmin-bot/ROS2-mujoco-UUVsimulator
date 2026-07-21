"""Joint-array extraction helpers for MuJoCo thruster debug output."""

from __future__ import annotations

import numpy as np


def free_joint_values(data: object, *, world_qvel_adr: int, attr_name: str) -> np.ndarray:
    values = getattr(data, attr_name, None)
    if values is None:
        return np.zeros(6, dtype=np.float64)
    arr = np.asarray(values, dtype=np.float64)
    end = int(world_qvel_adr) + 6
    if arr.ndim != 1 or arr.size < end:
        return np.zeros(6, dtype=np.float64)
    return arr[int(world_qvel_adr) : end].copy()


__all__ = ["free_joint_values"]
