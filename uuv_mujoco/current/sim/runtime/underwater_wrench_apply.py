"""Apply body-frame hydrodynamic wrenches to MuJoCo world-frame forces."""

from __future__ import annotations

import numpy as np


def immersed_fraction(submerged: float) -> float:
    return float(np.clip(submerged, 0.0, 1.0))


def apply_body_force_torque(
    *,
    data,
    base_id: int,
    base_rot: np.ndarray,
    force_body: np.ndarray,
    torque_body: np.ndarray,
    submerged: float,
) -> None:
    immersed = immersed_fraction(submerged)
    data.xfrc_applied[int(base_id), 0:3] += base_rot @ (immersed * force_body)
    data.xfrc_applied[int(base_id), 3:6] += base_rot @ (immersed * torque_body)


def apply_body_wrench(
    *,
    data,
    base_id: int,
    base_rot: np.ndarray,
    wrench_body: np.ndarray,
    submerged: float,
) -> None:
    apply_body_force_torque(
        data=data,
        base_id=base_id,
        base_rot=base_rot,
        force_body=wrench_body[:3],
        torque_body=wrench_body[3:],
        submerged=submerged,
    )


def apply_body_force(
    *,
    data,
    base_id: int,
    base_rot: np.ndarray,
    force_body: np.ndarray,
    submerged: float,
) -> None:
    data.xfrc_applied[int(base_id), 0:3] += base_rot @ (immersed_fraction(submerged) * force_body)


__all__ = [
    "apply_body_force",
    "apply_body_force_torque",
    "apply_body_wrench",
    "immersed_fraction",
]
