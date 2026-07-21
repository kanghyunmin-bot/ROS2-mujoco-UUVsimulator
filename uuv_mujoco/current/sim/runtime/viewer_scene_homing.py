"""Pinger homing direction overlay for the MuJoCo viewer."""

from __future__ import annotations

from dataclasses import dataclass
import os
import time
from typing import Any

import numpy as np


RED = (1.0, 0.02, 0.02, 1.0)


@dataclass(frozen=True)
class HomingDirectionArrow:
    start: np.ndarray
    end: np.ndarray


def _arrow_enabled() -> bool:
    return os.getenv("UUV_MUJOCO_HOMING_ARROW_ENABLE", "1").strip().lower() not in {
        "0",
        "false",
        "no",
        "off",
    }


def build_homing_direction_arrow(
    bridge: Any,
    data: Any,
    base_rot: np.ndarray,
    *,
    stale_after_s: float = 1.5,
    arrow_length_m: float = 2.0,
) -> HomingDirectionArrow | None:
    """Convert the latest hydrophone-estimated body direction into a world arrow."""

    if bridge is None or not _arrow_enabled():
        return None
    if not bool(getattr(bridge, "_hydrophone_last_estimated_direction_active", False)):
        return None

    sample_wall = float(getattr(bridge, "_hydrophone_last_estimated_direction_wall", float("-inf")))
    age_s = time.monotonic() - sample_wall
    if not np.isfinite(age_s) or age_s < 0.0 or age_s > max(float(stale_after_s), 0.0):
        return None

    direction_body = np.asarray(
        getattr(bridge, "_hydrophone_last_estimated_direction_body", (0.0, 0.0, 0.0)),
        dtype=np.float64,
    )
    if direction_body.shape != (3,) or not np.all(np.isfinite(direction_body)):
        return None
    direction_world = np.asarray(base_rot, dtype=np.float64).reshape(3, 3) @ direction_body
    norm = float(np.linalg.norm(direction_world))
    if norm <= 1.0e-9:
        return None
    direction_world /= norm

    site_id = int(getattr(bridge, "_hydrophone_center_site_id", -1))
    if site_id < 0:
        return None
    try:
        center = np.asarray(data.site_xpos[site_id], dtype=np.float64).copy()
    except (IndexError, TypeError, ValueError):
        return None
    if center.shape != (3,) or not np.all(np.isfinite(center)):
        return None

    length_m = float(np.clip(arrow_length_m, 0.25, 5.0))
    start = center + direction_world * 0.12
    return HomingDirectionArrow(start=start, end=start + direction_world * length_m)


def draw_homing_direction(runtime: Any, scene: Any, *, base_rot: np.ndarray) -> None:
    bridge = runtime.get_ros_bridge()
    arrow = build_homing_direction_arrow(bridge, runtime.data, base_rot)
    if arrow is None:
        return
    scene.add_arrow_segment(arrow.start, arrow.end, RED, thickness=0.055)


__all__ = [
    "HomingDirectionArrow",
    "build_homing_direction_arrow",
    "draw_homing_direction",
]
