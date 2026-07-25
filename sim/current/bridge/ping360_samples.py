"""Ping360 sample and status payload helpers."""

from __future__ import annotations

from typing import Any

import numpy as np

from .ping360_history import Ping360History
from .ping360_types import PING360_DEG_PER_GRAD, Ping360EffectiveSettings, Ping360Sample


def held_sample(
    *,
    latest: Ping360Sample,
    sim_t: float,
    settings: Ping360EffectiveSettings,
) -> Ping360Sample:
    return Ping360Sample(
        sim_time_s=float(sim_t),
        angle_grad=latest.angle_grad,
        profile=latest.profile,
        image=latest.image,
        ranges_m=latest.ranges_m,
        intensities=latest.intensities,
        settings=settings,
        ping_number=latest.ping_number,
        updated=False,
    )


def updated_sample(
    *,
    sim_t: float,
    angle_grad: int,
    profile: np.ndarray,
    history: Ping360History,
    settings: Ping360EffectiveSettings,
    ping_number: int,
) -> Ping360Sample:
    return Ping360Sample(
        sim_time_s=float(sim_t),
        angle_grad=int(angle_grad),
        profile=profile,
        image=history.image_copy(),
        ranges_m=history.ranges_copy(),
        intensities=history.intensities_copy(),
        settings=settings,
        ping_number=int(ping_number),
        updated=True,
    )


def status_payload(
    *,
    latest: Ping360Sample | None,
    sim_t: float | None,
    angle_grad: int,
    ping_number: int,
    settings: Ping360EffectiveSettings,
    active: bool,
) -> dict[str, Any]:
    if latest is not None:
        payload = latest.status_dict()
        payload["updated"] = False
        payload["sim_time_s"] = float(sim_t) if sim_t is not None else payload["sim_time_s"]
    else:
        payload = {
            "sim_time_s": float(sim_t) if sim_t is not None else 0.0,
            "angle_grad": int(angle_grad),
            "angle_deg": int(angle_grad) * PING360_DEG_PER_GRAD,
            "ping_number": int(ping_number),
            "settings": settings.as_dict(),
            "updated": False,
        }
    payload["active"] = bool(active)
    return payload


__all__ = ["held_sample", "status_payload", "updated_sample"]
