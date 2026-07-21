"""Status helpers for the public Ping360Simulator facade."""

from __future__ import annotations

from typing import Any

from .ping360_samples import status_payload
from .ping360_sim_lifecycle import ping360_simulator_active, refresh_ping360_simulator_runtime


def current_ping360_angle_grad(owner: object) -> int:
    return owner._runtime.sweep.current_angle_grad(owner.settings)


def ping360_status_dict(owner: object, sim_t: float | None = None) -> dict[str, Any]:
    refresh_ping360_simulator_runtime(owner)
    return status_payload(
        latest=owner._latest,
        sim_t=sim_t,
        angle_grad=current_ping360_angle_grad(owner),
        ping_number=owner._ping_number,
        settings=owner.settings,
        active=ping360_simulator_active(owner),
    )


__all__ = ["current_ping360_angle_grad", "ping360_status_dict"]
