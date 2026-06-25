"""GUI command-readiness helpers for UuvGuiNode."""

from __future__ import annotations

from typing import Any

from .config import BACKEND_SIM_BRIDGE
from .models import TelemetrySnapshot
from .node_backend_runtime import effective_backend, service_ready
from .node_readiness_inputs import build_command_readiness_inputs
from .readiness_contract import sitl_extnav_ready, sitl_mavlink_command_alive
from sim.runtime.readiness import command_readiness_label


def control_readiness(owner: Any, snap: TelemetrySnapshot) -> tuple[str, str]:
    arm_ready = service_ready(owner._arm_client) > 0
    mode_ready = service_ready(owner._mode_client) > 0
    rc_ready = owner._manual_control_subscribers > 0 or owner._rc_override_subscribers > 0
    backend = effective_backend(owner)
    sim_bridge_backend = backend == BACKEND_SIM_BRIDGE
    command_alive = sitl_mavlink_command_alive(backend, snap)
    extnav_ready = sitl_extnav_ready(backend, snap)

    return command_readiness_label(
        build_command_readiness_inputs(
            owner=owner,
            snap=snap,
            backend=backend,
            sim_bridge_backend=sim_bridge_backend,
            command_alive=command_alive,
            extnav_ready=extnav_ready,
            arm_ready=arm_ready,
            mode_ready=mode_ready,
            rc_ready=rc_ready,
        )
    )


def command_alive(owner: Any, snap: TelemetrySnapshot) -> bool:
    return sitl_mavlink_command_alive(effective_backend(owner), snap)


def extnav_ready(owner: Any, snap: TelemetrySnapshot) -> bool:
    return sitl_extnav_ready(effective_backend(owner), snap)


__all__ = ["command_alive", "control_readiness", "extnav_ready"]
