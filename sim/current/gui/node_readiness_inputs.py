"""Command-readiness input assembly for the GUI node."""

from __future__ import annotations

from typing import Any

from .models import TelemetrySnapshot
from .node_readiness_freshness import real_start_fresh
from .node_readiness_mode import required_auto_ready_mode
from .node_readiness_runtime_inputs import build_runtime_readiness
from sim.runtime.readiness import CommandReadinessInputs


def build_command_readiness_inputs(
    *,
    owner: Any,
    snap: TelemetrySnapshot,
    backend: str,
    sim_bridge_backend: bool,
    command_alive: bool,
    extnav_ready: bool,
    arm_ready: bool,
    mode_ready: bool,
    rc_ready: bool,
) -> CommandReadinessInputs:
    return CommandReadinessInputs(
        runtime=build_runtime_readiness(
            snap=snap,
            command_alive=command_alive,
            extnav_ready=extnav_ready,
        ),
        arm_service_ready=arm_ready,
        mode_service_ready=mode_ready,
        rc_source_ready=rc_ready,
        real_start_fresh=real_start_fresh(snap),
        real_start_required=bool(snap.real_start_required),
        real_start_ok=bool(snap.real_start_ok),
        real_start_status=str(snap.real_start_status),
        real_start_released=bool(snap.real_start_released),
        settle_left_s=float(owner._arm_mode_settle_left_s()),
        manual_input=bool(snap.manual_input),
        armed=bool(snap.armed),
        mode=str(snap.mode or snap.vehicle_mode or ""),
        required_mode=required_auto_ready_mode(backend, snap),
        require_runtime_command_path=sim_bridge_backend,
        require_fresh_rcout=True,
    )


__all__ = ["build_command_readiness_inputs"]
