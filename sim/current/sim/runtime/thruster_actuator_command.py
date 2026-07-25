"""Command-to-force conversion helpers for MuJoCo thruster updates."""

from __future__ import annotations

import numpy as np

from physics.hydrodynamics_helpers import first_order_response, shape_thruster_command
from sim.physics.thruster_force_model import force_from_shaped_command
from sim.runtime.thruster_actuator_immersion import force_immersion_scale
from sim.runtime.thruster_actuator_params import ThrusterUpdateParams, thruster_tau


def update_thruster_state(runtime, name: str, dt: float, params: ThrusterUpdateParams) -> float:
    target_norm = float(np.clip(runtime.target[name], -1.0, 1.0))
    tau_up, tau_down = thruster_tau(runtime, name, params)
    runtime.state[name] = first_order_response(runtime.state[name], target_norm, dt, tau_up, tau_down)
    return float(runtime.state[name])


def shaped_thruster_command(runtime, state_value: float, params: ThrusterUpdateParams) -> float:
    if runtime.perf_cfg.get("active") and runtime.perf_cfg.get("direct"):
        return float(np.clip(state_value, -1.0, 1.0))
    return shape_thruster_command(state_value, params.deadzone, params.command_limit)


def thruster_force(runtime, name: str, shaped_cmd: float) -> float:
    force = force_from_shaped_command(
        name=name,
        command_shaped=shaped_cmd,
        gain=float(runtime.thruster_scale.get(name, 1.0)),
        perf_cfg=runtime.perf_cfg,
        thruster_direct_scale=runtime.thruster_direct_scale,
        thruster_global=runtime.thruster_global,
        thruster_force_max=runtime.thruster_force_max,
        thruster_reverse_asymmetry=runtime.thruster_reverse_asymmetry,
    )
    return float(force * force_immersion_scale(runtime, name))


__all__ = ["shaped_thruster_command", "thruster_force", "update_thruster_state"]
