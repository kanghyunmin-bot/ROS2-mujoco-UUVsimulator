"""Parameter extraction for MuJoCo thruster force updates."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ThrusterUpdateParams:
    deadzone: float
    global_tau_up: float
    global_tau_down: float
    command_limit: float
    reaction_torque_gain: float


def global_thruster_update_params(runtime) -> ThrusterUpdateParams:
    deadzone = float(np.clip(runtime.thruster_global["deadzone"], 0.0, 0.95))
    return ThrusterUpdateParams(
        deadzone=deadzone,
        global_tau_up=float(max(runtime.thruster_global["tau_up"], 1e-4)),
        global_tau_down=float(max(runtime.thruster_global["tau_down"], 1e-4)),
        command_limit=float(np.clip(runtime.thruster_global["command_limit"], deadzone + 1e-3, 1.0)),
        reaction_torque_gain=float(max(runtime.thruster_global["reaction_torque_gain"], 0.0)),
    )


def thruster_tau(runtime, name: str, params: ThrusterUpdateParams) -> tuple[float, float]:
    tau_up = float(runtime.thruster_tau_up[name] if runtime.thruster_tau_up[name] is not None else params.global_tau_up)
    tau_down = float(
        runtime.thruster_tau_down[name] if runtime.thruster_tau_down[name] is not None else params.global_tau_down
    )
    return tau_up, tau_down


__all__ = ["ThrusterUpdateParams", "global_thruster_update_params", "thruster_tau"]
