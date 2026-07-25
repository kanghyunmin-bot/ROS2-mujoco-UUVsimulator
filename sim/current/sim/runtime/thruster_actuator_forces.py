"""Thruster force update loop for MuJoCo actuators."""

from __future__ import annotations

from typing import Any

import numpy as np

from sim.runtime.thruster_actuator_command import shaped_thruster_command, thruster_force, update_thruster_state
from sim.runtime.thruster_actuator_params import ThrusterUpdateParams, global_thruster_update_params
from sim.runtime.thruster_actuator_wrench import accumulate_thruster_wrench


def update_thruster_forces(runtime: Any, dt: float, *, base_id: int) -> None:
    runtime.last_reaction_torque_world = np.zeros(3, dtype=np.float64)
    runtime.last_force_body = np.zeros(3, dtype=np.float64)
    runtime.last_torque_body = np.zeros(3, dtype=np.float64)
    base_rot = runtime.data.xmat[base_id].reshape(3, 3)
    com_body = runtime.model.body_ipos[base_id].copy()
    params = global_thruster_update_params(runtime)

    for name in runtime.all_thruster_names:
        _update_one_thruster(
            runtime=runtime,
            name=name,
            dt=dt,
            base_rot=base_rot,
            com_body=com_body,
            params=params,
        )


def _update_one_thruster(
    *,
    runtime: Any,
    name: str,
    dt: float,
    base_rot: np.ndarray,
    com_body: np.ndarray,
    params: ThrusterUpdateParams,
) -> None:
    aid = runtime.actuator_ids[name]
    lo, hi = runtime.ctrlrange[aid]
    state_value = update_thruster_state(runtime, name, dt, params)
    shaped_cmd = shaped_thruster_command(runtime, state_value, params)
    force = thruster_force(runtime, name, shaped_cmd)
    force = float(np.clip(force, lo, hi))
    runtime.data.ctrl[aid] = force
    runtime.force_cmd[name] = force

    sid = runtime.site_ids.get(name, -1)
    accumulate_thruster_wrench(
        runtime,
        name=name,
        aid=aid,
        sid=sid,
        force=force,
        base_rot=base_rot,
        com_body=com_body,
        reaction_torque_gain=params.reaction_torque_gain,
    )


__all__ = ["update_thruster_forces"]
