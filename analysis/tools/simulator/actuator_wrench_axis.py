"""Per-axis actuator wrench calculation for contract audits."""

from __future__ import annotations

from typing import Any

import numpy as np

from actuator_wrench_model import actuator_site_id
from physics.thruster_mapping import (
    ARDUSUB_VECTORED_6DOF_MOTOR_FACTORS_FRD,
    ARDUSUB_VECTORED_6DOF_SERVO_MAP,
    ARDUSUB_VECTORED_6DOF_SERVO_SIGNS,
    REAL_ROBOT_MOT_DIRECTIONS,
)


def wrench_for_axis(
    model: mujoco.MjModel,
    axis_idx: int,
    direct_gains: dict[str, float],
    *,
    unit_gains: bool,
) -> tuple[np.ndarray, np.ndarray, list[dict[str, Any]]]:
    import mujoco

    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    com_body = model.body_ipos[base_id].copy() if base_id >= 0 else np.zeros(3)
    factors = np.asarray(
        [row[axis_idx] for row in ARDUSUB_VECTORED_6DOF_MOTOR_FACTORS_FRD],
        dtype=np.float64,
    )
    directions = np.asarray(REAL_ROBOT_MOT_DIRECTIONS, dtype=np.float64)
    servo_delta = factors * directions
    force_flu = np.zeros(3, dtype=np.float64)
    torque_flu = np.zeros(3, dtype=np.float64)
    rows: list[dict[str, Any]] = []
    for idx, thruster_name in enumerate(ARDUSUB_VECTORED_6DOF_SERVO_MAP):
        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, thruster_name)
        sid = actuator_site_id(model, thruster_name)
        if aid < 0 or sid < 0:
            continue
        gear = model.actuator_gear[aid, :3].copy()
        norm = float(np.linalg.norm(gear))
        if norm <= 1.0e-12:
            continue
        force_dir = gear / norm
        command = float(servo_delta[idx] * ARDUSUB_VECTORED_6DOF_SERVO_SIGNS[idx])
        gain = 1.0 if unit_gains else float(direct_gains.get(thruster_name, 1.0))
        force = force_dir * command * gain
        arm = model.site_pos[sid].copy() - com_body
        torque = np.cross(arm, force)
        force_flu += force
        torque_flu += torque
        rows.append(
            {
                "servo": idx + 1,
                "thruster": thruster_name,
                "servo_delta": float(servo_delta[idx]),
                "servo_sign": float(ARDUSUB_VECTORED_6DOF_SERVO_SIGNS[idx]),
                "command": command,
                "direct_gain": gain,
                "force_flu": force.tolist(),
                "torque_flu": torque.tolist(),
            }
        )
    return force_flu, torque_flu, rows


__all__ = ["wrench_for_axis"]
