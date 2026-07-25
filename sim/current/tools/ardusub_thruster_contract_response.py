"""Axis response calculations for the ArduSub to MuJoCo thruster contract."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from ardusub_thruster_contract_constants import AXES, EXPECTED_PRIMARY_SIGN, PRIMARY_WRENCH_INDEX
from ardusub_thruster_contract_scene import load_thruster_wrenches_frd
from physics.thruster_mapping import (  # noqa: E402
    ARDUSUB_VECTORED_6DOF_MOTOR_FACTORS_FRD,
    ARDUSUB_VECTORED_6DOF_SERVO_MAP,
    ARDUSUB_VECTORED_6DOF_SERVO_SIGNS,
    REAL_ROBOT_MOT_DIRECTIONS,
)


def build_axis_response(scene: Path) -> dict[str, dict[str, object]]:
    wrenches = load_thruster_wrenches_frd(scene)
    missing = [name for name in ARDUSUB_VECTORED_6DOF_SERVO_MAP if name not in wrenches]
    if missing:
        raise RuntimeError(f"scene is missing thruster actuators: {', '.join(missing)}")

    factors = np.asarray(ARDUSUB_VECTORED_6DOF_MOTOR_FACTORS_FRD, dtype=np.float64)
    directions = np.asarray(REAL_ROBOT_MOT_DIRECTIONS, dtype=np.float64)
    signs = np.asarray(ARDUSUB_VECTORED_6DOF_SERVO_SIGNS, dtype=np.float64)

    report: dict[str, dict[str, object]] = {}
    for axis_idx, axis in enumerate(AXES):
        report[axis] = _build_single_axis_response(
            axis=axis,
            axis_idx=axis_idx,
            wrenches=wrenches,
            factors=factors,
            directions=directions,
            signs=signs,
        )
    return report


def _build_single_axis_response(
    *,
    axis: str,
    axis_idx: int,
    wrenches: dict[str, np.ndarray],
    factors: np.ndarray,
    directions: np.ndarray,
    signs: np.ndarray,
) -> dict[str, object]:
    command_by_thruster: dict[str, float] = {}
    channel_cmds: list[float] = []
    for motor_idx, thr_name in enumerate(ARDUSUB_VECTORED_6DOF_SERVO_MAP):
        cmd = factors[motor_idx, axis_idx] * directions[motor_idx] * signs[motor_idx]
        channel_cmds.append(float(cmd))
        command_by_thruster[thr_name] = float(cmd)
    total = np.zeros(6, dtype=np.float64)
    for thr_name, cmd in command_by_thruster.items():
        total += float(cmd) * wrenches[thr_name]
    primary_idx = PRIMARY_WRENCH_INDEX[axis]
    primary = float(total[primary_idx])
    expected_sign = float(EXPECTED_PRIMARY_SIGN[axis])
    return {
        "primary": primary,
        "expected_sign": expected_sign,
        "primary_index": primary_idx,
        "wrench_frd": [float(v) for v in total],
        "channel_cmds": channel_cmds,
        "thruster_cmds": dict(command_by_thruster),
        "ok": primary * expected_sign > 1.0e-6,
    }
