"""Coupling summaries for actuator wrench audit axes."""

from __future__ import annotations

import math

import numpy as np


def _force_leak_ratio(force_frd: np.ndarray, primary_idx: int) -> float:
    primary = abs(force_frd[primary_idx])
    leak = math.sqrt(
        sum(float(force_frd[idx]) ** 2 for idx in range(3) if idx != primary_idx)
    )
    return leak / primary if primary > 1.0e-12 else math.inf


def _torque_leak_ratio(torque_frd: np.ndarray, primary_idx: int) -> float:
    primary = abs(torque_frd[primary_idx])
    leak = math.sqrt(
        sum(float(torque_frd[idx]) ** 2 for idx in range(3) if idx != primary_idx)
    )
    return leak / primary if primary > 1.0e-12 else math.inf


def coupling_summary(force_frd: np.ndarray, torque_frd: np.ndarray, axis: str) -> dict[str, float]:
    force_norm = float(np.linalg.norm(force_frd))
    torque_norm = float(np.linalg.norm(torque_frd))
    summary: dict[str, float] = {
        "force_norm": force_norm,
        "torque_norm": torque_norm,
        "surge_force": float(force_frd[0]),
        "right_force": float(force_frd[1]),
        "down_force": float(force_frd[2]),
        "roll_torque": float(torque_frd[0]),
        "pitch_torque": float(torque_frd[1]),
        "yaw_torque": float(torque_frd[2]),
    }
    if axis == "forward":
        summary["force_offaxis_ratio"] = _force_leak_ratio(force_frd, 0)
    elif axis == "lateral":
        summary["force_offaxis_ratio"] = _force_leak_ratio(force_frd, 1)
    elif axis == "heave":
        summary["force_offaxis_ratio"] = _force_leak_ratio(force_frd, 2)
    elif axis == "roll":
        summary["torque_offaxis_ratio"] = _torque_leak_ratio(torque_frd, 0)
    elif axis == "pitch":
        summary["torque_offaxis_ratio"] = _torque_leak_ratio(torque_frd, 1)
    elif axis == "yaw":
        summary["torque_offaxis_ratio"] = _torque_leak_ratio(torque_frd, 2)
    return summary


__all__ = ["coupling_summary"]
