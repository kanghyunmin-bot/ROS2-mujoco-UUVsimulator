"""Velocity-term helpers for Fossen residual wrench evaluation."""

from __future__ import annotations

import numpy as np


FOSSEN_OUTPUT_INDEX = {"x": 0, "y": 1, "z": 2, "k": 3, "m": 4, "n": 5}


def fossen_velocity_terms(rel_lin_vel_body: np.ndarray, ang_vel_body: np.ndarray) -> dict[str, float]:
    u_body, v_body, w_body = (float(value) for value in rel_lin_vel_body)
    p_body, q_body, r_body = (float(value) for value in ang_vel_body)
    return {
        "u": u_body,
        "v": v_body,
        "w": w_body,
        "p": p_body,
        "q": q_body,
        "r": r_body,
    }


__all__ = ["FOSSEN_OUTPUT_INDEX", "fossen_velocity_terms"]
