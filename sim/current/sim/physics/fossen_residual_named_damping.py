"""Named linear damping terms for Fossen residual wrench evaluation."""

from __future__ import annotations

import numpy as np


def apply_named_damping(
    wrench_body: np.ndarray,
    nu_terms: dict[str, float],
    output_index: dict[str, int],
    group: dict[str, float],
    *,
    forward_speed_scale: float = 1.0,
) -> None:
    for key, coeff in group.items():
        _apply_named_damping_term(
            wrench_body,
            nu_terms,
            output_index,
            key,
            float(coeff),
            forward_speed_scale=forward_speed_scale,
        )


def _apply_named_damping_term(
    wrench_body: np.ndarray,
    nu_terms: dict[str, float],
    output_index: dict[str, int],
    key: str,
    coeff: float,
    *,
    forward_speed_scale: float,
) -> None:
    if abs(coeff) <= 1.0e-12:
        return
    parts = key.split("_")
    if len(parts) != 2:
        return
    out_name, src_name = parts
    out_idx = output_index.get(out_name)
    if out_idx is None or src_name not in nu_terms:
        return
    wrench_body[out_idx] -= coeff * forward_speed_scale * nu_terms[src_name]


__all__ = ["apply_named_damping"]
