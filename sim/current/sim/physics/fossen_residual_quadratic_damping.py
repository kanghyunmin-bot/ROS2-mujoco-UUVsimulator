"""Quadratic damping terms for Fossen residual wrench evaluation."""

from __future__ import annotations

import numpy as np


def apply_quadratic_damping(
    wrench_body: np.ndarray,
    nu_terms: dict[str, float],
    output_index: dict[str, int],
    group: dict[str, float],
) -> None:
    for key, coeff in group.items():
        _apply_quadratic_damping_term(wrench_body, nu_terms, output_index, key, float(coeff))


def _apply_quadratic_damping_term(
    wrench_body: np.ndarray,
    nu_terms: dict[str, float],
    output_index: dict[str, int],
    key: str,
    coeff: float,
) -> None:
    if abs(coeff) <= 1.0e-12:
        return
    parts = key.split("_")
    if len(parts) != 4 or parts[1] != "abs":
        return
    out_name, abs_src_name, signed_src_name = parts[0], parts[2], parts[3]
    out_idx = output_index.get(out_name)
    if out_idx is None or abs_src_name not in nu_terms or signed_src_name not in nu_terms:
        return
    wrench_body[out_idx] -= coeff * abs(nu_terms[abs_src_name]) * nu_terms[signed_src_name]


__all__ = ["apply_quadratic_damping"]
