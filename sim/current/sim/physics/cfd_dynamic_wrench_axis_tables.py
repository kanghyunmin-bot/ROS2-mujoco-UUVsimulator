"""Validation helpers for CFD-derived dynamic wrench axis tables."""

from __future__ import annotations

from collections.abc import Callable
from typing import Optional

import numpy as np


def valid_cfd_axis_force_tables(
    speeds: np.ndarray | None,
    positive: np.ndarray | None,
    negative: np.ndarray | None,
) -> bool:
    if speeds is None or positive is None or negative is None:
        return False
    return bool(
        speeds.size == positive.size
        and speeds.size == negative.size
        and speeds.size > 0
        and np.all(np.isfinite(speeds))
        and np.all(np.isfinite(positive))
        and np.all(np.isfinite(negative))
    )


def log_invalid_cfd_axis(axis_name: str, log: Optional[Callable[[str], None]]) -> None:
    if log is None:
        return
    log(
        "[physics] ignoring cfd_dynamic_wrench axis "
        f"{axis_name!r}: expected finite same-length speeds/positive/negative force tables"
    )


def nonnegative_float_table(values: np.ndarray) -> np.ndarray:
    return np.maximum(values.astype(np.float64, copy=False), 0.0)


__all__ = [
    "log_invalid_cfd_axis",
    "nonnegative_float_table",
    "valid_cfd_axis_force_tables",
]
