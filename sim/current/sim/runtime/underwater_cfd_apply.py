"""Apply CFD-derived dynamic hydrodynamic force."""

from __future__ import annotations

from typing import Any

import numpy as np

from sim.runtime.underwater_cfd_debug import maybe_log_cfd_dynamic_wrench
from sim.runtime.underwater_residual_wrench_body import cfd_dynamic_force_from_hyd
from sim.runtime.underwater_wrench_apply import apply_body_force, immersed_fraction


def apply_cfd_dynamic_wrench(
    runtime: Any,
    *,
    base_rot: np.ndarray,
    rel_lin_vel_body: np.ndarray,
    submerged: float,
) -> None:
    hyd = runtime.hydrodynamics
    immersed = immersed_fraction(submerged)
    cfd_force_body = cfd_dynamic_force_from_hyd(hyd, rel_lin_vel_body)
    apply_body_force(
        data=runtime.data,
        base_id=int(runtime.base_id),
        base_rot=base_rot,
        force_body=cfd_force_body,
        submerged=immersed,
    )
    maybe_log_cfd_dynamic_wrench(
        hyd=hyd,
        data=runtime.data,
        rel_lin_vel_body=rel_lin_vel_body,
        cfd_force_body=cfd_force_body,
        submerged=immersed,
    )


__all__ = ["apply_cfd_dynamic_wrench"]
