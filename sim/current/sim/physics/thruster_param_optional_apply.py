"""Optional per-thruster dynamics/asymmetry overrides."""

from __future__ import annotations

from typing import Any, MutableMapping, Optional

import numpy as np

from .thruster_param_common import is_number


def apply_optional_thruster_overrides(
    name: str,
    cfg: dict[str, Any],
    *,
    thruster_reverse_asymmetry: MutableMapping[str, Optional[float]],
    thruster_tau_up: MutableMapping[str, Optional[float]],
    thruster_tau_down: MutableMapping[str, Optional[float]],
) -> None:
    reverse_asym = cfg.get("reverse_asymmetry")
    if is_number(reverse_asym):
        thruster_reverse_asymmetry[name] = float(np.clip(float(reverse_asym), 0.1, 1.5))
    tau_up = cfg.get("tau_up")
    if is_number(tau_up):
        thruster_tau_up[name] = float(np.clip(float(tau_up), 1.0e-4, 2.0))
    tau_down = cfg.get("tau_down")
    if is_number(tau_down):
        thruster_tau_down[name] = float(np.clip(float(tau_down), 1.0e-4, 2.0))


__all__ = ["apply_optional_thruster_overrides"]
