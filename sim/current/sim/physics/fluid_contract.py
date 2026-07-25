"""Fluid-model ownership contract setup for the MuJoCo runner."""

from __future__ import annotations

from pathlib import Path
from typing import Any


def configure_fluid_model_contract(
    *,
    model: Any,
    fluid_model: str,
    scene_path: str,
    scene_fluid_density: float,
    scene_fluid_viscosity: float,
) -> bool:
    """Apply fluid ownership settings and return whether Python hydro owns drag."""
    use_custom_hydrodynamics = str(fluid_model) == "legacy"
    if use_custom_hydrodynamics:
        model.opt.density = 0.0
        model.opt.viscosity = 0.0
        print(
            "[physics] fluid model: legacy/custom Python 6DOF "
            f"(MuJoCo built-in fluid disabled, scene rho={scene_fluid_density:.1f}, "
            f"viscosity={scene_fluid_viscosity:.6f})",
            flush=True,
        )
        print(
            "[physics] hydrodynamic contract: Python applies hydrostatic buoyancy, "
            "added mass, Coriolis, linear damping, quadratic damping, and final thruster force.",
            flush=True,
        )
        return True

    model.opt.density = scene_fluid_density
    model.opt.viscosity = scene_fluid_viscosity
    print(
        "[physics] fluid model: current MuJoCo built-in ellipsoid "
        f"(rho={scene_fluid_density:.1f}, viscosity={scene_fluid_viscosity:.6f})",
        flush=True,
    )
    print(
        "[physics] hydrodynamic contract: MuJoCo ellipsoid fluidcoef owns baseline drag/lift; "
        "Python applies hydrostatic buoyancy/restoring torque, explicitly enabled residual "
        "Fossen/CFD terms, and final thruster force.",
        flush=True,
    )
    if not any(token in Path(scene_path).name for token in ("current", "ellipsoid")):
        print(
            "[physics] warning: current ellipsoid mode selected but the scene path "
            "does not look like an ellipsoid proxy scene.",
            flush=True,
        )
    return False


__all__ = ["configure_fluid_model_contract"]
