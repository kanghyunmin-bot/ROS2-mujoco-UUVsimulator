"""Fluid-model ownership contract setup for the MuJoCo runner."""

from __future__ import annotations

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
    if use_custom_hydrodynamics and _preserve_environment_fluid(model):
        model.opt.density = scene_fluid_density
        model.opt.viscosity = scene_fluid_viscosity
        print(
            "[physics] Python owns vehicle hydro; native fluid retained for mooring ropes",
            flush=True,
        )
        return True
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
    # Validate the compiled model, not its filename.  Research scenes can carry
    # ellipsoid fluid proxies without using either token in the XML path.
    has_ellipsoid_proxy = any(float(fluid[0]) > 0.0 for fluid in model.geom_fluid)
    if not has_ellipsoid_proxy:
        print(
            "[physics] warning: current ellipsoid mode selected but the compiled scene "
            "does not contain an active ellipsoid fluid proxy "
            f"({scene_path}).",
            flush=True,
        )
    return False


def _preserve_environment_fluid(model: Any) -> bool:
    """Disable native vehicle loads without removing drag from articulated ropes.

    Called once during model setup. An effectively zero-interaction geom on each vehicle body
    prevents MuJoCo's inertia-based fluid fallback when its ellipsoids are disabled.
    Environment geometry retains its authored fluid coefficients.
    """
    if not hasattr(model, "geom") or not any(
        "_rope_geom_" in (model.geom(i).name or "") for i in range(model.ngeom)
    ):
        return False
    vehicle = next(
        (i for i in range(model.nbody) if model.body(i).name == "base_link"), -1
    )
    if vehicle < 0:
        raise ValueError("Articulated-rope fluid ownership requires a base_link body")
    bodies = {vehicle}
    for body in range(vehicle + 1, model.nbody):
        if int(model.body_parentid[body]) in bodies:
            bodies.add(body)
    for body in bodies:
        first = int(model.body_geomadr[body])
        count = int(model.body_geomnum[body])
        if count:
            model.geom_fluid[first : first + count] = 0
            # A positive interaction coefficient selects geom fluid instead of
            # inertia-box fallback. Near-zero interaction also removes the
            # viscosity term, which ignores the quadratic drag coefficients.
            model.geom_fluid[first, 0] = 1e-30
    return True


__all__ = ["configure_fluid_model_contract"]
