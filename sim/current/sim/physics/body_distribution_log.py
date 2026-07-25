"""Logging for distributed body component setup."""

from __future__ import annotations

import numpy as np


def log_body_component_distribution(
    *,
    components,
    old_mass: float,
    old_com: np.ndarray,
    old_inertia: np.ndarray,
    total_mass: float,
    composite_com: np.ndarray,
    composite_inertia: np.ndarray,
    inertia_scale: np.ndarray,
) -> None:
    print(
        "[model] distributed body components: "
        f"{len(components)} parts, mass {old_mass:.3f} -> {total_mass:.3f} kg, "
        f"CoM {np.array2string(old_com, precision=4)} -> "
        f"{np.array2string(composite_com, precision=4)}, "
        f"inertia {np.array2string(old_inertia, precision=4)} -> "
        f"{np.array2string(composite_inertia, precision=4)} "
        f"(scale={np.array2string(inertia_scale, precision=4)})",
        flush=True,
    )
    for component in components:
        log_body_component(component)


def log_body_component(component) -> None:
    print(
        "[model]   component "
        f"{component.name}: mass={component.mass:.3f}kg, "
        f"mass_pos={np.array2string(component.mass_pos, precision=4)}, "
        f"buoyancy_pos={np.array2string(component.buoyancy_pos, precision=4)}, "
        f"share={component.buoyancy_share:.3f}",
        flush=True,
    )


__all__ = ["log_body_component", "log_body_component_distribution"]
