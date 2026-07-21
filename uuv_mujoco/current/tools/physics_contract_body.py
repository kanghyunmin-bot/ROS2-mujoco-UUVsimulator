"""Runtime body mass, center, and inertia contract helpers."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from physics_contract_types import BodyContract
from physics_contract_mujoco import mujoco
from sim.physics.body_distribution_inertia import (  # noqa: E402
    component_self_inertia_diag,
    compute_composite_body_distribution,
)


def _xml_body_contract(xml_mass: float, xml_com: np.ndarray, xml_inertia: np.ndarray) -> BodyContract:
    inertia_scale = np.ones(3, dtype=np.float64)
    return BodyContract(
        xml_mass,
        xml_mass,
        float(xml_com[0]),
        float(xml_com[1]),
        float(xml_com[2]),
        float(xml_com[0]),
        float(xml_com[1]),
        float(xml_com[2]),
        float(xml_inertia[0]),
        float(xml_inertia[1]),
        float(xml_inertia[2]),
        float(xml_inertia[0]),
        float(xml_inertia[1]),
        float(xml_inertia[2]),
        float(inertia_scale[0]),
        float(inertia_scale[1]),
        float(inertia_scale[2]),
    )


def apply_runtime_body_contract(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    base_id: int,
    hydro_cfg: Any,
    sim_profile: dict[str, Any],
) -> BodyContract:
    """Mirror run_uuv_mujoco.py's runtime body mass/CoM/inertia contract."""

    xml_mass = float(model.body_mass[base_id])
    xml_com = model.body_ipos[base_id].copy()
    xml_inertia = model.body_inertia[base_id].copy()
    components = tuple(hydro_cfg.body_components)
    if not components:
        return _xml_body_contract(xml_mass, xml_com, xml_inertia)

    composite = compute_composite_body_distribution(components, sim_profile)
    if composite is None:
        raise RuntimeError("body_components total mass is not positive")

    model.body_mass[base_id] = composite.total_mass
    model.body_ipos[base_id, :] = composite.composite_com
    model.body_inertia[base_id, :] = np.maximum(composite.composite_inertia, 1.0e-6)
    if hasattr(mujoco, "mj_setConst"):
        mujoco.mj_setConst(model, data)
    mujoco.mj_forward(model, data)

    return BodyContract(
        xml_mass_kg=xml_mass,
        runtime_mass_kg=composite.total_mass,
        xml_com_x_m=float(xml_com[0]),
        xml_com_y_m=float(xml_com[1]),
        xml_com_z_m=float(xml_com[2]),
        runtime_com_x_m=float(composite.composite_com[0]),
        runtime_com_y_m=float(composite.composite_com[1]),
        runtime_com_z_m=float(composite.composite_com[2]),
        xml_inertia_x=float(xml_inertia[0]),
        xml_inertia_y=float(xml_inertia[1]),
        xml_inertia_z=float(xml_inertia[2]),
        runtime_inertia_x=float(composite.composite_inertia[0]),
        runtime_inertia_y=float(composite.composite_inertia[1]),
        runtime_inertia_z=float(composite.composite_inertia[2]),
        inertia_scale_x=float(composite.inertia_scale[0]),
        inertia_scale_y=float(composite.inertia_scale[1]),
        inertia_scale_z=float(composite.inertia_scale[2]),
    )
