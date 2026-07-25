"""Runtime application of distributed body component contracts."""

from __future__ import annotations

from sim.physics.body_distribution_apply import apply_composite_body_to_model
from sim.physics.body_distribution_inertia import compute_composite_body_distribution
from sim.physics.body_distribution_log import log_body_component_distribution


def apply_body_component_distribution(
    *,
    mujoco,
    model,
    data,
    components,
    sim_profile: dict,
    base_id: int,
    world_qpos_adr: int,
    world_qvel_adr: int,
) -> None:
    """Apply distributed profile body components to the MuJoCo base body."""
    composite = compute_composite_body_distribution(components, sim_profile)
    if composite is None:
        return
    old_mass, old_com, old_inertia = apply_composite_body_to_model(
        mujoco=mujoco,
        model=model,
        data=data,
        base_id=base_id,
        world_qpos_adr=world_qpos_adr,
        world_qvel_adr=world_qvel_adr,
        total_mass=composite.total_mass,
        composite_com=composite.composite_com,
        composite_inertia=composite.composite_inertia,
    )
    log_body_component_distribution(
        components=components,
        old_mass=old_mass,
        old_com=old_com,
        old_inertia=old_inertia,
        total_mass=composite.total_mass,
        composite_com=composite.composite_com,
        composite_inertia=composite.composite_inertia,
        inertia_scale=composite.inertia_scale,
    )


__all__ = ["apply_body_component_distribution"]
