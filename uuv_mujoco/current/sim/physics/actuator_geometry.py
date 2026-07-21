"""Compatibility surface for actuator site and propeller geometry helpers."""

from __future__ import annotations

from sim.physics.actuator_geometry_offsets import (
    apply_horizontal_thruster_z_offset,
    apply_vertical_thruster_x_scale,
)
from sim.physics.actuator_geometry_propellers import PropellerJointMaps, propeller_joint_maps
from sim.physics.actuator_geometry_sites import actuator_site_id, actuator_site_ids

__all__ = [
    "PropellerJointMaps",
    "actuator_site_id",
    "actuator_site_ids",
    "apply_horizontal_thruster_z_offset",
    "apply_vertical_thruster_x_scale",
    "propeller_joint_maps",
]
