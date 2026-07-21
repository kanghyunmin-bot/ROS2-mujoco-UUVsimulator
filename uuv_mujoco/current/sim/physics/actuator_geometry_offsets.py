"""Compatibility surface for runtime actuator site geometry adjustments."""

from __future__ import annotations

from sim.physics.actuator_geometry_horizontal import apply_horizontal_thruster_z_offset
from sim.physics.actuator_geometry_vertical import apply_vertical_thruster_x_scale

__all__ = ["apply_horizontal_thruster_z_offset", "apply_vertical_thruster_x_scale"]
