"""Profile key constants for closed-loop contract audits."""

from __future__ import annotations


PROFILE_KEYS = (
    "buoyancy_scale",
    "buoyancy_slope_scale",
    "cob_x_offset",
    "cob_z_offset",
    "cob_torque_scale",
    "body_inertia_scale_xyz",
    "yaw_torque_scale",
    "thruster_voltage",
    "thruster_force_max",
    "mujoco_fluidcoef_scale",
    "mujoco_fluidcoef_geom_scales",
    "dynamic_fluidcoef",
)

CURRENT_INACTIVE_KEYS = (
    "thruster_force_max",
)


__all__ = ["CURRENT_INACTIVE_KEYS", "PROFILE_KEYS"]
