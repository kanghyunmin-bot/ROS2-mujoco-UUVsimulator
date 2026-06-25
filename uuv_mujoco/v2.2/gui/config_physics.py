"""GUI-exposed physics tuning configuration."""

from __future__ import annotations

from typing import Any


PHYSICS_PROFILE_NAME = "current"
PHYSICS_PARAM_SPECS: tuple[dict[str, Any], ...] = (
    {
        "key": "buoyancy_scale",
        "label": "Buoyancy scale",
        "default": 1.0005,
        "description": "Net buoyancy multiplier. Small changes strongly affect depth trim.",
    },
    {
        "key": "buoyancy_slope_scale",
        "label": "Buoyancy slope scale",
        "default": 1.2,
        "description": "Waterline buoyancy transition sharpness. High values can make depth response stiff.",
    },
    {
        "key": "cob_torque_scale",
        "label": "CoB torque scale",
        "default": 0.35,
        "description": "Restoring moment gain from CB-CG offset. Main roll/pitch stability knob.",
    },
    {
        "key": "cob_z_offset",
        "label": "CoB z offset",
        "default": 0.012,
        "description": "Vertical CB-CG offset. Higher usually increases roll/pitch restoring.",
    },
    {
        "key": "cob_x_offset",
        "label": "CoB x offset",
        "default": 0.009,
        "description": "Forward CB offset. Tunes pitch trim under buoyancy.",
    },
    {
        "key": "buoyancy_point_blend",
        "label": "Buoyancy point blend",
        "default": 1.0,
        "description": "0=center buoyancy, 1=distributed buoyancy points. Affects roll/pitch torque.",
    },
    {
        "key": "thruster_force_max",
        "label": "Thruster force max",
        "default": 21.0,
        "description": "Per-thruster force limit used by the disabled-performance simple model.",
    },
    {
        "key": "mujoco_fluidcoef_scale",
        "label": "MuJoCo fluidcoef scale",
        "kind": "vector5",
        "default": [1.0, 1.0, 1.0, 1.0, 1.0],
        "description": "Active current-mode geom fluidcoef scale: blunt, slender, angular, Kutta, Magnus.",
    },
    {
        "key": "current_world",
        "label": "Current world xyz",
        "kind": "vector3",
        "default": [0.0, 0.0, 0.0],
        "description": "Water-current velocity in world frame.",
    },
    {
        "key": "body_inertia_scale_xyz",
        "label": "Body inertia x y z",
        "kind": "vector3",
        "default": [1.0, 1.0, 1.0],
        "description": "Body rotational inertia scale. Affects roll/pitch/yaw acceleration.",
    },
    {
        "key": "yaw_torque_scale",
        "label": "Yaw torque scale",
        "default": 1.0,
        "description": "Additional yaw torque multiplier. Keep near 1 unless yaw rate is off.",
    },
)

CURRENT_MODE_INACTIVE_PHYSICS_KEYS: dict[str, str] = {
    "thruster_force_max": "inactive while the T200 performance curve is active",
}


__all__ = [
    "CURRENT_MODE_INACTIVE_PHYSICS_KEYS",
    "PHYSICS_PARAM_SPECS",
    "PHYSICS_PROFILE_NAME",
]
