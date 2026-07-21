"""GUI-exposed physics tuning configuration."""

from __future__ import annotations

from copy import deepcopy
from typing import Any

from physics.sim_profile_defaults import DEFAULT_SIM_PROFILES


PHYSICS_PROFILE_NAME = "current"
_CURRENT_DEFAULTS = DEFAULT_SIM_PROFILES[PHYSICS_PROFILE_NAME]


def _default(key: str) -> Any:
    """Keep GUI fallbacks identical to the runtime's canonical profile."""

    return deepcopy(_CURRENT_DEFAULTS[key])


PHYSICS_PARAM_SPECS: tuple[dict[str, Any], ...] = (
    {
        "key": "buoyancy_scale",
        "label": "Buoyancy scale",
        "default": _default("buoyancy_scale"),
        "description": "Net buoyancy multiplier. Small changes strongly affect depth trim.",
    },
    {
        "key": "buoyancy_slope_scale",
        "label": "Buoyancy slope scale",
        "default": _default("buoyancy_slope_scale"),
        "description": "Waterline buoyancy transition sharpness. High values can make depth response stiff.",
    },
    {
        "key": "cob_torque_scale",
        "label": "CoB torque scale",
        "default": _default("cob_torque_scale"),
        "description": "Restoring moment gain from CB-CG offset. Main roll/pitch stability knob.",
    },
    {
        "key": "cob_z_offset",
        "label": "CoB z offset",
        "default": _default("cob_z_offset"),
        "description": "Vertical CB-CG offset. Higher usually increases roll/pitch restoring.",
    },
    {
        "key": "cob_x_offset",
        "label": "CoB x offset",
        "default": _default("cob_x_offset"),
        "description": "Forward CB offset. Tunes pitch trim under buoyancy.",
    },
    {
        "key": "buoyancy_point_blend",
        "label": "Buoyancy point blend",
        "default": _default("buoyancy_point_blend"),
        "description": "0=center buoyancy, 1=distributed buoyancy points. Affects roll/pitch torque.",
    },
    {
        "key": "thruster_force_max",
        "label": "Thruster force max",
        "default": _default("thruster_force_max"),
        "description": "Per-thruster force limit used by the disabled-performance simple model.",
    },
    {
        "key": "mujoco_fluidcoef_scale",
        "label": "MuJoCo fluidcoef scale",
        "kind": "vector5",
        "default": _default("mujoco_fluidcoef_scale"),
        "description": "Current-mode source-of-truth scale: blunt, slender, angular, Kutta, Magnus.",
    },
    {
        "key": "current_world",
        "label": "Current world xyz",
        "kind": "vector3",
        "default": _default("current_world"),
        "description": "Water-current velocity in world frame.",
    },
    {
        "key": "body_inertia_scale_xyz",
        "label": "Body inertia x y z",
        "kind": "vector3",
        "default": _default("body_inertia_scale_xyz"),
        "description": "Body rotational inertia scale. Affects roll/pitch/yaw acceleration.",
    },
    {
        "key": "yaw_torque_scale",
        "label": "Yaw torque scale",
        "default": _default("yaw_torque_scale"),
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
