"""Built-in simulation profile defaults."""

from __future__ import annotations

from typing import Any


PROFILE_ALIASES: dict[str, str] = {
    "legacy": "legacy",
    "custom": "legacy",
    "current": "current",
    "ellipsoid": "current",
}


DEFAULT_SIM_PROFILES: dict[str, dict[str, Any]] = {
    "legacy": {
        "half_height": 0.147,
        "buoyancy_model": "ellipsoid",
        "buoyancy_scale": 0.9997,
        "buoyancy_slope_scale": 1.0,
        "surface_heave_damping": 18.0,
        "heave_damping_scale": 1.0,
        "cob_torque_scale": 0.20,
        "buoyancy_point_blend": 1.0,
        "cob_x_offset": -0.003,
        "cob_z_offset": 0.010,
        "thruster_voltage": 20.0,
        "thruster_force_max": 65.0,
        "linear_drag": 1.10,
        "angular_drag": 0.32,
        "linear_damping_linear": [1.10, 1.32, 1.54],
        "linear_damping_angular": [0.32, 0.36, 0.28],
        "quadratic_damping_linear": [1.40, 2.00, 2.40],
        "quadratic_damping_angular": [0.10, 0.12, 0.08],
        "added_mass_linear": [1.80, 2.30, 3.00],
        "added_mass_angular": [0.10, 0.12, 0.08],
        "current_world": [0.0, 0.0, 0.0],
        "body_components": [
            {
                "name": "center_enclosure",
                "shape": "cylinder",
                "size": [0.220, 0.120, 0.110],
                "mass": 6.0,
                "mass_pos": [-0.005, 0.0, 0.075],
                "buoyancy_pos": [-0.005, 0.0, 0.060],
                "buoyancy_share": 0.20,
            },
            {
                "name": "port_lower_body",
                "shape": "capsule",
                "size": [0.380, 0.090, 0.090],
                "mass": 4.5,
                "mass_pos": [0.0, 0.208, -0.005],
                "buoyancy_pos": [0.0, 0.208, 0.040],
                "buoyancy_share": 0.40,
            },
            {
                "name": "starboard_lower_body",
                "shape": "capsule",
                "size": [0.380, 0.090, 0.090],
                "mass": 4.5,
                "mass_pos": [0.0, -0.208, -0.005],
                "buoyancy_pos": [0.0, -0.208, 0.040],
                "buoyancy_share": 0.40,
            },
        ],
        "buoyancy_points": [
            {"name": "buoy_ver_lf", "pos": [0.149, 0.2346, 0.040], "share": 0.25, "half_height": 0.090},
            {"name": "buoy_ver_lr", "pos": [-0.171, 0.2346, 0.040], "share": 0.25, "half_height": 0.090},
            {"name": "buoy_ver_rf", "pos": [0.149, -0.2344, 0.040], "share": 0.25, "half_height": 0.090},
            {"name": "buoy_ver_rr", "pos": [-0.171, -0.2344, 0.040], "share": 0.25, "half_height": 0.090},
        ],
        "spin_gain": 22.0,
        "yaw_torque_scale": 1.0,
        "validation_timestep": 0.004,
        "validation_iterations": 20,
        "validation_ls_iterations": 8,
        "validation_step_start": 0.5,
        "validation_step_end": 1.4,
        "validation_total_time": 3.2,
        "validation_step_amp_ratio": 0.35,
    },
    "current": {
        "buoyancy_model": "ellipsoid",
        "buoyancy_scale": 1.0005,
        "buoyancy_slope_scale": 1.0,
        "cob_torque_scale": 1.05,
        "buoyancy_point_blend": 1.0,
        "hydrostatic_volume_source": "body_components",
        "hydrostatic_restoring": {
            "active": False,
            "roll_stiffness_nm_per_rad": 0.0,
            "pitch_stiffness_nm_per_rad": 0.0,
        },
        "cob_x_offset": 0.0,
        "cob_z_offset": 0.0,
        "thruster_voltage": 16.0,
        "thruster_force_max": 21.0,
        "current_world": [0.0, 0.0, 0.0],
        "body_components": [
            {
                "name": "center_enclosure",
                "shape": "cylinder",
                "size": [0.220, 0.120, 0.110],
                "mass": 6.0,
                "mass_pos": [-0.005, 0.0, 0.075],
                "buoyancy_pos": [-0.005, 0.0, 0.060],
                "buoyancy_share": 0.20,
            },
            {
                "name": "port_lower_body",
                "shape": "capsule",
                "size": [0.380, 0.090, 0.090],
                "mass": 4.5,
                "mass_pos": [0.0, 0.208, -0.005],
                "buoyancy_pos": [0.0, 0.208, 0.040],
                "buoyancy_share": 0.40,
            },
            {
                "name": "starboard_lower_body",
                "shape": "capsule",
                "size": [0.380, 0.090, 0.090],
                "mass": 4.5,
                "mass_pos": [0.0, -0.208, -0.005],
                "buoyancy_pos": [0.0, -0.208, 0.040],
                "buoyancy_share": 0.40,
            },
        ],
        "buoyancy_points": [
            {
                "name": "buoy_ver_lf",
                "pos": [0.149, 0.2346, 0.040],
                "share": 0.25,
                "half_height": 0.090,
            },
            {
                "name": "buoy_ver_lr",
                "pos": [-0.171, 0.2346, 0.040],
                "share": 0.25,
                "half_height": 0.090,
            },
            {
                "name": "buoy_ver_rf",
                "pos": [0.149, -0.2344, 0.040],
                "share": 0.25,
                "half_height": 0.090,
            },
            {
                "name": "buoy_ver_rr",
                "pos": [-0.171, -0.2344, 0.040],
                "share": 0.25,
                "half_height": 0.090,
            },
        ],
        "ellipsoid_model": {
            "active": True,
            "semi_axes": [0.160, 0.106, 0.147],
            "use_shape_volume": False,
        },
        "spin_gain": 22.0,
        "yaw_torque_scale": 1.0,
        "validation_timestep": 0.005,
        "validation_iterations": 20,
        "validation_ls_iterations": 8,
        "validation_step_start": 0.5,
        "validation_step_end": 1.4,
        "validation_total_time": 3.2,
        "validation_step_amp_ratio": 0.30,
    },
}


__all__ = ["PROFILE_ALIASES", "DEFAULT_SIM_PROFILES"]
