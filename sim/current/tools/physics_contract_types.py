"""Typed outputs for MuJoCo static physics contract audits."""

from __future__ import annotations

from dataclasses import dataclass, field


PHYSICAL_VERTICAL_THRUSTERS = ("ver_lf", "ver_lr", "ver_rf", "ver_rr")
PHYSICAL_YAW_THRUSTERS = ("yaw_lf", "yaw_lr", "yaw_rf", "yaw_rr")


@dataclass(frozen=True)
class ForceBalance:
    label: str
    base_depth_m: float
    bar30_depth_m: float
    buoyancy_n: float
    weight_n: float
    net_up_n: float
    net_down_n: float
    accel_down_mps2: float
    submerged_fraction: float
    required_buoyancy_scale: float


@dataclass(frozen=True)
class NeutralSimSummary:
    label: str
    duration_s: float
    start_depth_m: float
    end_depth_m: float
    drift_m: float
    max_abs_vz_down_mps: float
    rms_vz_down_mps: float
    csv: str
    start_roll_deg: float = 0.0
    end_roll_deg: float = 0.0
    start_pitch_deg: float = 0.0
    end_pitch_deg: float = 0.0
    start_yaw_deg: float = 0.0
    end_yaw_deg: float = 0.0
    max_abs_roll_deg: float = 0.0
    max_abs_pitch_deg: float = 0.0
    max_abs_yaw_deg: float = 0.0
    max_abs_angular_rate_x_rad_s: float = 0.0
    max_abs_angular_rate_y_rad_s: float = 0.0
    max_abs_angular_rate_z_rad_s: float = 0.0
    max_angular_speed_rad_s: float = 0.0
    rms_angular_speed_rad_s: float = 0.0


@dataclass
class NeutralMotionSamples:
    """Attitude and angular-rate samples recorded by a neutral audit run."""

    roll_deg: list[float] = field(default_factory=list)
    pitch_deg: list[float] = field(default_factory=list)
    yaw_deg: list[float] = field(default_factory=list)
    angular_rate_x_rad_s: list[float] = field(default_factory=list)
    angular_rate_y_rad_s: list[float] = field(default_factory=list)
    angular_rate_z_rad_s: list[float] = field(default_factory=list)
    angular_speed_rad_s: list[float] = field(default_factory=list)


@dataclass(frozen=True)
class BodyContract:
    xml_mass_kg: float
    runtime_mass_kg: float
    xml_com_x_m: float
    xml_com_y_m: float
    xml_com_z_m: float
    runtime_com_x_m: float
    runtime_com_y_m: float
    runtime_com_z_m: float
    xml_inertia_x: float
    xml_inertia_y: float
    xml_inertia_z: float
    runtime_inertia_x: float
    runtime_inertia_y: float
    runtime_inertia_z: float
    inertia_scale_x: float
    inertia_scale_y: float
    inertia_scale_z: float


__all__ = [
    "BodyContract",
    "ForceBalance",
    "NeutralMotionSamples",
    "NeutralSimSummary",
    "PHYSICAL_VERTICAL_THRUSTERS",
    "PHYSICAL_YAW_THRUSTERS",
]
