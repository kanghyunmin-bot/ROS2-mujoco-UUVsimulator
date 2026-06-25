"""Typed outputs for MuJoCo static physics contract audits."""

from __future__ import annotations

from dataclasses import dataclass


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
    "NeutralSimSummary",
    "PHYSICAL_VERTICAL_THRUSTERS",
    "PHYSICAL_YAW_THRUSTERS",
]
