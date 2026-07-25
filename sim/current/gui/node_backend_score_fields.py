"""Backend graph-count score tables."""

from __future__ import annotations


MAVROS_SCORE_FIELDS = (
    ("vehicle_info_services", 3),
    ("arm_services", 1),
    ("mode_services", 1),
    ("rc_out_publishers", 3),
    ("state_publishers", 1),
    ("pose_publishers", 1),
    ("velocity_body_publishers", 3),
)

SIM_BRIDGE_SCORE_FIELDS = (
    ("rc_in_publishers", 3),
    ("velocity_local_publishers", 2),
    ("bridge_imu_publishers", 2),
    ("bridge_rovio_publishers", 3),
    ("bridge_dvl_odom_publishers", 2),
    ("bridge_dvl_velocity_publishers", 1),
    ("bridge_depth_publishers", 1),
    ("bridge_battery_publishers", 1),
    ("rc_override_subscribers", 2),
    ("manual_control_subscribers", 2),
)

MAVROS_TIE_BREAK_FIELDS = (
    "rc_out_publishers",
    "velocity_body_publishers",
    "vehicle_info_services",
)


def score_fields(counts: dict[str, int], fields: tuple[tuple[str, int], ...]) -> int:
    return sum(weight for key, weight in fields if counts[key] > 0)


def has_mavros_tie_breaker(counts: dict[str, int]) -> bool:
    return any(counts[key] > 0 for key in MAVROS_TIE_BREAK_FIELDS)


__all__ = [
    "MAVROS_SCORE_FIELDS",
    "MAVROS_TIE_BREAK_FIELDS",
    "SIM_BRIDGE_SCORE_FIELDS",
    "has_mavros_tie_breaker",
    "score_fields",
]
