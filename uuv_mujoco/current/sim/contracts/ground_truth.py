"""Ground-truth publication safety policy for the ROS bridge."""

from __future__ import annotations


UNSAFE_LEGACY_GROUND_TRUTH_ODOMETRY_FILTERED_CLI = (
    "--unsafe-legacy-ground-truth-odometry-filtered"
)
UNSAFE_LEGACY_GROUND_TRUTH_ODOMETRY_FILTERED_ENV = (
    "ROS2_UUV_UNSAFE_LEGACY_GROUND_TRUTH_ODOMETRY_FILTERED"
)
UNSAFE_LEGACY_GROUND_TRUTH_ODOMETRY_FILTERED_WARNING = (
    "UNSAFE LEGACY GROUND-TRUTH LEAKAGE ENABLED: uuv_mujoco_bridge is publishing "
    "exact MuJoCo state on /odometry/filtered. This topic is not a state estimate "
    "and must never be used for SLAM, state estimation, control, or research metrics."
)


def resolve_unsafe_legacy_ground_truth_odometry_filtered(
    *,
    requested: bool,
    real_pkg_compat: bool,
) -> bool:
    """Resolve the legacy exact-state alias while protecting strict mode."""

    enabled = bool(requested)
    if enabled and real_pkg_compat:
        raise ValueError(
            f"{UNSAFE_LEGACY_GROUND_TRUTH_ODOMETRY_FILTERED_CLI} and "
            f"{UNSAFE_LEGACY_GROUND_TRUTH_ODOMETRY_FILTERED_ENV}=1 are forbidden "
            "with --ros2-real-pkg-compat; /odometry/filtered must be owned only "
            "by the external estimator"
        )
    return enabled


__all__ = [
    "UNSAFE_LEGACY_GROUND_TRUTH_ODOMETRY_FILTERED_CLI",
    "UNSAFE_LEGACY_GROUND_TRUTH_ODOMETRY_FILTERED_ENV",
    "UNSAFE_LEGACY_GROUND_TRUTH_ODOMETRY_FILTERED_WARNING",
    "resolve_unsafe_legacy_ground_truth_odometry_filtered",
]
