"""ExternalNav startup logging helpers for SitlTransport."""

from __future__ import annotations

import os


def ignored_extnav_override_envs() -> list[str]:
    return [
        name
        for name in ("ROS2_UUV_SITL_EXTNAV_Z_UP", "ROS2_UUV_SITL_EXTNAV_VELZ_SCALE")
        if os.getenv(name) not in (None, "")
    ]


def log_ignored_extnav_override_envs() -> None:
    ignored = ignored_extnav_override_envs()
    if not ignored:
        return
    print(
        "[sitl_transport] ignoring ExternalNav z override envs "
        f"{', '.join(ignored)}; ExternalNav uses canonical NED down-positive state.",
        flush=True,
    )


def log_extnav_startup_state(transport: object) -> None:
    if transport._sitl_extnav_enabled:
        print(
            "[sitl_transport] EKF3 ExternalNav output enabled "
            f"({transport._sitl_extnav_rate_hz:.1f}Hz VISION_POSITION_DELTA body-frame odometry, "
            "NED down-positive source state, "
            f"scheduler={transport._sitl_extnav_scheduler})",
            flush=True,
        )
    elif transport._sitl_bridge_extnav_disabled:
        print(
            "[sitl_transport] bridge ExternalNav output disabled; "
            "native VPD is owned by the external controller-parity probe",
            flush=True,
        )
    elif bool(getattr(transport, "_sitl_truth_extnav_blocked", False)):
        print(
            "[sitl_transport] MuJoCo-truth ExternalNav blocked by real-package parity; "
            "use the DVL driver and custom MAVROS VISION_POSITION_DELTA path",
            flush=True,
        )


__all__ = [
    "ignored_extnav_override_envs",
    "log_extnav_startup_state",
    "log_ignored_extnav_override_envs",
]
