"""ExternalNav readiness fields for MAVLink telemetry status payloads."""

from __future__ import annotations

from .sitl_status_age import wall_age_s


def build_extnav_status_fields(
    *,
    now_wall: float,
    extnav_enabled: bool,
    extnav_required: bool,
    extnav_last_send_wall: float,
    extnav_last_rate_hz: float,
    extnav_min_tx_hz: float,
    extnav_max_stale_s: float,
    extnav_scheduler: str,
    extnav_start_wall: float,
    extnav_grace_s: float,
    extnav_fault: str,
) -> dict[str, object]:
    last_send_age_s = wall_age_s(now_wall, extnav_last_send_wall)
    grace_active = bool(now_wall - extnav_start_wall < extnav_grace_s)
    fresh = (
        not extnav_required
        or (extnav_enabled and last_send_age_s <= extnav_max_stale_s and not extnav_fault)
    )
    rate_ok = not extnav_required or grace_active or extnav_last_rate_hz >= extnav_min_tx_hz
    return {
        "extnav_enabled": bool(extnav_enabled),
        "extnav_required": bool(extnav_required),
        "extnav_last_send_age_s": last_send_age_s,
        "extnav_last_rate_hz": float(extnav_last_rate_hz),
        "extnav_min_tx_hz": float(extnav_min_tx_hz),
        "extnav_max_stale_s": float(extnav_max_stale_s),
        "extnav_scheduler": str(extnav_scheduler),
        "extnav_grace_active": grace_active,
        "extnav_fault": str(extnav_fault),
        "extnav_ready": bool(fresh and rate_ok),
    }


__all__ = ["build_extnav_status_fields"]
