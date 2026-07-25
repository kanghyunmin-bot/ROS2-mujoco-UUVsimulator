"""Heartbeat fallback helpers for MAVLink command target resolution."""

from __future__ import annotations


def heartbeat_for_command_mav(transport, mav):
    hb = transport._sitl_cmd_mav_hb if mav is transport._sitl_cmd_mav else transport._sitl_mav_hb
    return hb if hb is not None else transport._sitl_mav_hb


def fill_target_from_heartbeat(target_sys: int, target_comp: int, heartbeat) -> tuple[int, int] | None:
    if heartbeat is None:
        return None
    if target_sys <= 0:
        target_sys = int(heartbeat.get_srcSystem())
    if target_comp <= 0:
        target_comp = int(heartbeat.get_srcComponent())
    if target_sys <= 0 or target_comp <= 0:
        return None
    return target_sys, target_comp


def assign_mav_target(mav, target_sys: int, target_comp: int) -> None:
    try:
        mav.target_system = target_sys
        mav.target_component = target_comp
    except Exception:
        pass


__all__ = ["assign_mav_target", "fill_target_from_heartbeat", "heartbeat_for_command_mav"]
