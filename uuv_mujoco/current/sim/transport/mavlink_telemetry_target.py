"""MAVLink telemetry target matching helpers."""

from __future__ import annotations


def message_source_ids(msg: object) -> tuple[int, int] | None:
    try:
        return int(msg.get_srcSystem()), int(msg.get_srcComponent())
    except Exception:
        return None


def source_matches_target_ids(
    msg: object,
    *,
    target_system: int,
    target_component: int,
) -> bool:
    source = message_source_ids(msg)
    if source is None:
        return False
    src_sys, src_comp = source
    if int(target_system) > 0 and src_sys != int(target_system):
        return False
    if int(target_component) > 0 and src_comp != int(target_component):
        return False
    return True


__all__ = ["message_source_ids", "source_matches_target_ids"]
