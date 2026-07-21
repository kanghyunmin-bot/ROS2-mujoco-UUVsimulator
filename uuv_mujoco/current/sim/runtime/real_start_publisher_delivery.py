"""Publish/log delivery for real-start status payloads."""

from __future__ import annotations

from sim.runtime.real_start_status_logging import (
    log_real_start_ok,
    log_real_start_released,
    real_start_status_is_ok,
    real_start_status_is_released,
)
from sim.runtime.real_start_status_publish import publish_real_start_status


def publish_and_log_real_start_status(owner, payload: dict[str, object]) -> None:
    if not publish_real_start_status(owner.publisher, owner.string_type, payload):
        return
    if real_start_status_is_ok(payload) and not owner.logged_ok:
        owner.logged_ok = True
        log_real_start_ok(payload)
    if real_start_status_is_released(payload) and not owner.logged_released:
        owner.logged_released = True
        log_real_start_released()


__all__ = ["publish_and_log_real_start_status"]
