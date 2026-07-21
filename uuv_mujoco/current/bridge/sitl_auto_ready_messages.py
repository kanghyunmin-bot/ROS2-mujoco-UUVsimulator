"""Log-message helpers for SITL auto-ready sequencing."""

from __future__ import annotations


def auto_ready_started_message(target_mode: str) -> str:
    return (
        f"[sitl_transport] auto-ready sequence started: arm + {target_mode} "
        "(neutral RC only until operator input)"
    )


__all__ = ["auto_ready_started_message"]
