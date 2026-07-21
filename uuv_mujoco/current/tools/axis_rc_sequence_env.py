"""Environment-backed RC sequence policy helpers."""

from __future__ import annotations

import os


def althold_heave_is_inverted(mode: str) -> bool:
    enabled = os.environ.get("UUV_ALT_HOLD_RC_HEAVE_INVERT", "0").strip().lower()
    return str(mode).upper() == "ALT_HOLD" and enabled in {"1", "true", "yes", "on", "enable", "enabled"}


__all__ = ["althold_heave_is_inverted"]
