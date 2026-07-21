"""RC channel field helpers for axis RC samples."""

from __future__ import annotations

from typing import Any


def add_channels(sample: dict[str, Any], prefix: str, msg: Any) -> None:
    if msg is None:
        return
    channels = [int(value) for value in getattr(msg, "channels", [])]
    for idx, value in enumerate(channels[:8], start=1):
        sample[f"{prefix}{idx}"] = value


__all__ = ["add_channels"]
