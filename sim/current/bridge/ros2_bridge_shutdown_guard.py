"""Exception guard for best-effort Ros2Bridge shutdown steps."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any


def run_shutdown_step(action: Callable[..., None], *args: Any) -> None:
    try:
        action(*args)
    except Exception:
        pass


__all__ = ["run_shutdown_step"]
