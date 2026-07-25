"""Per-cycle Ping360 publish cache state."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass
class Ping360PublishCache:
    sample: Any | None = None
    image_msg: Any | None = None
    scan_msg: Any | None = None
    echo_msg: Any | None = None
    status_msg: Any | None = None


__all__ = ["Ping360PublishCache"]
