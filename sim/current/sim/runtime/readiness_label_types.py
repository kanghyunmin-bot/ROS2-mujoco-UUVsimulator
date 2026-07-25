"""Shared label constants for command-readiness policy."""

from __future__ import annotations


ReadinessLabel = tuple[str, str]

NOT_READY_STYLE = "NotReady.TLabel"
LIMITED_STYLE = "Limited.TLabel"
READY_STYLE = "Ready.TLabel"


__all__ = ["LIMITED_STYLE", "NOT_READY_STYLE", "READY_STYLE", "ReadinessLabel"]
