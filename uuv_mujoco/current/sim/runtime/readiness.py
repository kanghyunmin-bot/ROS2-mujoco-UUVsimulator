"""Compatibility exports for runtime readiness state and label policy.

This module defines readiness as a set of explicit gates instead of one GUI
boolean.  It is intentionally dependency-free so GUI, scripts, and validation
tools can share the same state vocabulary.
"""

from __future__ import annotations

from .readiness_label import command_readiness_label
from .readiness_types import CommandReadinessInputs, RuntimeReadiness


__all__ = ["CommandReadinessInputs", "RuntimeReadiness", "command_readiness_label"]
