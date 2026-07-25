"""Metadata filtering for control-loop golden fingerprints."""

from __future__ import annotations

from typing import Any

from control_loop_golden_fingerprint_fields import METADATA_KEYS


def build_fingerprint_metadata(payload: dict[str, Any]) -> dict[str, Any]:
    metadata = payload.get("metadata", {})
    return {key: metadata.get(key) for key in METADATA_KEYS if key in metadata}


__all__ = ["build_fingerprint_metadata"]
