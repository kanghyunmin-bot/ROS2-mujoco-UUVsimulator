"""ArduSub custom-mode lookup helpers."""

from __future__ import annotations

ARDUSUB_MODE_FALLBACKS: dict[str, int] = {
    "STABILIZE": 0,
    "ACRO": 1,
    "ALT_HOLD": 2,
    "AUTO": 3,
    "GUIDED": 4,
    "CIRCLE": 7,
    "SURFACE": 9,
    "POSHOLD": 16,
    "MANUAL": 19,
}


def resolve_ardusub_mode_id(mode: str, mode_map: dict[str, int] | None = None) -> tuple[str, int] | None:
    """Resolve an ArduSub mode name or numeric string to a custom_mode id."""
    mode_text = str(mode or "").strip()
    if not mode_text:
        return None
    upper = mode_text.upper()
    lookup = {str(key).upper(): int(value) for key, value in (mode_map or {}).items()}
    if upper in lookup:
        return mode_text, lookup[upper]
    if upper in ARDUSUB_MODE_FALLBACKS:
        return mode_text, int(ARDUSUB_MODE_FALLBACKS[upper])
    return mode_text, int(mode_text)
