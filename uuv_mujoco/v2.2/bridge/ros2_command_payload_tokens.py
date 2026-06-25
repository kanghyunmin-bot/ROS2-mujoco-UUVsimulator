"""Token parsing helpers for ROS2 command override payloads."""

from __future__ import annotations


def _split_payload_token(token: str) -> tuple[str, str] | None:
    if "=" in token:
        key, value = token.split("=", 1)
    elif ":" in token:
        key, value = token.split(":", 1)
    else:
        parts = token.split(None, 1)
        if len(parts) != 2:
            return None
        key, value = parts
    return key.strip().lower(), value.strip()


def _parse_payload_tokens(raw: str) -> dict[str, object]:
    payload: dict[str, object] = {}
    normalized = raw.replace(",", " ")
    for token in normalized.split():
        parsed = _split_payload_token(token)
        if parsed is None:
            continue
        key, value = parsed
        if key:
            payload[key] = value
    return payload


__all__ = ["_parse_payload_tokens", "_split_payload_token"]
