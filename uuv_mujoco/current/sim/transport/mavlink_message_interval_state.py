"""State helpers for throttled MAVLink message-interval requests."""

from __future__ import annotations


def last_request_wall(last_request_wall_by_key: dict[str, float], key: str) -> float:
    return float(last_request_wall_by_key.get(str(key), -1.0))


def request_due(last_request_wall_by_key: dict[str, float], key: str, now_wall: float, period_s: float) -> bool:
    return float(now_wall) - last_request_wall(last_request_wall_by_key, str(key)) >= float(period_s)


def mark_if_sent(last_request_wall_by_key: dict[str, float], key: str, now_wall: float, sent: bool) -> bool:
    if sent:
        last_request_wall_by_key[str(key)] = float(now_wall)
    return bool(sent)


__all__ = ["last_request_wall", "mark_if_sent", "request_due"]
