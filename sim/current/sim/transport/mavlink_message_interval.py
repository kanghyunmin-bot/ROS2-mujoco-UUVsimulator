"""MAVLink SET_MESSAGE_INTERVAL request scheduling helpers."""

from __future__ import annotations

from dataclasses import dataclass, field

from .mavlink_message_constants import DEFAULT_AP_TELEMETRY_MESSAGE_CONSTANTS
from .mavlink_message_interval_requests import request_named_message_intervals, request_servo_output_raw_interval
from .mavlink_message_interval_send import (
    send_message_interval,
)
from .mavlink_message_interval_state import last_request_wall


@dataclass
class MavlinkMessageIntervalRequester:
    """Throttle and send MAVLink message-interval requests by stream key."""

    last_request_wall: dict[str, float] = field(default_factory=dict)

    def last_wall(self, key: str) -> float:
        return last_request_wall(self.last_request_wall, key)

    def request_servo_output_raw(
        self,
        *,
        key: str,
        mav: object | None,
        mavutil: object | None,
        target_sys: int,
        target_comp: int,
        now_wall: float,
        requested_hz: float,
        period_s: float,
    ) -> bool:
        return request_servo_output_raw_interval(
            self.last_request_wall,
            key=key,
            mav=mav,
            mavutil=mavutil,
            target_sys=target_sys,
            target_comp=target_comp,
            now_wall=now_wall,
            requested_hz=requested_hz,
            period_s=period_s,
        )

    def request_named_messages(
        self,
        *,
        key: str,
        mav: object | None,
        mavutil: object | None,
        target_sys: int,
        target_comp: int,
        now_wall: float,
        requested_hz: float,
        period_s: float,
        message_constant_names: tuple[str, ...] = DEFAULT_AP_TELEMETRY_MESSAGE_CONSTANTS,
    ) -> bool:
        return request_named_message_intervals(
            self.last_request_wall,
            key=key,
            mav=mav,
            mavutil=mavutil,
            target_sys=target_sys,
            target_comp=target_comp,
            now_wall=now_wall,
            requested_hz=requested_hz,
            period_s=period_s,
            message_constant_names=message_constant_names,
        )

    @staticmethod
    def _send_message_interval(
        *,
        mav: object | None,
        mavlink_defs: object | None,
        target_sys: int,
        target_comp: int,
        message_id: int,
        requested_hz: float,
    ) -> bool:
        return send_message_interval(
            mav=mav,
            mavlink_defs=mavlink_defs,
            target_sys=target_sys,
            target_comp=target_comp,
            message_id=message_id,
            requested_hz=requested_hz,
        )


__all__ = [
    "DEFAULT_AP_TELEMETRY_MESSAGE_CONSTANTS",
    "MavlinkMessageIntervalRequester",
]
