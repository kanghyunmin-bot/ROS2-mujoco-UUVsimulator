"""Frame and client bookkeeping for JSON servo packets."""

from __future__ import annotations


def record_json_servo_frame_state(self, servo_packet) -> None:
    if servo_packet.frame_rate_hz <= 0:
        return
    self._sitl_json_latest_frame_rate_hz = int(servo_packet.frame_rate_hz)
    self._sitl_json_latest_frame_count = int(servo_packet.frame_count)
    if self._sitl_json_first_frame_count is None:
        self._sitl_json_first_frame_count = int(servo_packet.frame_count)


def record_json_servo_client(self, addr, now_wall: float) -> None:
    if self._sitl_client_addr != addr:
        if self._sitl_client_addr is None:
            print(f"[sitl_transport] SITL servo endpoint discovered: {addr}", flush=True)
        else:
            print(
                f"[sitl_transport] SITL servo endpoint changed: "
                f"{self._sitl_client_addr} -> {addr}",
                flush=True,
            )
        self._sitl_client_addr = addr
        self._json_servo_receiver.client_addr = addr
        self._sitl_client_logged = True

    self._sitl_client_last_wall = now_wall
    if self._sitl_first_servo_wall <= 0.0:
        self._sitl_first_servo_wall = now_wall
    self._sitl_last_command_stale_wall = -1.0


def json_servo_packet_can_drive_plant(self, now_wall: float) -> bool:
    if self._sitl_mav is None or self._sitl_json_servo_fallback:
        return True
    if now_wall - self._sitl_json_servo_ignored_warn_wall > 3.0:
        print(
            "[sitl_transport] SITL(json) servo packet ignored because "
            "JSON servo is not the active plant source.",
            flush=True,
        )
        self._sitl_json_servo_ignored_warn_wall = now_wall
    return False


__all__ = [
    "json_servo_packet_can_drive_plant",
    "record_json_servo_client",
    "record_json_servo_frame_state",
]
