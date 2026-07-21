"""Message-loop helpers for ALT_HOLD DataFlash BIN readers."""

from __future__ import annotations

from pathlib import Path

from pymavlink import mavutil

from althold_contract_bin_streams import collect_bin_message, empty_streams


def collect_streams_from_bin(path: Path) -> tuple[dict[str, dict[str, list[float]]], list[dict], dict[str, float]]:
    streams = empty_streams()
    modes: list[dict] = []
    params: dict[str, float] = {}
    mlog = mavutil.mavlink_connection(str(path))
    while True:
        msg = mlog.recv_match()
        if msg is None:
            break
        collect_bin_message(
            streams=streams,
            modes=modes,
            params=params,
            msg_type=msg.get_type(),
            data=msg.to_dict(),
        )
    return streams, modes, params


__all__ = ["collect_streams_from_bin"]
