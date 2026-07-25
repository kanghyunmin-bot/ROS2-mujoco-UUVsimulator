"""DataFlash message-to-stream collection helpers."""

from __future__ import annotations

import math
from collections import defaultdict
from typing import Iterable

from althold_contract_bin_schema import BIN_STREAM_FIELDS
from althold_contract_model import MODE_NAMES


def empty_streams() -> dict[str, dict[str, list[float]]]:
    return defaultdict(lambda: defaultdict(list))


def append_value(streams: dict[str, dict[str, list[float]]], msg_type: str, msg: dict, fields: Iterable[str]) -> None:
    t = float(msg["TimeUS"]) / 1.0e6
    streams[msg_type]["t"].append(t)
    for field in fields:
        streams[msg_type][field].append(float(msg.get(field, math.nan)))


def collect_bin_message(
    *,
    streams: dict[str, dict[str, list[float]]],
    modes: list[dict],
    params: dict[str, float],
    msg_type: str,
    data: dict,
) -> None:
    if "TimeUS" not in data:
        return
    if msg_type == "MODE":
        append_mode(modes, data)
        return
    if msg_type == "PARM":
        append_param(params, data)
        return
    if msg_type in BIN_STREAM_FIELDS:
        append_value(streams, msg_type, data, BIN_STREAM_FIELDS[msg_type])


def append_mode(modes: list[dict], data: dict) -> None:
    mode_num = int(data.get("ModeNum", data.get("Mode", -1)))
    modes.append(
        {
            "t": float(data["TimeUS"]) / 1.0e6,
            "mode": mode_num,
            "name": MODE_NAMES.get(mode_num, str(mode_num)),
        }
    )


def append_param(params: dict[str, float], data: dict) -> None:
    name = str(data.get("Name", ""))
    try:
        params[name] = float(data.get("Value", math.nan))
    except (TypeError, ValueError):
        pass


__all__ = ["collect_bin_message", "empty_streams"]
