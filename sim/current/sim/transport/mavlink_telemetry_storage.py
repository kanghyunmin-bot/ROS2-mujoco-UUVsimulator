"""MAVLink telemetry status-data storage helpers."""

from __future__ import annotations


def store_fields(
    status_data: dict[str, object],
    prefix: str,
    data: dict[str, object],
    now_wall: float,
    fields: tuple[str, ...],
    *,
    set_wall: bool = True,
) -> None:
    if set_wall:
        status_data[f"{prefix}_wall_s"] = float(now_wall)
    for field_name in fields:
        if field_name in data:
            status_data[f"{prefix}_{field_name}"] = data.get(field_name)


def store_heartbeat(status_data: dict[str, object], msg: object, data: dict[str, object], now_wall: float) -> None:
    prefix = "heartbeat"
    status_data[f"{prefix}_wall_s"] = float(now_wall)
    try:
        status_data[f"{prefix}_src_system"] = int(msg.get_srcSystem())
        status_data[f"{prefix}_src_component"] = int(msg.get_srcComponent())
    except Exception:
        pass
    store_fields(
        status_data,
        prefix,
        data,
        now_wall,
        ("type", "autopilot", "base_mode", "custom_mode", "system_status", "mavlink_version"),
        set_wall=False,
    )


def store_rc_channels(status_data: dict[str, object], data: dict[str, object], now_wall: float) -> None:
    prefix = "rc_channels"
    store_fields(status_data, prefix, data, now_wall, ("time_boot_ms", "chancount", "rssi"))
    for idx in range(1, 19):
        key = f"chan{idx}_raw"
        if key in data:
            status_data[f"{prefix}_ch{idx}"] = data.get(key)


__all__ = ["store_fields", "store_heartbeat", "store_rc_channels"]
