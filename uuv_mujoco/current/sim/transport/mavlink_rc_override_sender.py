"""Low-level MAVLink RC_CHANNELS_OVERRIDE sender."""

from __future__ import annotations


def send_rc_channels_override_on_link(
    mav: object | None,
    target_sys: int,
    target_comp: int,
    values: list[int],
) -> int:
    """Send dist-style 8-channel RC_CHANNELS_OVERRIDE.

    ArduSub 4.1.2 consumes the pilot axes from channels 1..8 here.  Keeping the
    payload to eight fields matches the older dist bundle/QGC path and avoids
    MAVLink2 extension-channel release/no-change semantics on channels the sub
    does not need for pilot control.
    """
    if mav is None:
        return 0
    payload = [int(v) for v in values[:8]]
    if len(payload) < 8:
        payload.extend([65535] * (8 - len(payload)))
    mav.mav.rc_channels_override_send(
        int(target_sys),
        int(target_comp),
        *payload,
    )
    return len(payload)


__all__ = ["send_rc_channels_override_on_link"]
