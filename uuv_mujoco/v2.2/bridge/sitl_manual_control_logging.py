"""Logging and control-activity bookkeeping for MANUAL_CONTROL sends."""

from __future__ import annotations

import time

from bridge.sitl_manual_control_frame import ManualControlFrame


def mark_manual_control_sent(owner, *, target_sys: int, frame: ManualControlFrame) -> None:
    # MANUAL_CONTROL is also pilot input. Treat it as recent external control
    # so neutral RC keepalive cannot interleave with joystick commands when
    # keepalive is enabled for non-GUI runs.
    owner._sitl_last_external_rc_override_wall = time.monotonic()
    if not owner._sitl_cmd_debug:
        return
    now = time.monotonic()
    if now - owner._sitl_last_manual_control_log_wall <= 0.2:
        return
    print(
        "[sitl_transport] MANUAL_CONTROL forwarded to ArduSub "
        f"target={target_sys} axes=(x={frame.x}, y={frame.y}, "
        f"z={frame.z}, r={frame.r}) buttons={frame.buttons}",
        flush=True,
    )
    owner._sitl_last_manual_control_log_wall = now


__all__ = ["mark_manual_control_sent"]
