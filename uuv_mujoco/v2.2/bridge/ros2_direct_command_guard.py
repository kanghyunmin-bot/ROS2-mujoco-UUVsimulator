"""SITL closed-loop guard for direct MuJoCo command callbacks."""

from __future__ import annotations


def direct_command_blocked_by_sitl(bridge) -> bool:
    return bridge._sitl_transport is not None and not bridge._sitl_allow_direct_cmd


def warn_direct_command_blocked_once(bridge) -> None:
    if bridge._sitl_direct_cmd_blocked_warned or bridge.node is None:
        return
    bridge._sitl_direct_cmd_blocked_warned = True
    bridge.node.get_logger().warn(
        "direct MuJoCo command callback ignored in SITL closed-loop mode; "
        "ArduSub JSON servo is the only plant input."
    )


__all__ = ["direct_command_blocked_by_sitl", "warn_direct_command_blocked_once"]
