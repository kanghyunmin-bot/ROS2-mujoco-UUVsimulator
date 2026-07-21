"""Publish policy for MAVROS-compatible RCOut telemetry."""

from __future__ import annotations


def publish_rcout_event_if_enabled(bridge, rc_out) -> None:
    if bridge._mavros_rc_out_publish_mode == "event" and bridge.pub_mavros_rc_out is not None:
        bridge._safe_publish(bridge.pub_mavros_rc_out, rc_out, "/mavros/rc/out")


__all__ = ["publish_rcout_event_if_enabled"]
