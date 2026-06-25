"""Ping360 ROS2 publisher endpoint construction."""

from __future__ import annotations


def create_ping360_publishers(bridge, *, q10, q1) -> None:
    node = bridge.node
    bridge.pub_ping360_image = node.create_publisher(bridge.Image, "/ping360/image", q1)
    bridge.pub_ping360_scan_image = node.create_publisher(bridge.Image, "/ping360/scan_image", q1)
    bridge.pub_ping360_scan = node.create_publisher(bridge.LaserScan, "/ping360/scan", q10)
    bridge.pub_ping360_echo = (
        node.create_publisher(bridge.SonarEcho, "/ping360/scan_echo", q10)
        if bridge.SonarEcho
        else None
    )
    bridge.pub_ping360_echo_alias = (
        node.create_publisher(bridge.SonarEcho, "/ping360/echo", q10)
        if bridge.SonarEcho
        else None
    )
    bridge.pub_ping360_status = node.create_publisher(bridge.String, "/ping360/status", q10)


__all__ = ["create_ping360_publishers"]
