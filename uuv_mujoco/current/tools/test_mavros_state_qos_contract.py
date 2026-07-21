#!/usr/bin/env python3
"""Regression check for the simulator's MAVROS state durability contract."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from bridge.ros2_endpoint_mavros_publishers import create_mavros_publishers


class FakeNode:
    def __init__(self) -> None:
        self.publishers: dict[str, object] = {}

    def create_publisher(self, _msg_type, topic: str, qos: object) -> str:
        self.publishers[topic] = qos
        return topic


def test_mavros_state_uses_latched_qos_only() -> None:
    node = FakeNode()
    bridge = SimpleNamespace(
        node=node,
        _mavros_surface_enabled=True,
        RCOut=object,
    )
    for attr in (
        "VfrHud",
        "MavrosState",
        "Imu",
        "FluidPressure",
        "PoseStamped",
        "Odometry",
        "TwistStamped",
        "TwistWithCovarianceStamped",
        "BatteryState",
        "RCIn",
    ):
        setattr(bridge, attr, object)

    regular_qos = object()
    latched_qos = object()
    create_mavros_publishers(
        bridge,
        q10=regular_qos,
        latched_qos=latched_qos,
    )

    assert node.publishers["/mavros/state"] is latched_qos
    assert node.publishers["/mavros/imu/data"] is regular_qos
    assert node.publishers["/mavros/rc/out"] is regular_qos


if __name__ == "__main__":
    test_mavros_state_uses_latched_qos_only()
    print("mavros_state_qos_contract=PASS")
