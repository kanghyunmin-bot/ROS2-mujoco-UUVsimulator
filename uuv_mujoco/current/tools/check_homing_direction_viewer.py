#!/usr/bin/env python3
"""Regression checks for the MuJoCo pinger homing direction arrow."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
import sys
import time
from unittest.mock import patch

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim.runtime.viewer_scene_homing import build_homing_direction_arrow  # noqa: E402
from bridge import ros2_hydrophone_sim  # noqa: E402


BRIDGE_SOURCE = ROOT / "bridge" / "ros2_hydrophone_sim.py"


def _assert(condition: bool, label: str) -> None:
    if not condition:
        raise AssertionError(label)


def _bridge(*, active: bool = True, sample_time: float = 10.0):
    return SimpleNamespace(
        _hydrophone_last_estimated_direction_active=active,
        _hydrophone_last_estimated_direction_body=np.array([1.0, 0.0, 0.0]),
        _hydrophone_last_estimated_direction_wall=sample_time,
        _hydrophone_last_estimated_direction_source="none",
        _hydrophone_last_canonical_direction_wall=float("-inf"),
        _hydrophone_center_site_id=0,
    )


def _direction_msg(x: float, y: float, z: float, *, frame_id: str = "base_link"):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame_id),
        vector=SimpleNamespace(x=x, y=y, z=z),
    )


class _SubscriptionNode:
    def __init__(self) -> None:
        self.callbacks = {}

    def create_subscription(self, _msg_type, topic, callback, _qos):
        self.callbacks[topic] = callback
        return SimpleNamespace(topic=topic)


def check_body_direction_rotates_into_world() -> None:
    data = SimpleNamespace(time=10.1, site_xpos=np.array([[2.0, 3.0, -4.0]]))
    yaw_90 = np.array(
        [
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    arrow = build_homing_direction_arrow(
        _bridge(sample_time=time.monotonic()), data, yaw_90, arrow_length_m=2.0
    )
    _assert(arrow is not None, "fresh active direction must produce an arrow")
    assert arrow is not None
    direction = arrow.end - arrow.start
    _assert(np.allclose(direction, [0.0, 2.0, 0.0]), "body forward must rotate with vehicle yaw")
    _assert(np.allclose(arrow.start, [2.0, 3.12, -4.0]), "arrow must start at the hydrophone")


def check_inactive_and_stale_samples_are_hidden() -> None:
    data = SimpleNamespace(time=12.0, site_xpos=np.zeros((1, 3)))
    identity = np.eye(3)
    _assert(
        build_homing_direction_arrow(
            _bridge(active=False, sample_time=time.monotonic()), data, identity
        ) is None,
        "inactive signal must not be drawn",
    )
    _assert(
        build_homing_direction_arrow(
            _bridge(sample_time=time.monotonic() - 2.0), data, identity
        ) is None,
        "stale signal must not be drawn",
    )


def check_canonical_direction_has_freshness_priority() -> None:
    bridge = _bridge(active=False)
    bridge.node = _SubscriptionNode()
    bridge.Vector3Stamped = object
    ros2_hydrophone_sim.create_hydrophone_subscriptions(bridge, q10=10)
    canonical_callback = bridge.node.callbacks["/pinger_homing/direction_body"]
    raw_snr_callback = bridge.node.callbacks["/homing/direction"]

    with patch.object(ros2_hydrophone_sim.time, "monotonic", return_value=100.0):
        canonical_callback(_direction_msg(0.8, 0.0, -0.6))
    canonical_direction = bridge._hydrophone_last_estimated_direction_body.copy()
    _assert(
        np.allclose(canonical_direction, [0.8, 0.0, -0.6]),
        "canonical controller direction was not stored",
    )
    _assert(
        bridge._hydrophone_last_estimated_direction_source == "canonical",
        "canonical source label was not stored",
    )

    # The raw SNR estimator can publish an opposite elevation immediately
    # after the controller's fused body vector.  It is only a fallback and must
    # not replace the fresh canonical red-arrow input.
    with patch.object(ros2_hydrophone_sim.time, "monotonic", return_value=100.2):
        raw_snr_callback(_direction_msg(0.8, 0.0, 0.6))
    _assert(
        np.allclose(bridge._hydrophone_last_estimated_direction_body, canonical_direction),
        "raw SNR fallback overwrote a fresh canonical direction",
    )
    _assert(
        bridge._hydrophone_last_estimated_direction_wall == 100.0,
        "ignored fallback refreshed the canonical sample timestamp",
    )

    # Once the canonical sample is older than the same window used by the
    # viewer, the raw body-frame estimate may resume as a genuine fallback.
    expired_wall = (
        100.0
        + ros2_hydrophone_sim.CANONICAL_HOMING_DIRECTION_FRESHNESS_S
        + 0.01
    )
    with patch.object(
        ros2_hydrophone_sim.time, "monotonic", return_value=expired_wall
    ):
        raw_snr_callback(_direction_msg(0.8, 0.0, 0.6))
    _assert(
        np.allclose(
            bridge._hydrophone_last_estimated_direction_body, [0.8, 0.0, 0.6]
        ),
        "raw SNR fallback did not resume after canonical expiry",
    )
    _assert(
        bridge._hydrophone_last_estimated_direction_source == "raw_snr_fallback",
        "fallback source label was not stored after canonical expiry",
    )


def main() -> int:
    _assert(
        '"/pinger_homing/direction_body"' in BRIDGE_SOURCE.read_text(encoding="utf-8"),
        "viewer bridge must consume the C++ pinger body-direction topic",
    )
    check_body_direction_rotates_into_world()
    check_inactive_and_stale_samples_are_hidden()
    check_canonical_direction_has_freshness_priority()
    print("homing_direction_viewer=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
