"""Action chunks advance on the dataset clock even with slow simulation."""

from concurrent.futures import Future
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip("rclpy")
from kmu26_auv_vla_policy.kmu26_ros import RovPolicyAdapter


def test_slow_sim_uses_ros_age_for_chunk(monkeypatch):
    monkeypatch.setattr("kmu26_auv_vla_policy.kmu26_ros.time.monotonic", lambda: 100.2)
    n = SimpleNamespace(
        last_tick=100.1,
        sensors=SimpleNamespace(_now=lambda: 10.05, policy_observation=lambda p: {}),
        last_ros_time=10.0,
        clock_advanced_at=100.0,
        enabled=True,
        timeout=0.3,
        _ready=lambda t: True,
        future=Future(),
        chunk=np.tile(np.arange(16)[:, None], (1, 4)) * 0.01,
        chunk_at=100.0,
        chunk_ros_at=10.0,
        requested_at=100.0,
        last_command_ros_at=None,
        owned=False,
    )
    seen = []
    n.limiter = SimpleNamespace(
        previous=np.zeros(4), apply=lambda action, dt: seen.append(action.copy())
    )
    n._publish = lambda x: None
    n._stop = lambda: pytest.fail("Unexpected stop")
    n.get_logger = lambda: SimpleNamespace(warning=lambda msg: pytest.fail(msg))
    RovPolicyAdapter._tick(n)
    np.testing.assert_equal(seen[0], np.zeros(4))


@pytest.mark.parametrize("publishers,expected", [(1, True), (2, False), (0, False)])
def test_rc_requires_exclusive_ros_publisher(publishers, expected):
    node = SimpleNamespace(
        deadman_at=10.0,
        dry_run=False,
        state_at=10.0,
        vehicle_state=SimpleNamespace(connected=True, armed=True, mode="STABILIZE"),
        expected_mode="STABILIZE",
        output=SimpleNamespace(topic_name="/mavros/rc/override"),
        count_publishers=lambda topic: publishers,
        sensors=SimpleNamespace(
            get_parameter=lambda name: SimpleNamespace(value=False)
        ),
    )
    assert RovPolicyAdapter._ready(node, 10.1) is expected


@pytest.mark.parametrize(
    "sim_time,wall_age,ros_age,clock_age,expected",
    [
        (True, 4.0, 0.5, 0.0, True),
        (True, 0.5, 1.0025, 0.0, True),
        (True, 0.5, 2.1, 0.0, False),
        (True, 0.2, 0.0, 0.31, False),
        (True, 0.1, -0.1, 0.0, False),
        (False, 0.9, 2.0, 0.0, True),
        (False, 1.1, 0.5, 0.0, False),
    ],
)
def test_policy_vehicle_state_uses_sensor_clock(
    sim_time, wall_age, ros_age, clock_age, expected
):
    now = 100.0 + wall_age
    node = SimpleNamespace(
        deadman_at=now,
        dry_run=False,
        state_at=100.0,
        state_ros_at=10.0,
        vehicle_state=SimpleNamespace(connected=True, armed=True, mode="STABILIZE"),
        expected_mode="STABILIZE",
        timeout=0.3,
        last_ros_time=10.0 + max(0.0, ros_age),
        clock_advanced_at=now - clock_age,
        output=SimpleNamespace(topic_name="/mavros/rc/override"),
        count_publishers=lambda topic: 1,
        sensors=SimpleNamespace(
            _now=lambda: 10.0 + ros_age,
            get_parameter=lambda name: SimpleNamespace(value=sim_time),
        ),
    )
    assert RovPolicyAdapter._ready(node, now) is expected
    node.deadman_at = now - 0.31
    assert not RovPolicyAdapter._ready(node, now), (
        "Deadman expiry must remain wall-time based"
    )


def test_policy_rewind_invalidates_vehicle_even_when_disabled(monkeypatch):
    monkeypatch.setattr("kmu26_auv_vla_policy.kmu26_ros.time.monotonic", lambda: 100.1)
    node = SimpleNamespace(
        last_tick=100.0,
        sensors=SimpleNamespace(
            _now=lambda: 9.0, get_parameter=lambda name: SimpleNamespace(value=True)
        ),
        last_ros_time=10.0,
        clock_advanced_at=100.0,
        enabled=False,
        vehicle_state=object(),
        state_at=100.0,
        state_ros_at=10.0,
    )
    RovPolicyAdapter._tick(node)
    assert node.vehicle_state is None
    assert node.state_at == node.state_ros_at == -float("inf")
