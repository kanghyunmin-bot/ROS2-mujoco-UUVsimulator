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
    )
    assert RovPolicyAdapter._ready(node, 10.1) is expected
