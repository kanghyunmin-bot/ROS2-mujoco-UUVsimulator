"""Exercise RC handoff interleavings without ROS, sockets, or timing sleeps."""

from pathlib import Path
import sys
import threading
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from bridge import ros2_rc_override_forward_cache as cache  # noqa: E402
from bridge.ros2_rc_override_callback import _on_mavros_rc_override  # noqa: E402


def make_bridge():
    bridge = SimpleNamespace(
        _mavros_rc_override_cache_lock=threading.RLock(),
        _sitl_transport_lock=threading.RLock(),
        _mavros_pending_rc_override_channels=None,
        _mavros_pending_rc_override_wall=-1.0,
        _mavros_last_forwarded_rc_override_wall=-1.0,
        _mavros_rc_override_forward_period_s=0.0,
        _mavros_rc_override_stale_s=3.0,
        _mavros_rc_override_backend="rc_override",
        _mavros_rc_override_local_fallback=False,
        _mavros_rc_pwm_span=400.0,
        node=None,
        RCIn=SimpleNamespace,
        pub_mavros_rc_in=None,
        _safe_publish=lambda *args: None,
    )
    for axis, channel in [("forward", 5), ("sway", 6), ("yaw", 4), ("heave", 3)]:
        setattr(bridge, "_mavros_rc_" + axis + "_channel", channel)
        setattr(bridge, "_mavros_rc_" + axis + "_invert", False)
    bridge.forwarded = []
    bridge._sitl_transport = SimpleNamespace(
        send_rc_override=lambda channels: bridge.forwarded.append(list(channels)) or True,
    )
    return bridge


def frame(pwm):
    result = [1500] * 18
    result[4] = pwm
    return result


def test_arrival_during_send_is_not_deleted_by_older_success():
    bridge = make_bridge()
    old, new = frame(1600), frame(1700)
    cache.store_pending_rc_override(bridge, old)

    def send(owner, channels):
        cache.store_pending_rc_override(owner, new)
        owner.forwarded.append(list(channels))
        return True

    with patch.object(cache, "forward_rc_override_to_sitl", send):
        cache.service_pending_rc_override_forward(bridge)
    assert bridge._mavros_pending_rc_override_channels == new
    cache.service_pending_rc_override_forward(bridge)
    assert bridge.forwarded == [old, new]
    assert bridge._mavros_pending_rc_override_channels is None


def test_same_values_and_same_clock_still_represent_a_new_receipt():
    bridge = make_bridge()
    command = frame(1600)
    with patch.object(cache.time, "monotonic", return_value=100.0):
        cache.store_pending_rc_override(bridge, command)

        def send(owner, channels):
            cache.store_pending_rc_override(owner, command)
            return True

        with patch.object(cache, "forward_rc_override_to_sitl", send):
            cache.service_pending_rc_override_forward(bridge)
    assert bridge._mavros_pending_rc_override_channels == command


def test_callback_cannot_send_new_then_allow_an_older_cached_send():
    bridge = make_bridge()
    old, new = frame(1600), frame(1700)
    cache.store_pending_rc_override(bridge, old)
    entered, release = threading.Event(), threading.Event()
    original = cache.forward_rc_override_to_sitl
    errors = []

    def delayed_send(owner, channels):
        entered.set()
        if not release.wait(2.0):
            raise AssertionError("test handoff was not released")
        return original(owner, channels)

    def consume():
        try:
            cache.service_pending_rc_override_forward(bridge)
        except BaseException as error:
            errors.append(error)

    with patch.object(cache, "forward_rc_override_to_sitl", delayed_send):
        worker = threading.Thread(target=consume)
        worker.start()
        try:
            assert entered.wait(2.0)
            _on_mavros_rc_override(bridge, SimpleNamespace(channels=new))
        finally:
            release.set()
            worker.join(2.0)
    assert not worker.is_alive() and not errors
    cache.service_pending_rc_override_forward(bridge)
    assert bridge.forwarded == [old, new]


def test_failed_send_retains_newest_pending_frame():
    bridge = make_bridge()
    cache.store_pending_rc_override(bridge, frame(1600))

    def fail(owner, channels):
        cache.store_pending_rc_override(owner, frame(1700))
        return False

    with patch.object(cache, "forward_rc_override_to_sitl", fail), patch.object(
        cache, "_warn_rc_override_not_forwarded"
    ):
        cache.service_pending_rc_override_forward(bridge)
    assert bridge._mavros_pending_rc_override_channels == frame(1700)


def test_forward_period_and_stale_timeout_are_preserved():
    bridge = make_bridge()
    bridge._mavros_rc_override_forward_period_s = 0.1
    with patch.object(cache.time, "monotonic", return_value=10.0):
        cache.store_pending_rc_override(bridge, frame(1600))
        cache.service_pending_rc_override_forward(bridge)
    with patch.object(cache.time, "monotonic", return_value=10.01):
        cache.store_pending_rc_override(bridge, frame(1700))
        cache.service_pending_rc_override_forward(bridge)
    assert bridge.forwarded == [frame(1600)]
    with patch.object(cache.time, "monotonic", return_value=10.11):
        cache.service_pending_rc_override_forward(bridge)
    assert bridge.forwarded == [frame(1600), frame(1700)]
    with patch.object(cache.time, "monotonic", return_value=11.0):
        cache.store_pending_rc_override(bridge, frame(1800))
    with patch.object(cache.time, "monotonic", return_value=15.0):
        cache.service_pending_rc_override_forward(bridge)
    assert bridge.forwarded == [frame(1600), frame(1700)]


def test_direct_control_fallback_still_receives_the_callback():
    bridge = make_bridge()
    bridge._sitl_transport = None
    bridge.commands = []
    bridge._handle_normalized_cmd = lambda *values: bridge.commands.append(values)
    _on_mavros_rc_override(bridge, SimpleNamespace(channels=frame(1700)))
    assert len(bridge.commands) == 1
    assert not bridge.forwarded
