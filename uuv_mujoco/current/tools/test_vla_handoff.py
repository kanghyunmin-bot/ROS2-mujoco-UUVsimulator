"""Ownership transitions cannot leave joystick and policy publishing together."""
import sys
import threading
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gui.vla_control_handoff import prepare_vla_control, restore_manual_control


def controller():
    return SimpleNamespace(
        _pinger_rc_handoff_lock=threading.RLock(), _vla_control_prepared=False,
        recorder=SimpleNamespace(require_configuration_unlocked=Mock()),
        processes=SimpleNamespace(simulation_runtime_available=lambda: True,
            pinger_homing_running=lambda: False, mission_running=lambda: False),
        replay=SimpleNamespace(running=lambda: False),
        release_rc=Mock(),
        node=SimpleNamespace(suspend_rc_override_publisher=Mock(return_value=True),
            restore_rc_override_publisher=Mock(return_value=True),
            push_event=Mock(), count_publishers=Mock(return_value=0),
            _rc_override_topic="/mavros/rc/override"),
    )


def test_release_precedes_suspend_and_prepare_is_idempotent():
    c = controller()
    events = []
    c.release_rc.side_effect = lambda: events.append("release")
    c.node.suspend_rc_override_publisher.side_effect = lambda: events.append("suspend") or True
    assert prepare_vla_control(c)["vla_prepared"]
    prepare_vla_control(c)
    assert events == ["release", "suspend"]
    assert restore_manual_control(c)["vla_prepared"] is False
    c.node.restore_rc_override_publisher.assert_called_once()


def test_cannot_restore_while_policy_still_publishes():
    c = controller()
    prepare_vla_control(c)
    c.node.count_publishers.return_value = 1
    with pytest.raises(ValueError, match="정책"):
        restore_manual_control(c)
    assert c._vla_control_prepared
    c.node.restore_rc_override_publisher.assert_not_called()


def test_suspend_failure_preserves_manual_state():
    c = controller()
    c.node.suspend_rc_override_publisher.return_value = False
    with pytest.raises(ValueError):
        prepare_vla_control(c)
    assert not c._vla_control_prepared


def test_recording_or_replay_prevents_handoff():
    c = controller()
    c.recorder.require_configuration_unlocked.side_effect = ValueError("session")
    with pytest.raises(ValueError):
        prepare_vla_control(c)
    c.node.suspend_rc_override_publisher.assert_not_called()
    c.recorder.require_configuration_unlocked.side_effect = None
    c.replay.running = lambda: True
    with pytest.raises(ValueError, match="재생"):
        prepare_vla_control(c)
