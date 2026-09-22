"""Web reset must preserve active recordings and await simulator completion."""
import sys
import threading
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gui.web_recorder import WebRecorder


def recorder(**changes):
    r = WebRecorder.__new__(WebRecorder)
    r.lock = threading.RLock()
    state = dict(online=True, owned=True, active=False, busy=False, expected_mode='STABILIZE')
    state.update(changes)
    r.payload = lambda: state
    r.clients = {'demo_reset': Mock()}
    r.clients['demo_reset'].service_is_ready.return_value = True
    r.future = None
    return r


@pytest.mark.parametrize('changes', [dict(active=True), dict(online=False), dict(owned=False), dict(busy=True), dict(expected_mode='ALT_HOLD')])
def test_reset_rejected_without_releasing_or_calling_simulator(changes):
    r = recorder(**changes)
    release = Mock()
    with pytest.raises(ValueError):
        r.reset_scene(release)
    release.assert_not_called()
    r.clients['demo_reset'].call_async.assert_not_called()


def test_idle_reset_releases_input_and_keeps_busy_until_ack():
    r = recorder()
    r.session = 'unchanged_session'
    release = Mock()
    r.reset_scene(release)
    release.assert_called_once()
    assert r.resetting and r.future is not None
    future = r.future
    future.result.return_value = NS(success=True, message='reset done')
    r._command_done(future)
    assert r.session == 'unchanged_session'
    assert r.future is None and not r.resetting and r.received == 0


def test_missing_reset_service_keeps_current_control():
    r = recorder()
    r.clients['demo_reset'].service_is_ready.return_value = False
    release = Mock()
    with pytest.raises(ValueError):
        r.reset_scene(release)
    release.assert_not_called()
