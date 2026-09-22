"""Control ownership and lifecycle contract for the GUI collection worker."""
import sys
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gui.web_auto_collection import WebAutoCollection


class AutomaticCollection(unittest.TestCase):
    def setUp(self):
        self.manager = WebAutoCollection(SimpleNamespace())
        self.manager.process = Mock()
        self.manager.process.poll.return_value = None
        self.manager.token = 'private-worker-token'

    def test_only_worker_can_control_while_running(self):
        with self.assertRaises(ValueError):
            self.manager.authorize({'command': 'rc'})
        self.manager.authorize({'command': 'rc', '_auto_collection_token': self.manager.token})
        self.manager.authorize({'command': 'auto_collection_stop'})

    def test_stop_signals_once_and_keeps_running_until_cleanup(self):
        self.manager.stop()
        self.manager.stop()
        self.manager.process.send_signal.assert_called_once()
        self.assertTrue(self.manager.payload()['running'])
        self.assertTrue(self.manager.payload()['stopping'])

    def test_duplicate_start_is_rejected(self):
        with self.assertRaises(ValueError):
            self.manager.start(3)

    def test_invalid_count_is_rejected_before_any_controller_action(self):
        self.manager.process = None
        for count in (0, -1, True, 1.5, 10001, 'bad'):
            with self.subTest(count=count), self.assertRaises(ValueError):
                self.manager.start(count)

    def test_token_never_exposed_in_status(self):
        self.assertNotIn(self.manager.token, str(self.manager.payload()))
