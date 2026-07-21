#!/usr/bin/env python3
"""Regression tests for simulator-exit MAVROS/process-group cleanup."""

from __future__ import annotations

import signal
import sys
import tempfile
import threading
import unittest
from pathlib import Path
from unittest import mock


CURRENT_DIR = Path(__file__).resolve().parents[1]
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))

from gui.process_termination import terminate_exited_process_group, terminate_process_group  # noqa: E402
from gui.web_process_manager import WebProcessManager  # noqa: E402


class _FakeProcess:
    def __init__(self, *, pid: int = 4242, return_code: int = 0) -> None:
        self.pid = pid
        self.return_code = return_code

    def wait(self) -> int:
        return self.return_code

    def poll(self) -> int:
        return self.return_code


class _TerminatingLeader:
    def __init__(self, *, pid: int = 4242) -> None:
        self.pid = pid
        self.return_code: int | None = None

    def wait(self, timeout: float | None = None) -> int:
        del timeout
        self.return_code = 0
        return 0

    def poll(self) -> int | None:
        return self.return_code


class _RunningPinger:
    def __init__(self, *, pid: int = 4243) -> None:
        self.pid = pid
        self.return_code: int | None = None

    def wait(self) -> int:
        assert self.return_code is not None
        return self.return_code

    def poll(self) -> int | None:
        return self.return_code


class _FakeNode:
    def __init__(self) -> None:
        self.events: list[str] = []

    def push_event(self, message: str) -> None:
        self.events.append(message)


class SimExitCleanupTest(unittest.TestCase):
    def test_pinger_terminal_state_closes_launch_group_and_clears_running(self) -> None:
        manager = WebProcessManager.__new__(WebProcessManager)
        manager.node = _FakeNode()
        manager._lock = threading.Lock()
        process = _RunningPinger()
        manager._pinger_homing_process = process
        manager._pinger_homing_status = "pinger homing: running"

        with tempfile.TemporaryDirectory() as directory:
            log_path = Path(directory) / "pinger.log"
            log_path.write_text(
                "[INFO] C++ homing state: NO_ODOM_PHASE_PROBE -> FAILED_TIMEOUT\n",
                encoding="utf-8",
            )

            def terminate(proc, _timeout_s):
                self.assertIs(proc, process)
                process.return_code = -signal.SIGTERM

            with mock.patch(
                "gui.web_process_manager.terminate_process_group",
                side_effect=terminate,
            ) as stop_group:
                manager._watch_process(
                    proc=process,
                    log_path=log_path,
                    label="pinger homing",
                    attr_name="_pinger_homing_process",
                    status_attr="_pinger_homing_status",
                )

        stop_group.assert_called_once_with(process, 1.0)
        self.assertIsNone(manager._pinger_homing_process)
        self.assertEqual(
            manager._pinger_homing_status,
            "pinger homing failed: configured runtime limit reached",
        )

    def test_normal_stop_cleans_group_after_leader_exits(self) -> None:
        process = _TerminatingLeader(pid=8123)
        with (
            mock.patch(
                "gui.process_termination.terminate_group_or_process",
                return_value=8123,
            ),
            mock.patch(
                "gui.process_termination.terminate_exited_process_group"
            ) as clean_remaining,
        ):
            terminate_process_group(process, timeout_s=4.0)

        clean_remaining.assert_called_once_with(process, timeout_s=1.0)

    def test_exited_leader_terminates_remaining_group(self) -> None:
        process = _FakeProcess(pid=7319)
        with (
            mock.patch("gui.process_termination.os.killpg") as killpg,
            mock.patch("gui.process_termination.time.monotonic", side_effect=[0.0, 0.0, 1.1]),
            mock.patch("gui.process_termination.time.sleep"),
        ):
            terminate_exited_process_group(process, timeout_s=1.0)

        self.assertEqual(
            killpg.call_args_list,
            [
                mock.call(7319, signal.SIGTERM),
                mock.call(7319, 0),
                mock.call(7319, signal.SIGKILL),
            ],
        )

    def test_sim_exit_detaches_and_stops_exact_mavros_generation(self) -> None:
        manager = WebProcessManager.__new__(WebProcessManager)
        manager.node = _FakeNode()
        manager._lock = threading.Lock()
        sim_process = _FakeProcess(pid=9001)
        mavros_process = _FakeProcess(pid=9002)
        manager._sim_process = sim_process
        manager._sim_stack_status = "sim: running"
        manager._ros_pkg_process = mavros_process
        manager._ros_pkg_status = "mavros: running"

        with tempfile.TemporaryDirectory() as directory:
            log_path = Path(directory) / "sim.log"
            log_path.write_text("last runtime line\n", encoding="utf-8")
            with (
                mock.patch("gui.web_process_manager.terminate_exited_process_group") as stop_group,
                mock.patch("gui.web_process_manager.terminate_process_group") as stop_mavros,
            ):
                manager._watch_process(
                    proc=sim_process,
                    log_path=log_path,
                    label="sim stack",
                    attr_name="_sim_process",
                    status_attr="_sim_stack_status",
                )

        self.assertIsNone(manager._sim_process)
        self.assertIsNone(manager._ros_pkg_process)
        self.assertEqual(manager._ros_pkg_status, "mavros: stopped (sim exited)")
        stop_group.assert_called_once_with(sim_process)
        stop_mavros.assert_called_once_with(mavros_process)
        self.assertIn("mavros: stopped (sim exited)", manager.node.events)
        self.assertIn("sim stack: exited", manager.node.events)


if __name__ == "__main__":
    unittest.main()
