#!/usr/bin/env python3
"""Real-socket tests for the standalone A50 TCP JSON emulator."""

from __future__ import annotations

import json
import math
from pathlib import Path
import socket
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch


CURRENT = Path(__file__).resolve().parents[1]
if str(CURRENT) not in sys.path:
    sys.path.insert(0, str(CURRENT))

from bridge.dvl_a50_tcp_emulator import A50TcpJsonEmulator  # noqa: E402


def _beam(beam_id: int, *, valid: bool = True) -> SimpleNamespace:
    return SimpleNamespace(
        beam_id=beam_id,
        measured_radial_velocity_mps=0.01 * (beam_id + 1),
        measured_range_m=0.55 + 0.01 * beam_id,
        rssi_dbm=-30.0 - beam_id,
        nsd_dbm=-90.0 - beam_id,
        velocity_std_mps=0.002 + 0.0001 * beam_id,
        valid=valid,
    )


def _sample(
    *,
    validity_us: int = 1_638_191_471_563_017,
    valid: bool = True,
    one_invalid_beam: bool = False,
) -> SimpleNamespace:
    beams = [
        _beam(index, valid=not one_invalid_beam or index != 3)
        for index in range(4)
    ]
    return SimpleNamespace(
        measured_velocity_frd_mps=(0.12, -0.04, 0.02) if valid else None,
        covariance_frd_mps2=(
            4.0e-6,
            1.0e-7,
            0.0,
            1.0e-7,
            5.0e-6,
            0.0,
            0.0,
            0.0,
            2.0e-6,
        )
        if valid
        else None,
        altitude_estimate_m=0.50 if valid else None,
        beams=tuple(beams),
        velocity_valid=valid,
        fom_mps=0.003 if valid else math.inf,
        time_of_validity_us=validity_us,
        time_of_transmission_us=validity_us + 4_000,
        status=0,
    )


def _delivery(timestamp_us: int = 1_638_191_471_752_336) -> SimpleNamespace:
    return SimpleNamespace(
        position_local_frd_m=(1.25, -0.50, 0.30),
        attitude_rpy_deg=(2.0, -3.0, 127.5),
        position_std_m=0.012,
        sample=SimpleNamespace(time_of_transmission_us=timestamp_us),
        status=0,
    )


def _connect(emulator: A50TcpJsonEmulator) -> socket.socket:
    client = socket.create_connection(emulator.address, timeout=1.0)
    client.settimeout(1.0)
    return client


def _read_json_line(client: socket.socket) -> tuple[dict, bytes]:
    frame = bytearray()
    while not frame.endswith(b"\n"):
        chunk = client.recv(1)
        if not chunk:
            raise ConnectionError("emulator closed before completing a JSON line")
        frame.extend(chunk)
    return json.loads(frame[:-1].decode("utf-8")), bytes(frame)


def _send_json(client: socket.socket, payload: dict) -> None:
    client.sendall(json.dumps(payload, separators=(",", ":")).encode() + b"\n")


def _pending_json(pending) -> dict:
    return json.loads(pending.data[:-1].decode("utf-8"))


def _wait_until(predicate, timeout_s: float = 1.0) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.005)
    return bool(predicate())


def _publish_position_batch(
    emulator: A50TcpJsonEmulator,
    offset: int,
) -> None:
    for index in range(25):
        timestamp_us = (offset * 25 + index + 1) * 1_000_000
        emulator.publish_position(_delivery(timestamp_us))


class A50TcpReportTest(unittest.TestCase):
    def test_velocity_and_position_reports_match_driver_schema(self) -> None:
        emulator = A50TcpJsonEmulator(port=0)
        client: socket.socket | None = None
        with emulator:
            self.assertEqual(emulator.address[0], "127.0.0.1")
            self.assertGreater(emulator.port, 0)
            client = _connect(emulator)
            self.assertTrue(_wait_until(lambda: emulator.client_connected))

            sample = _sample(one_invalid_beam=True)
            self.assertTrue(emulator.publish_velocity(sample, 0.1))
            velocity, raw_velocity = _read_json_line(client)

            self.assertTrue(raw_velocity.endswith(b"\n"))
            self.assertNotIn(b"\n", raw_velocity[:-1])
            self.assertEqual(
                set(velocity),
                {
                    "time",
                    "vx",
                    "vy",
                    "vz",
                    "fom",
                    "covariance",
                    "altitude",
                    "transducers",
                    "velocity_valid",
                    "status",
                    "format",
                    "type",
                    "time_of_validity",
                    "time_of_transmission",
                },
            )
            self.assertEqual(velocity["type"], "velocity")
            self.assertEqual(velocity["format"], "json_v3.3")
            self.assertEqual(velocity["time"], 100.0)
            self.assertEqual(
                (velocity["vx"], velocity["vy"], velocity["vz"]),
                sample.measured_velocity_frd_mps,
            )
            self.assertTrue(velocity["velocity_valid"])
            self.assertEqual(velocity["altitude"], 0.50)
            self.assertTrue(math.isfinite(velocity["fom"]))
            self.assertEqual(len(velocity["covariance"]), 3)
            self.assertTrue(all(len(row) == 3 for row in velocity["covariance"]))
            self.assertEqual(velocity["time_of_validity"], sample.time_of_validity_us)
            self.assertEqual(
                velocity["time_of_transmission"],
                sample.time_of_transmission_us,
            )
            self.assertEqual(len(velocity["transducers"]), 4)
            self.assertEqual(
                set(velocity["transducers"][0]),
                {"id", "velocity", "distance", "rssi", "nsd", "beam_valid"},
            )
            self.assertLess(velocity["transducers"][0]["rssi"], 0.0)
            self.assertLess(velocity["transducers"][0]["nsd"], 0.0)
            self.assertFalse(velocity["transducers"][3]["beam_valid"])
            self.assertEqual(velocity["transducers"][3]["distance"], -1.0)
            self.assertEqual(velocity["transducers"][3]["velocity"], 0.0)

            delivery = _delivery()
            self.assertTrue(emulator.publish_position(delivery))
            position, raw_position = _read_json_line(client)

            self.assertTrue(raw_position.endswith(b"\n"))
            self.assertEqual(
                set(position),
                {
                    "ts",
                    "x",
                    "y",
                    "z",
                    "std",
                    "roll",
                    "pitch",
                    "yaw",
                    "status",
                    "format",
                    "type",
                },
            )
            self.assertEqual(position["type"], "position_local")
            self.assertEqual(position["format"], "json_v3")
            self.assertAlmostEqual(
                position["ts"],
                delivery.sample.time_of_transmission_us * 1.0e-6,
                places=6,
            )
            self.assertEqual(
                (position["x"], position["y"], position["z"]),
                delivery.position_local_frd_m,
            )
            self.assertEqual(
                (position["roll"], position["pitch"], position["yaw"]),
                delivery.attitude_rpy_deg,
            )
            self.assertEqual(position["std"], delivery.position_std_m)
            self.assertEqual(position["status"], 0)

        self.assertFalse(emulator.is_running)
        self.assertFalse(emulator.client_connected)
        self.assertIsNone(emulator.last_error)
        if client is not None:
            client.close()

    def test_invalid_velocity_report_uses_finite_protocol_sentinels(self) -> None:
        with A50TcpJsonEmulator(port=0) as emulator:
            client = _connect(emulator)
            try:
                self.assertTrue(_wait_until(lambda: emulator.client_connected))
                self.assertTrue(emulator.publish_velocity(_sample(valid=False), 0.2))
                report, _ = _read_json_line(client)
            finally:
                client.close()

        self.assertFalse(report["velocity_valid"])
        self.assertEqual(report["altitude"], -1.0)
        self.assertEqual(report["fom"], 2.707)
        self.assertTrue(math.isfinite(report["fom"]))
        expected_variance = 2.707**2
        self.assertEqual(
            report["covariance"],
            [
                [expected_variance, 0.0, 0.0],
                [0.0, expected_variance, 0.0],
                [0.0, 0.0, expected_variance],
            ],
        )

    def test_velocity_time_uses_last_fully_sent_report_across_queue_loss(self) -> None:
        with A50TcpJsonEmulator(port=0, queue_size=1) as emulator:
            emulator.publish_velocity(_sample(validity_us=1_000_000), 0.1)
            first = emulator._dequeue()
            self.assertIsNotNone(first)
            first.sent_bytes = len(first.data)
            emulator._mark_fully_sent(first)

            # The 1.1 s sample is lost in the queue.  The next wire report must
            # measure its interval from the last report that was actually sent.
            emulator.publish_velocity(_sample(validity_us=1_100_000), 0.1)
            emulator.publish_velocity(_sample(validity_us=1_350_000), 0.1)
            self.assertEqual(emulator.dropped_message_count, 1)
            newest = emulator._dequeue()
            self.assertIsNotNone(newest)
            self.assertEqual(_pending_json(newest)["time"], 350.0)

    def test_partial_send_failure_does_not_advance_velocity_time(self) -> None:
        with A50TcpJsonEmulator(port=0) as emulator:
            emulator.publish_velocity(_sample(validity_us=1_000_000), 0.1)
            first = emulator._dequeue()
            self.assertIsNotNone(first)
            first.sent_bytes = len(first.data)
            emulator._mark_fully_sent(first)

            emulator.publish_velocity(_sample(validity_us=1_100_000), 0.1)
            failed = emulator._dequeue()
            self.assertIsNotNone(failed)
            failed.sent_bytes = 1
            # Simulate a connection failure: a partial frame is abandoned and
            # deliberately never passed to _mark_fully_sent().

            emulator.publish_velocity(_sample(validity_us=1_400_000), 0.02)
            after_reconnect = emulator._dequeue()
            self.assertIsNotNone(after_reconnect)
            self.assertEqual(_pending_json(after_reconnect)["time"], 400.0)


class A50TcpCommandTest(unittest.TestCase):
    def test_driver_startup_commands_and_bad_input_do_not_kill_server(self) -> None:
        with A50TcpJsonEmulator(port=0, queue_size=1) as emulator:
            client = _connect(emulator)
            try:
                # This is the exact startup ordering in the 2026 KMU driver.
                client.sendall(
                    b'{"command":"set_config","parameters":'
                    b'{"acoustic_enabled":true}}\n'
                    b'{"command":"get_config"}\n'
                )
                set_response, _ = _read_json_line(client)
                get_response, _ = _read_json_line(client)

                self.assertEqual(set_response["response_to"], "set_config")
                self.assertTrue(set_response["success"])
                self.assertIsNone(set_response["result"])
                self.assertEqual(
                    set(set_response),
                    {
                        "response_to",
                        "success",
                        "error_message",
                        "result",
                        "format",
                        "type",
                    },
                )
                self.assertEqual(set_response["format"], "json_v3.3")
                self.assertEqual(set_response["type"], "response")
                self.assertEqual(get_response["response_to"], "get_config")
                self.assertTrue(get_response["success"])
                self.assertEqual(
                    set(get_response["result"]),
                    {
                        "speed_of_sound",
                        "acoustic_enabled",
                        "dark_mode_enabled",
                        "mounting_rotation_offset",
                        "range_mode",
                        "periodic_cycling_enabled",
                    },
                )
                self.assertTrue(get_response["result"]["acoustic_enabled"])
                self.assertEqual(get_response["format"], "json_v3.3")
                self.assertEqual(get_response["type"], "response")
                self.assertTrue(emulator.acoustic_enabled)

                _send_json(
                    client,
                    {
                        "command": "set_config",
                        "parameters": {"acoustic_enabled": False},
                    },
                )
                disable_response, _ = _read_json_line(client)
                self.assertTrue(disable_response["success"])
                self.assertFalse(emulator.acoustic_enabled)
                self.assertFalse(emulator.publish_velocity(_sample(), 0.1))

                client.sendall(b"{not valid json}\n")
                malformed, _ = _read_json_line(client)
                self.assertFalse(malformed["success"])
                self.assertEqual(malformed["response_to"], "invalid_json")

                _send_json(client, {"command": "not_a_real_command"})
                unknown, _ = _read_json_line(client)
                self.assertFalse(unknown["success"])
                self.assertEqual(unknown["response_to"], "not_a_real_command")

                _send_json(client, {"command": "calibrate_gyro"})
                calibration, _ = _read_json_line(client)
                self.assertTrue(calibration["success"])
                self.assertEqual(calibration["result"], None)

                _send_json(client, {"command": "reset_dead_reckoning"})
                reset, _ = _read_json_line(client)
                self.assertEqual(reset["response_to"], "reset_dead_reckoning")
                self.assertTrue(reset["success"])
                self.assertEqual(reset["result"], None)
                self.assertEqual(reset["format"], "json_v3.3")
                self.assertEqual(reset["type"], "response")
                self.assertEqual(emulator.gyro_calibration_count, 1)
                self.assertEqual(emulator.dead_reckoning_reset_count, 1)
                self.assertTrue(emulator.is_running)
                self.assertIsNone(emulator.last_error)
            finally:
                client.close()

    def test_reset_purges_stale_positions_and_rejects_old_generation(self) -> None:
        with A50TcpJsonEmulator(port=0, queue_size=32) as emulator:
            client = _connect(emulator)
            try:
                self.assertTrue(_wait_until(lambda: emulator.client_connected))
                old_generation = emulator.dead_reckoning_reset_count
                for index in range(20):
                    self.assertTrue(
                        emulator.publish_position(
                            _delivery((index + 1) * 1_000_000),
                            expected_reset_count=old_generation,
                        )
                    )

                _send_json(client, {"command": "reset_dead_reckoning"})
                while True:
                    report, _ = _read_json_line(client)
                    if report.get("type") == "response":
                        break
                    # A frame whose send had already begun may finish, but it
                    # must remain ordered before the reset acknowledgement.
                    self.assertEqual(report["type"], "position_local")
                self.assertEqual(report["response_to"], "reset_dead_reckoning")
                self.assertTrue(report["success"])

                client.settimeout(0.1)
                with self.assertRaises(socket.timeout):
                    _read_json_line(client)

                self.assertFalse(
                    emulator.publish_position(
                        _delivery(21_000_000),
                        expected_reset_count=old_generation,
                    )
                )
                new_generation = emulator.dead_reckoning_reset_count
                self.assertEqual(new_generation, old_generation + 1)
                self.assertTrue(
                    emulator.publish_position(
                        _delivery(22_000_000),
                        expected_reset_count=new_generation,
                    )
                )
                client.settimeout(1.0)
                position, _ = _read_json_line(client)
                self.assertEqual(position["type"], "position_local")
                self.assertEqual(position["ts"], 22.0)
            finally:
                client.close()

    def test_reset_generation_check_is_atomic_with_queued_position_purge(self) -> None:
        with A50TcpJsonEmulator(port=0) as emulator:
            generation = emulator.dead_reckoning_reset_count
            self.assertTrue(
                emulator.publish_position(
                    _delivery(1_000_000),
                    expected_reset_count=generation,
                )
            )
            self.assertEqual(emulator.queued_message_count, 1)

            response = emulator._handle_command_line(
                b'{"command":"reset_dead_reckoning"}'
            )
            self.assertTrue(response["success"])
            self.assertEqual(emulator.queued_message_count, 0)
            self.assertFalse(
                emulator.publish_position(
                    _delivery(2_000_000),
                    expected_reset_count=generation,
                )
            )
            self.assertEqual(emulator.queued_message_count, 0)


class A50TcpLifecycleTest(unittest.TestCase):
    def test_start_failure_releases_bound_listener(self) -> None:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            probe.bind(("127.0.0.1", 0))
            port = int(probe.getsockname()[1])

        emulator = A50TcpJsonEmulator(port=port)
        with patch.object(
            threading.Thread,
            "start",
            side_effect=RuntimeError("injected thread start failure"),
        ):
            with self.assertRaisesRegex(RuntimeError, "injected"):
                emulator.start()

        self.assertFalse(emulator.is_running)
        self.assertIsNone(emulator._thread)
        self.assertIsNone(emulator._server_socket)
        with self.assertRaises(RuntimeError):
            _ = emulator.address
        emulator.stop()
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as rebound:
            rebound.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            rebound.bind(("127.0.0.1", port))

    def test_disconnect_then_reconnect(self) -> None:
        with A50TcpJsonEmulator(port=0) as emulator:
            first = _connect(emulator)
            self.assertTrue(_wait_until(lambda: emulator.client_connected))

            rejected = _connect(emulator)
            try:
                try:
                    self.assertEqual(rejected.recv(1), b"")
                except (ConnectionResetError, OSError):
                    pass
            finally:
                rejected.close()

            emulator.publish_position(_delivery(1_000_000))
            first_report, _ = _read_json_line(first)
            self.assertEqual(first_report["ts"], 1.0)
            emulator.publish_velocity(_sample(validity_us=1_000_000), 0.05)
            first_velocity, _ = _read_json_line(first)
            self.assertEqual(first_velocity["time"], 50.0)
            first.shutdown(socket.SHUT_RDWR)
            first.close()
            self.assertTrue(_wait_until(lambda: not emulator.client_connected))

            second = _connect(emulator)
            try:
                self.assertTrue(_wait_until(lambda: emulator.client_connected))
                emulator.publish_velocity(_sample(validity_us=2_000_000), 0.05)
                second_report, _ = _read_json_line(second)
                self.assertEqual(second_report["type"], "velocity")
                self.assertEqual(second_report["time_of_validity"], 2_000_000)
                self.assertEqual(second_report["time"], 1_000.0)
                self.assertTrue(emulator.is_running)
                self.assertIsNone(emulator.last_error)
            finally:
                second.close()

    def test_bounded_queue_keeps_newest_reports_and_stop_closes_client(self) -> None:
        emulator = A50TcpJsonEmulator(port=0, queue_size=2).start()
        client: socket.socket | None = None
        try:
            emulator.publish_position(_delivery(1_000_000))
            emulator.publish_position(_delivery(2_000_000))
            emulator.publish_position(_delivery(3_000_000))
            self.assertEqual(emulator.queued_message_count, 2)
            self.assertEqual(emulator.dropped_message_count, 1)

            client = _connect(emulator)
            self.assertTrue(_wait_until(lambda: emulator.client_connected))
            self.assertEqual(emulator.queued_message_count, 0)
            emulator.publish_position(_delivery(4_000_000))
            report, _ = _read_json_line(client)
            self.assertEqual(report["ts"], 4.0)

            emulator.stop()
            self.assertFalse(emulator.is_running)
            self.assertFalse(emulator.client_connected)
            self.assertIsNone(emulator.last_error)
            try:
                self.assertEqual(client.recv(1), b"")
            except (ConnectionResetError, OSError):
                pass
        finally:
            if emulator.is_running:
                emulator.stop()
            if client is not None:
                client.close()

    def test_concurrent_publishers_keep_queue_bounded(self) -> None:
        with A50TcpJsonEmulator(port=0, queue_size=8) as emulator:
            publishers = [
                threading.Thread(
                    target=_publish_position_batch,
                    args=(emulator, offset),
                )
                for offset in range(4)
            ]
            for publisher in publishers:
                publisher.start()
            for publisher in publishers:
                publisher.join(timeout=1.0)

            self.assertTrue(all(not publisher.is_alive() for publisher in publishers))
            self.assertEqual(emulator.queued_message_count, 8)
            self.assertEqual(emulator.dropped_message_count, 92)
            self.assertIsNone(emulator.last_error)


if __name__ == "__main__":
    unittest.main(verbosity=2)
