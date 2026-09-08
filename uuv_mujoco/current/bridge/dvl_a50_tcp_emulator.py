"""Water Linked A50-compatible newline-delimited TCP JSON emulator.

Protocol reference:
https://docs.waterlinked.com/dvl/dvl-json-protocol/

The 2026 KMU launch configures its driver to send ``set_config`` followed by
``get_config`` when TCP connects.  The driver consumes one JSON object per
newline and directly indexes the fields emitted here:
https://github.com/2026-kmu-underwater-robot/auv_dvl_a50/blob/
21574163bb822f8ed1b98763cd8805d777556b8e/src/dvl-sensor.cpp

The public API deliberately accepts duck-typed sensor samples and deliveries.
It has no ROS, MuJoCo, or A50 sensor-model dependency.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Mapping
from dataclasses import dataclass
import json
import math
import select
import socket
import threading
from typing import Any


JSON_FORMAT = "json_v3.3"
# The KMU DVLDR compatibility path identifies local-position reports with the
# legacy family label used by its existing ROS message builder.
POSITION_JSON_FORMAT = "json_v3"
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 16171
# Published invalid A50/A125 examples use a finite 2.707 m/s FOM sentinel:
# https://docs.waterlinked.com/dvl/dvl-serial-protocol/
INVALID_FOM_MPS = 2.707
_MAX_COMMAND_LINE_BYTES = 1024 * 1024


@dataclass(frozen=True)
class _QueuedTelemetry:
    """A telemetry payload plus state needed when it reaches the wire."""

    payload: dict[str, Any]
    report_type: str
    validity_us: int | None = None
    reset_count: int | None = None


@dataclass
class _PendingFrame:
    """One encoded frame being written to a client socket."""

    data: bytes
    report_type: str
    validity_us: int | None = None
    reset_count: int | None = None
    sent_bytes: int = 0

    @property
    def remaining(self) -> bytes:
        return self.data[self.sent_bytes :]

    @property
    def fully_sent(self) -> bool:
        return self.sent_bytes == len(self.data)


def _finite_float(value: Any, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _finite_or(value: Any, fallback: float) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return float(fallback)
    return result if math.isfinite(result) else float(fallback)


def _vector3(value: Any, name: str) -> tuple[float, float, float]:
    if isinstance(value, (str, bytes)):
        raise ValueError(f"{name} must contain 3 numeric values")
    try:
        values = tuple(value)
    except TypeError as exc:
        raise ValueError(f"{name} must contain 3 numeric values") from exc
    if len(values) != 3:
        raise ValueError(f"{name} must contain 3 numeric values")
    return tuple(
        _finite_float(component, f"{name}[{index}]")
        for index, component in enumerate(values)
    )


def _invalid_covariance3() -> list[list[float]]:
    variance = INVALID_FOM_MPS**2
    return [
        [variance if row == column else 0.0 for column in range(3)]
        for row in range(3)
    ]


def _covariance3(value: Any) -> list[list[float]]:
    if value is None:
        return _invalid_covariance3()
    if isinstance(value, (str, bytes)):
        raise ValueError("covariance_frd_mps2 must be a flat or 3x3 sequence")
    try:
        outer = tuple(value)
    except TypeError as exc:
        raise ValueError(
            "covariance_frd_mps2 must be a flat or 3x3 sequence"
        ) from exc

    if len(outer) == 9:
        flat = tuple(
            _finite_float(component, f"covariance_frd_mps2[{index}]")
            for index, component in enumerate(outer)
        )
        return [list(flat[index : index + 3]) for index in range(0, 9, 3)]

    if len(outer) == 3:
        rows = [
            _vector3(row, f"covariance_frd_mps2[{index}]")
            for index, row in enumerate(outer)
        ]
        return [list(row) for row in rows]
    raise ValueError("covariance_frd_mps2 must be a flat or 3x3 sequence")


def _response(
    command: str,
    *,
    success: bool,
    error_message: str = "",
    result: Any = None,
) -> dict[str, Any]:
    return {
        "response_to": command,
        "success": bool(success),
        "error_message": str(error_message),
        "result": result,
        "format": JSON_FORMAT,
        "type": "response",
    }


class A50TcpJsonEmulator:
    """A single-client Water Linked A50 TCP JSON server.

    Reports are buffered in a bounded, thread-safe drop-oldest queue.  Command
    responses use a separate bounded priority queue and are never displaced by
    telemetry.  A disconnected client may reconnect without restarting the
    emulator; reports from an earlier connection are discarded.
    """

    def __init__(
        self,
        host: str = DEFAULT_HOST,
        port: int = DEFAULT_PORT,
        *,
        queue_size: int = 128,
        acoustic_enabled: bool = True,
    ) -> None:
        if not isinstance(host, str) or not host:
            raise ValueError("host must be a non-empty string")
        if not 0 <= int(port) <= 65_535:
            raise ValueError("port must be in [0, 65535]")
        if int(queue_size) <= 0:
            raise ValueError("queue_size must be positive")

        self.host = host
        self.requested_port = int(port)
        self.queue_size = int(queue_size)

        self._state_lock = threading.RLock()
        self._config: dict[str, Any] = {
            "speed_of_sound": 1475.0,
            "acoustic_enabled": bool(acoustic_enabled),
            "dark_mode_enabled": False,
            "mounting_rotation_offset": 0.0,
            "range_mode": "auto",
            "periodic_cycling_enabled": False,
        }
        self._dead_reckoning_reset_count = 0
        self._gyro_calibration_count = 0

        self._queue_lock = threading.Lock()
        self._outbound: deque[_QueuedTelemetry] = deque()
        self._responses: deque[bytes] = deque()
        self._dropped_message_count = 0
        self._last_sent_velocity_validity_us: int | None = None

        self._lifecycle_lock = threading.Lock()
        self._socket_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._server_socket: socket.socket | None = None
        self._client_socket: socket.socket | None = None
        self._bound_address: tuple[str, int] | None = None
        self._last_error: str | None = None

    @property
    def address(self) -> tuple[str, int]:
        """Return the bound IPv4 address, including the selected ephemeral port."""

        with self._socket_lock:
            if self._bound_address is None:
                raise RuntimeError("emulator has not been started")
            return self._bound_address

    @property
    def port(self) -> int:
        """Return the bound TCP port."""

        return self.address[1]

    @property
    def is_running(self) -> bool:
        """Return whether the background network thread is alive."""

        with self._lifecycle_lock:
            return self._thread is not None and self._thread.is_alive()

    @property
    def client_connected(self) -> bool:
        """Return whether one active TCP client is connected."""

        with self._socket_lock:
            return self._client_socket is not None

    @property
    def acoustic_enabled(self) -> bool:
        """Return the emulated acoustic-enabled configuration state."""

        with self._state_lock:
            return bool(self._config["acoustic_enabled"])

    @property
    def config_snapshot(self) -> dict[str, Any]:
        """Return an isolated copy of the emulated device configuration."""

        with self._state_lock:
            return dict(self._config)

    @property
    def queued_message_count(self) -> int:
        """Return the number of complete frames waiting to be sent."""

        with self._queue_lock:
            return len(self._responses) + len(self._outbound)

    @property
    def dropped_message_count(self) -> int:
        """Return the number of frames discarded by queue overflow."""

        with self._queue_lock:
            return self._dropped_message_count

    @property
    def last_error(self) -> str | None:
        """Return an unexpected background-thread error, if one occurred."""

        with self._state_lock:
            return self._last_error

    @property
    def dead_reckoning_reset_count(self) -> int:
        """Return the number of accepted reset commands."""

        with self._state_lock:
            return self._dead_reckoning_reset_count

    @property
    def gyro_calibration_count(self) -> int:
        """Return the number of accepted calibration commands."""

        with self._state_lock:
            return self._gyro_calibration_count

    def reset_dead_reckoning(self) -> int:
        """Advance the DR generation and purge unsent older position reports."""

        with self._state_lock:
            self._dead_reckoning_reset_count += 1
            reset_count = self._dead_reckoning_reset_count
            with self._queue_lock:
                self._outbound = deque(
                    queued
                    for queued in self._outbound
                    if not (
                        queued.report_type == "position_local"
                        and queued.reset_count is not None
                        and queued.reset_count < reset_count
                    )
                )
        return reset_count

    def start(self) -> A50TcpJsonEmulator:
        """Bind the listening socket and start the background thread."""

        with self._lifecycle_lock:
            if self._thread is not None and self._thread.is_alive():
                return self

            server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                server.bind((self.host, self.requested_port))
                server.listen(1)
                server.setblocking(False)
            except Exception:
                server.close()
                raise

            with self._queue_lock:
                self._outbound.clear()
                self._responses.clear()
                self._dropped_message_count = 0
                self._last_sent_velocity_validity_us = None
            with self._state_lock:
                self._last_error = None
            with self._socket_lock:
                self._server_socket = server
                bound_host, bound_port = server.getsockname()[:2]
                self._bound_address = (str(bound_host), int(bound_port))
                self._client_socket = None

            self._stop_event.clear()
            thread = threading.Thread(
                target=self._run,
                name="dvl-a50-tcp-emulator",
                daemon=True,
            )
            self._thread = thread
            try:
                thread.start()
            except Exception:
                self._stop_event.set()
                try:
                    server.close()
                except OSError:
                    pass
                if thread.is_alive():
                    thread.join(timeout=2.0)
                with self._socket_lock:
                    if self._server_socket is server:
                        self._server_socket = None
                    self._client_socket = None
                    self._bound_address = None
                self._thread = None
                raise
        return self

    def stop(self, timeout_s: float = 2.0) -> None:
        """Stop the network thread and close the client and listener sockets."""

        timeout = _finite_float(timeout_s, "timeout_s")
        if timeout < 0.0:
            raise ValueError("timeout_s must be non-negative")
        with self._lifecycle_lock:
            thread = self._thread
        if thread is None:
            return

        self._stop_event.set()
        thread.join(timeout)
        if thread.is_alive():
            self._force_close_sockets()
            thread.join(timeout)
        if thread.is_alive():
            raise RuntimeError("A50 TCP emulator thread did not stop")
        with self._lifecycle_lock:
            if self._thread is thread:
                self._thread = None

    def __enter__(self) -> A50TcpJsonEmulator:
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.stop()

    def publish_velocity(self, sample: Any, report_period_s: float) -> bool:
        """Queue one ``velocity`` report from a duck-typed A50 sample.

        Returns ``False`` without queueing when acoustics are disabled.
        """

        if not self.acoustic_enabled:
            return False
        period_s = _finite_float(report_period_s, "report_period_s")
        if period_s < 0.0:
            raise ValueError("report_period_s must be non-negative")
        report = self._build_velocity_report(sample, period_s)
        self._enqueue_json(report)
        return True

    def publish_position(
        self,
        delivery: Any,
        expected_reset_count: int | None = None,
    ) -> bool:
        """Queue one ``position_local`` report from a duck-typed delivery.

        Args:
            delivery: Duck-typed dead-reckoning delivery to serialize.
            expected_reset_count: Reset generation used to produce ``delivery``.
                If supplied and a reset raced with its production, the stale
                report is rejected atomically and ``False`` is returned.
        """

        report = self._build_position_report(delivery)
        if expected_reset_count is not None:
            expected_reset_count = int(expected_reset_count)
            if expected_reset_count < 0:
                raise ValueError("expected_reset_count must be non-negative")

        # The generation check and enqueue must be atomic with the reset
        # command's generation increment and stale-position purge.
        with self._state_lock:
            current_reset_count = self._dead_reckoning_reset_count
            if (
                expected_reset_count is not None
                and expected_reset_count != current_reset_count
            ):
                return False
            self._enqueue_json(
                report,
                reset_count=current_reset_count,
            )
        return True

    def _build_velocity_report(
        self,
        sample: Any,
        report_period_s: float,
    ) -> dict[str, Any]:
        measured = getattr(sample, "measured_velocity_frd_mps", None)
        declared_valid = bool(getattr(sample, "velocity_valid", False))
        try:
            velocity = _vector3(measured, "measured_velocity_frd_mps")
        except (TypeError, ValueError):
            velocity = (0.0, 0.0, 0.0)
            declared_valid = False

        covariance = (
            _covariance3(getattr(sample, "covariance_frd_mps2", None))
            if declared_valid
            else _invalid_covariance3()
        )
        altitude = _finite_or(
            getattr(sample, "altitude_estimate_m", None),
            -1.0,
        )
        if not declared_valid or altitude < 0.0:
            altitude = -1.0

        beams = tuple(getattr(sample, "beams"))
        if len(beams) != 4:
            raise ValueError("sample.beams must contain exactly 4 beams")
        transducers = [
            self._build_transducer(beam, index)
            for index, beam in enumerate(beams)
        ]

        fom = (
            max(
                0.0,
                _finite_or(
                    getattr(sample, "fom_mps", INVALID_FOM_MPS),
                    INVALID_FOM_MPS,
                ),
            )
            if declared_valid
            else INVALID_FOM_MPS
        )
        validity_us = int(getattr(sample, "time_of_validity_us"))
        transmission_us = int(getattr(sample, "time_of_transmission_us"))
        if validity_us < 0 or transmission_us < validity_us:
            raise ValueError("sample timestamps are invalid")

        return {
            "time": report_period_s * 1000.0,
            "vx": velocity[0],
            "vy": velocity[1],
            "vz": velocity[2],
            "fom": fom,
            "covariance": covariance,
            "altitude": altitude,
            "transducers": transducers,
            "velocity_valid": declared_valid,
            "status": int(getattr(sample, "status", 0)),
            "format": JSON_FORMAT,
            "type": "velocity",
            "time_of_validity": validity_us,
            "time_of_transmission": transmission_us,
        }

    def _build_transducer(self, beam: Any, index: int) -> dict[str, Any]:
        beam_valid = bool(
            getattr(beam, "valid", getattr(beam, "beam_valid", False))
        )
        velocity = _finite_or(
            getattr(
                beam,
                "measured_radial_velocity_mps",
                getattr(beam, "velocity", None),
            ),
            0.0,
        )
        distance = _finite_or(
            getattr(
                beam,
                "measured_range_m",
                getattr(beam, "distance", None),
            ),
            -1.0,
        )
        if not beam_valid or distance < 0.0:
            beam_valid = False
            velocity = 0.0
            distance = -1.0
        return {
            "id": int(getattr(beam, "beam_id", getattr(beam, "id", index))),
            "velocity": velocity,
            "distance": distance,
            "rssi": _finite_or(
                getattr(beam, "rssi_dbm", getattr(beam, "rssi", -120.0)),
                -120.0,
            ),
            "nsd": _finite_or(
                getattr(beam, "nsd_dbm", getattr(beam, "nsd", -94.0)),
                -94.0,
            ),
            "beam_valid": beam_valid,
        }

    def _build_position_report(self, delivery: Any) -> dict[str, Any]:
        position_value = getattr(delivery, "position_local_frd_m", None)
        if position_value is None:
            position_value = (
                getattr(delivery, "x"),
                getattr(delivery, "y"),
                getattr(delivery, "z"),
            )
        position = _vector3(position_value, "position_local_frd_m")

        attitude_value = getattr(delivery, "attitude_rpy_deg", None)
        if attitude_value is None:
            attitude_value = (
                getattr(delivery, "roll"),
                getattr(delivery, "pitch"),
                getattr(delivery, "yaw"),
            )
        attitude = _vector3(attitude_value, "attitude_rpy_deg")
        position_std = max(
            0.0,
            _finite_float(
                getattr(delivery, "position_std_m", getattr(delivery, "std", 0.0)),
                "position_std_m",
            ),
        )
        return {
            "ts": self._position_timestamp_s(delivery),
            "x": position[0],
            "y": position[1],
            "z": position[2],
            "std": position_std,
            "roll": attitude[0],
            "pitch": attitude[1],
            "yaw": attitude[2],
            "status": int(getattr(delivery, "status", 0)),
            "format": POSITION_JSON_FORMAT,
            "type": "position_local",
        }

    def _position_timestamp_s(self, delivery: Any) -> float:
        for name in ("ts", "report_time_s", "timestamp_s"):
            if hasattr(delivery, name):
                return _finite_float(getattr(delivery, name), name)
        if hasattr(delivery, "time_of_transmission_us"):
            return _finite_float(
                getattr(delivery, "time_of_transmission_us"),
                "time_of_transmission_us",
            ) * 1.0e-6
        sample = getattr(delivery, "sample", None)
        if sample is not None and hasattr(sample, "time_of_transmission_us"):
            return _finite_float(
                getattr(sample, "time_of_transmission_us"),
                "sample.time_of_transmission_us",
            ) * 1.0e-6
        raise ValueError("delivery must provide a report timestamp")

    def _enqueue_json(
        self,
        payload: Mapping[str, Any],
        *,
        reset_count: int | None = None,
    ) -> None:
        if not self.is_running:
            raise RuntimeError("emulator is not running")
        payload_copy = dict(payload)
        report_type = str(payload_copy.get("type", ""))
        validity_us = (
            int(payload_copy["time_of_validity"])
            if report_type == "velocity"
            else None
        )
        queued = _QueuedTelemetry(
            payload=payload_copy,
            report_type=report_type,
            validity_us=validity_us,
            reset_count=reset_count,
        )

        with self._queue_lock:
            if len(self._outbound) >= self.queue_size:
                self._outbound.popleft()
                self._dropped_message_count += 1
            self._outbound.append(queued)

    def _enqueue_response(self, payload: Mapping[str, Any]) -> None:
        frame = self._encode_json(payload)
        with self._queue_lock:
            if len(self._responses) >= self.queue_size:
                raise RuntimeError("command response queue is full")
            self._responses.append(frame)

    def _encode_json(self, payload: Mapping[str, Any]) -> bytes:
        return (
            json.dumps(
                dict(payload),
                allow_nan=False,
                ensure_ascii=False,
                separators=(",", ":"),
            ).encode("utf-8")
            + b"\n"
        )

    def _dequeue(self) -> _PendingFrame | None:
        with self._queue_lock:
            if self._responses:
                return _PendingFrame(
                    data=self._responses.popleft(),
                    report_type="response",
                )
            if not self._outbound:
                return None
            queued = self._outbound.popleft()
            payload = dict(queued.payload)
            if queued.report_type == "velocity":
                previous_us = self._last_sent_velocity_validity_us
                if (
                    previous_us is not None
                    and queued.validity_us is not None
                    and queued.validity_us >= previous_us
                ):
                    payload["time"] = (queued.validity_us - previous_us) * 1.0e-3
            return _PendingFrame(
                data=self._encode_json(payload),
                report_type=queued.report_type,
                validity_us=queued.validity_us,
                reset_count=queued.reset_count,
            )

    def _mark_fully_sent(self, pending: _PendingFrame) -> None:
        """Record wire history only after every byte was accepted by TCP."""

        if not pending.fully_sent:
            raise ValueError("cannot mark a partial frame as fully sent")
        if pending.report_type != "velocity" or pending.validity_us is None:
            return
        with self._queue_lock:
            self._last_sent_velocity_validity_us = pending.validity_us

    def _discard_unsent_stale_position(
        self,
        pending: _PendingFrame | None,
    ) -> _PendingFrame | None:
        """Drop a pre-reset position only if no bytes reached the socket."""

        if (
            pending is None
            or pending.report_type != "position_local"
            or pending.sent_bytes != 0
            or pending.reset_count is None
        ):
            return pending
        with self._state_lock:
            if pending.reset_count < self._dead_reckoning_reset_count:
                return None
        return pending

    def _has_outbound(self) -> bool:
        with self._queue_lock:
            return bool(self._responses or self._outbound)

    def _response_queue_has_capacity(self) -> bool:
        with self._queue_lock:
            return len(self._responses) < self.queue_size

    def _discard_stale_frames(self) -> None:
        with self._queue_lock:
            self._responses.clear()
            self._outbound.clear()

    def _run(self) -> None:
        with self._socket_lock:
            server = self._server_socket
        if server is None:
            return

        client: socket.socket | None = None
        receive_buffer = bytearray()
        pending_frame: _PendingFrame | None = None
        try:
            while not self._stop_event.is_set():
                if client is not None and receive_buffer:
                    self._consume_command_lines(receive_buffer)
                    pending_frame = self._discard_unsent_stale_position(
                        pending_frame
                    )
                readers: list[socket.socket] = [server]
                if client is not None:
                    readers.append(client)
                writers = (
                    [client]
                    if client is not None
                    and (pending_frame is not None or self._has_outbound())
                    else []
                )
                try:
                    readable, writable, _ = select.select(
                        readers,
                        writers,
                        [],
                        0.05,
                    )
                except (OSError, ValueError):
                    if self._stop_event.is_set():
                        break
                    raise

                if server in readable:
                    previous_client = client
                    client = self._accept_clients(server, client)
                    if client is not previous_client:
                        receive_buffer.clear()

                if client is not None and client in readable:
                    try:
                        chunk = client.recv(65_536)
                    except BlockingIOError:
                        chunk = None
                    except OSError:
                        chunk = b""
                    if chunk == b"":
                        self._close_client(client)
                        client = None
                        receive_buffer.clear()
                        pending_frame = None
                    elif chunk:
                        receive_buffer.extend(chunk)
                        if len(receive_buffer) > _MAX_COMMAND_LINE_BYTES:
                            receive_buffer.clear()
                            if self._response_queue_has_capacity():
                                self._enqueue_response(
                                    _response(
                                        "invalid_json",
                                        success=False,
                                        error_message="command line exceeds 1 MiB",
                                    )
                                )
                        self._consume_command_lines(receive_buffer)
                        pending_frame = self._discard_unsent_stale_position(
                            pending_frame
                        )

                if client is not None and client in writable:
                    if pending_frame is None:
                        pending_frame = self._dequeue()
                    pending_frame = self._discard_unsent_stale_position(
                        pending_frame
                    )
                    if pending_frame is not None:
                        try:
                            sent = client.send(pending_frame.remaining)
                        except BlockingIOError:
                            sent = 0
                        except OSError:
                            sent = -1
                        if sent < 0:
                            self._close_client(client)
                            client = None
                            receive_buffer.clear()
                            pending_frame = None
                        elif sent > 0:
                            pending_frame.sent_bytes += sent
                            if pending_frame.fully_sent:
                                self._mark_fully_sent(pending_frame)
                                pending_frame = None
        except Exception as exc:
            with self._state_lock:
                self._last_error = f"{type(exc).__name__}: {exc}"
        finally:
            if client is not None:
                self._close_client(client)
            try:
                server.close()
            except OSError:
                pass
            with self._socket_lock:
                if self._server_socket is server:
                    self._server_socket = None

    def _accept_clients(
        self,
        server: socket.socket,
        client: socket.socket | None,
    ) -> socket.socket | None:
        while True:
            try:
                candidate, _ = server.accept()
            except BlockingIOError:
                return client
            except OSError:
                return client
            candidate.setblocking(False)
            candidate.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            if client is not None:
                candidate.close()
                continue
            # A physical A50 does not replay telemetry accumulated while its
            # TCP peer was absent.  Responses from an earlier peer are stale too.
            self._discard_stale_frames()
            client = candidate
            with self._socket_lock:
                self._client_socket = client

    def _consume_command_lines(self, receive_buffer: bytearray) -> None:
        while self._response_queue_has_capacity():
            newline = receive_buffer.find(b"\n")
            if newline < 0:
                return
            raw_line = bytes(receive_buffer[:newline]).rstrip(b"\r")
            del receive_buffer[: newline + 1]
            if not raw_line:
                continue
            response = self._handle_command_line(raw_line)
            self._enqueue_response(response)

    def _handle_command_line(self, raw_line: bytes) -> dict[str, Any]:
        try:
            message = json.loads(raw_line.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            return _response(
                "invalid_json",
                success=False,
                error_message=f"invalid JSON: {exc}",
            )
        if not isinstance(message, Mapping):
            return _response(
                "unknown",
                success=False,
                error_message="command must be a JSON object",
            )
        command = message.get("command")
        if not isinstance(command, str) or not command:
            return _response(
                "unknown",
                success=False,
                error_message="missing string command",
            )
        if command == "get_config":
            return _response(
                command,
                success=True,
                result=self.config_snapshot,
            )
        if command == "set_config":
            return self._handle_set_config(message)
        if command == "reset_dead_reckoning":
            # The runtime adapter owns the integrated position state; this
            # counter is its thread-safe reset signal, while the wire contract
            # only requires the immediate response envelope.
            self.reset_dead_reckoning()
            return _response(command, success=True)
        if command == "calibrate_gyro":
            with self._state_lock:
                self._gyro_calibration_count += 1
            return _response(command, success=True)
        return _response(
            command,
            success=False,
            error_message=f"unknown command: {command}",
        )

    def _handle_set_config(self, message: Mapping[str, Any]) -> dict[str, Any]:
        parameters = message.get("parameters")
        if not isinstance(parameters, Mapping):
            return _response(
                "set_config",
                success=False,
                error_message="parameters must be a JSON object",
            )
        try:
            updates = self._validate_config_updates(parameters)
        except ValueError as exc:
            return _response(
                "set_config",
                success=False,
                error_message=str(exc),
            )
        with self._state_lock:
            self._config.update(updates)
        return _response("set_config", success=True)

    def _validate_config_updates(
        self,
        parameters: Mapping[str, Any],
    ) -> dict[str, Any]:
        allowed = set(self._config)
        unknown = sorted(str(name) for name in set(parameters) - allowed)
        if unknown:
            raise ValueError(f"unsupported configuration parameter: {unknown[0]}")

        updates: dict[str, Any] = {}
        for name, value in parameters.items():
            if name in {
                "acoustic_enabled",
                "dark_mode_enabled",
                "periodic_cycling_enabled",
            }:
                if not isinstance(value, bool):
                    raise ValueError(f"{name} must be boolean")
                updates[name] = value
            elif name == "speed_of_sound":
                number = _finite_float(value, name)
                if not 1000.0 <= number <= 2000.0:
                    raise ValueError("speed_of_sound must be in [1000, 2000] m/s")
                updates[name] = number
            elif name == "mounting_rotation_offset":
                number = _finite_float(value, name)
                if not 0.0 <= number <= 360.0:
                    raise ValueError(
                        "mounting_rotation_offset must be in [0, 360] degrees"
                    )
                updates[name] = number
            elif name == "range_mode":
                if not isinstance(value, str) or not value:
                    raise ValueError("range_mode must be a non-empty string")
                if value == "wt":
                    raise ValueError("water tracking is not emulated")
                updates[name] = value
        return updates

    def _close_client(self, client: socket.socket) -> None:
        try:
            client.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            client.close()
        except OSError:
            pass
        with self._socket_lock:
            if self._client_socket is client:
                self._client_socket = None

    def _force_close_sockets(self) -> None:
        with self._socket_lock:
            sockets = (self._client_socket, self._server_socket)
        for active_socket in sockets:
            if active_socket is None:
                continue
            try:
                active_socket.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                active_socket.close()
            except OSError:
                pass


__all__ = [
    "A50TcpJsonEmulator",
    "DEFAULT_HOST",
    "DEFAULT_PORT",
    "INVALID_FOM_MPS",
    "JSON_FORMAT",
    "POSITION_JSON_FORMAT",
]
