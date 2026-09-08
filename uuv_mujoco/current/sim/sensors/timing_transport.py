"""Deterministic sensor capture timing and transport simulation.

The runtime deliberately has no ROS dependency.  A sensor implementation can
use :meth:`SensorTimingTransportRuntime.schedule_until` to sample its physical
quantity at each capture instant, then submit the value for latency, dropout,
and bounded-queue handling.  :meth:`SensorTimingTransportRuntime.advance` is a
convenience wrapper for the common capture-and-submit path.

All ``*_time_s`` values except ``device_time_s`` use the caller's reference
clock, normally MuJoCo simulation time.  ``device_time_s`` models the timestamp
written by the sensor's own clock at capture time.
"""

from __future__ import annotations

import math
import random
from collections import OrderedDict
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, Generic, TypeVar


PayloadT = TypeVar("PayloadT")


def _finite(value: float, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite, got {value!r}")
    return result


@dataclass(frozen=True, slots=True)
class CaptureScheduleConfig:
    """Fixed-rate capture schedule expressed in reference-clock seconds."""

    rate_hz: float
    phase_s: float = 0.0
    start_time_s: float = 0.0
    epsilon_s: float = 1.0e-9

    def __post_init__(self) -> None:
        rate_hz = _finite(self.rate_hz, "rate_hz")
        phase_s = _finite(self.phase_s, "phase_s")
        _finite(self.start_time_s, "start_time_s")
        epsilon_s = _finite(self.epsilon_s, "epsilon_s")
        if rate_hz <= 0.0:
            raise ValueError("rate_hz must be greater than zero")
        period_s = 1.0 / rate_hz
        if phase_s < 0.0 or phase_s >= period_s:
            raise ValueError(
                f"phase_s must be in [0, period), got phase={phase_s} period={period_s}"
            )
        if epsilon_s < 0.0:
            raise ValueError("epsilon_s must be non-negative")

    @property
    def period_s(self) -> float:
        """Capture period [s]."""

        return 1.0 / float(self.rate_hz)

    @property
    def first_capture_time_s(self) -> float:
        """First capture timestamp on the reference clock [s]."""

        return float(self.start_time_s) + float(self.phase_s)


@dataclass(frozen=True, slots=True)
class DeviceClockConfig:
    """Affine sensor clock model relative to a reference clock."""

    offset_s: float = 0.0
    drift_ppm: float = 0.0
    reference_time_s: float = 0.0

    def __post_init__(self) -> None:
        _finite(self.offset_s, "offset_s")
        drift_ppm = _finite(self.drift_ppm, "drift_ppm")
        _finite(self.reference_time_s, "reference_time_s")
        if drift_ppm <= -1_000_000.0:
            raise ValueError("drift_ppm must keep the device clock strictly increasing")

    def timestamp(self, reference_time_s: float) -> float:
        """Return the device timestamp corresponding to a reference time [s]."""

        reference_time_s = _finite(reference_time_s, "reference_time_s")
        elapsed_s = reference_time_s - float(self.reference_time_s)
        drift_fraction = float(self.drift_ppm) * 1.0e-6
        return reference_time_s + float(self.offset_s) + elapsed_s * drift_fraction


@dataclass(frozen=True, slots=True)
class LatencyConfig:
    """Gaussian-jitter latency model, clamped to configured bounds."""

    mean_s: float = 0.0
    jitter_std_s: float = 0.0
    min_s: float = 0.0
    max_s: float | None = None

    def __post_init__(self) -> None:
        mean_s = _finite(self.mean_s, "mean_s")
        jitter_std_s = _finite(self.jitter_std_s, "jitter_std_s")
        min_s = _finite(self.min_s, "min_s")
        if min_s < 0.0:
            raise ValueError("min_s must be non-negative")
        if mean_s < min_s:
            raise ValueError("mean_s must be greater than or equal to min_s")
        if jitter_std_s < 0.0:
            raise ValueError("jitter_std_s must be non-negative")
        if self.max_s is not None:
            max_s = _finite(self.max_s, "max_s")
            if max_s < mean_s:
                raise ValueError("max_s must be greater than or equal to mean_s")

    def sample(self, rng: random.Random) -> float:
        """Draw one bounded latency sample [s]."""

        value = float(self.mean_s)
        if self.jitter_std_s > 0.0:
            value += rng.gauss(0.0, float(self.jitter_std_s))
        value = max(float(self.min_s), value)
        if self.max_s is not None:
            value = min(float(self.max_s), value)
        return value


class OverflowPolicy(str, Enum):
    """Policy applied when the bounded pending-packet queue is full."""

    DROP_OLDEST = "drop_oldest"
    DROP_NEWEST = "drop_newest"


class PacketDropReason(str, Enum):
    """Reason an incoming packet was rejected."""

    PROBABILISTIC = "probabilistic"
    QUEUE_OVERFLOW = "queue_overflow"


@dataclass(frozen=True, slots=True)
class SensorTransportConfig:
    """Processing, link, dropout, and queue configuration."""

    processing_latency: LatencyConfig = field(default_factory=LatencyConfig)
    transport_latency: LatencyConfig = field(default_factory=LatencyConfig)
    dropout_probability: float = 0.0
    queue_capacity: int = 32
    overflow_policy: OverflowPolicy = OverflowPolicy.DROP_OLDEST

    def __post_init__(self) -> None:
        if not isinstance(self.processing_latency, LatencyConfig):
            raise TypeError("processing_latency must be a LatencyConfig")
        if not isinstance(self.transport_latency, LatencyConfig):
            raise TypeError("transport_latency must be a LatencyConfig")
        probability = _finite(self.dropout_probability, "dropout_probability")
        if probability < 0.0 or probability > 1.0:
            raise ValueError("dropout_probability must be in [0, 1]")
        if isinstance(self.queue_capacity, bool) or not isinstance(self.queue_capacity, int):
            raise TypeError("queue_capacity must be an integer")
        if self.queue_capacity < 1:
            raise ValueError("queue_capacity must be at least one")
        if not isinstance(self.overflow_policy, OverflowPolicy):
            raise TypeError("overflow_policy must be an OverflowPolicy")


@dataclass(frozen=True, slots=True)
class SensorTimingTransportConfig:
    """Complete reusable timing and transport configuration for one sensor."""

    schedule: CaptureScheduleConfig
    clock: DeviceClockConfig = field(default_factory=DeviceClockConfig)
    transport: SensorTransportConfig = field(default_factory=SensorTransportConfig)
    seed: int = 0

    def __post_init__(self) -> None:
        if not isinstance(self.schedule, CaptureScheduleConfig):
            raise TypeError("schedule must be a CaptureScheduleConfig")
        if not isinstance(self.clock, DeviceClockConfig):
            raise TypeError("clock must be a DeviceClockConfig")
        if not isinstance(self.transport, SensorTransportConfig):
            raise TypeError("transport must be a SensorTransportConfig")
        if isinstance(self.seed, bool) or not isinstance(self.seed, int):
            raise TypeError("seed must be an integer")


@dataclass(frozen=True, slots=True)
class SensorCapture:
    """One scheduled sensor capture before value generation or transport."""

    sequence: int
    capture_time_s: float
    device_time_s: float

    def __post_init__(self) -> None:
        if isinstance(self.sequence, bool) or not isinstance(self.sequence, int):
            raise TypeError("sequence must be an integer")
        if self.sequence < 0:
            raise ValueError("sequence must be non-negative")
        _finite(self.capture_time_s, "capture_time_s")
        _finite(self.device_time_s, "device_time_s")


@dataclass(frozen=True, slots=True)
class SensorPacket(Generic[PayloadT]):
    """A timestamped sensor payload moving through the simulated link."""

    sequence: int
    capture_time_s: float
    device_time_s: float
    transmission_time_s: float
    arrival_time_s: float
    processing_delay_s: float
    transport_delay_s: float
    payload: PayloadT


@dataclass(frozen=True, slots=True)
class EnqueueOutcome(Generic[PayloadT]):
    """Result of submitting one captured payload to the transport."""

    packet: SensorPacket[PayloadT]
    accepted: bool
    rejected_reason: PacketDropReason | None = None
    evicted_packet: SensorPacket[PayloadT] | None = None


@dataclass(frozen=True, slots=True)
class SensorTransportStats:
    """Immutable runtime counter snapshot."""

    captures: int
    submitted: int
    enqueued: int
    delivered: int
    probabilistic_drops: int
    queue_overflow_drops: int
    pending: int


class SensorTimingTransportRuntime(Generic[PayloadT]):
    """Deterministic fixed-rate sensor timing and bounded transport runtime."""

    _PROCESSING_SEED_SALT = 0x243F6A8885A308D3
    _TRANSPORT_SEED_SALT = 0x13198A2E03707344
    _DROPOUT_SEED_SALT = 0xA4093822299F31D0

    def __init__(self, config: SensorTimingTransportConfig) -> None:
        if not isinstance(config, SensorTimingTransportConfig):
            raise TypeError("config must be a SensorTimingTransportConfig")
        self.config = config
        self._pending: OrderedDict[int, SensorPacket[PayloadT]] = OrderedDict()
        self.reset()

    def reset(self, *, seed: int | None = None) -> None:
        """Reset schedule, queue, counters, and seeded random streams."""

        selected_seed = self.config.seed if seed is None else seed
        if isinstance(selected_seed, bool) or not isinstance(selected_seed, int):
            raise TypeError("seed must be an integer")
        self._processing_rng = random.Random(selected_seed ^ self._PROCESSING_SEED_SALT)
        self._transport_rng = random.Random(selected_seed ^ self._TRANSPORT_SEED_SALT)
        self._dropout_rng = random.Random(selected_seed ^ self._DROPOUT_SEED_SALT)
        self._next_capture_index = 0
        self._last_submitted_sequence = -1
        self._last_observed_time_s = -math.inf
        self._pending.clear()
        self._captures = 0
        self._submitted = 0
        self._enqueued = 0
        self._delivered = 0
        self._probabilistic_drops = 0
        self._queue_overflow_drops = 0

    @property
    def next_capture_time_s(self) -> float:
        """Next scheduled capture on the reference clock [s]."""

        schedule = self.config.schedule
        return schedule.first_capture_time_s + self._next_capture_index * schedule.period_s

    @property
    def pending_count(self) -> int:
        """Number of accepted packets waiting to be drained."""

        return len(self._pending)

    @property
    def stats(self) -> SensorTransportStats:
        """Return an immutable snapshot of runtime counters."""

        return SensorTransportStats(
            captures=self._captures,
            submitted=self._submitted,
            enqueued=self._enqueued,
            delivered=self._delivered,
            probabilistic_drops=self._probabilistic_drops,
            queue_overflow_drops=self._queue_overflow_drops,
            pending=len(self._pending),
        )

    def discard_pending(self) -> tuple[SensorPacket[PayloadT], ...]:
        """Discard queued packets without resetting schedule, counters, or RNGs."""

        packets = tuple(self._pending.values())
        self._pending.clear()
        return packets

    def schedule_until(self, reference_time_s: float) -> list[SensorCapture]:
        """Return every capture due at or before ``reference_time_s``."""

        reference_time_s = self._observe_time(reference_time_s)
        schedule = self.config.schedule
        first_time_s = schedule.first_capture_time_s
        if reference_time_s + schedule.epsilon_s < self.next_capture_time_s:
            return []

        last_due_index = math.floor(
            (reference_time_s + schedule.epsilon_s - first_time_s) / schedule.period_s
        )
        captures = []
        for sequence in range(self._next_capture_index, last_due_index + 1):
            capture_time_s = first_time_s + sequence * schedule.period_s
            captures.append(
                SensorCapture(
                    sequence=sequence,
                    capture_time_s=capture_time_s,
                    device_time_s=self.config.clock.timestamp(capture_time_s),
                )
            )
        self._next_capture_index = last_due_index + 1
        self._captures += len(captures)
        return captures

    def submit(self, capture: SensorCapture, payload: PayloadT) -> EnqueueOutcome[PayloadT]:
        """Apply latency, dropout, and queue policy to one captured payload."""

        if not isinstance(capture, SensorCapture):
            raise TypeError("capture must be a SensorCapture")
        if capture.sequence <= self._last_submitted_sequence:
            raise ValueError(
                "captures must be submitted once in strictly increasing sequence order"
            )
        self._last_submitted_sequence = capture.sequence
        self._submitted += 1

        transport = self.config.transport
        processing_delay_s = transport.processing_latency.sample(self._processing_rng)
        transport_delay_s = transport.transport_latency.sample(self._transport_rng)
        transmission_time_s = capture.capture_time_s + processing_delay_s
        arrival_time_s = transmission_time_s + transport_delay_s
        packet = SensorPacket(
            sequence=capture.sequence,
            capture_time_s=capture.capture_time_s,
            device_time_s=capture.device_time_s,
            transmission_time_s=transmission_time_s,
            arrival_time_s=arrival_time_s,
            processing_delay_s=processing_delay_s,
            transport_delay_s=transport_delay_s,
            payload=payload,
        )

        if self._is_probabilistically_dropped():
            self._probabilistic_drops += 1
            return EnqueueOutcome(
                packet=packet,
                accepted=False,
                rejected_reason=PacketDropReason.PROBABILISTIC,
            )

        evicted_packet = None
        if len(self._pending) >= transport.queue_capacity:
            self._queue_overflow_drops += 1
            if transport.overflow_policy is OverflowPolicy.DROP_NEWEST:
                return EnqueueOutcome(
                    packet=packet,
                    accepted=False,
                    rejected_reason=PacketDropReason.QUEUE_OVERFLOW,
                )
            _, evicted_packet = self._pending.popitem(last=False)

        self._pending[packet.sequence] = packet
        self._enqueued += 1
        return EnqueueOutcome(
            packet=packet,
            accepted=True,
            evicted_packet=evicted_packet,
        )

    def drain_arrived(self, reference_time_s: float) -> list[SensorPacket[PayloadT]]:
        """Remove and return packets whose arrival timestamp is now due."""

        reference_time_s = self._observe_time(reference_time_s)
        epsilon_s = self.config.schedule.epsilon_s
        due_sequences = [
            sequence
            for sequence, packet in self._pending.items()
            if packet.arrival_time_s <= reference_time_s + epsilon_s
        ]
        packets = [self._pending.pop(sequence) for sequence in due_sequences]
        packets.sort(key=lambda packet: (packet.arrival_time_s, packet.sequence))
        self._delivered += len(packets)
        return packets

    def advance(
        self,
        reference_time_s: float,
        payload_factory: Callable[[SensorCapture], PayloadT],
    ) -> list[SensorPacket[PayloadT]]:
        """Capture, submit, and drain all events through ``reference_time_s``."""

        if not callable(payload_factory):
            raise TypeError("payload_factory must be callable")
        captures = self.schedule_until(reference_time_s)
        for capture in captures:
            self.submit(capture, payload_factory(capture))
        return self.drain_arrived(reference_time_s)

    def _observe_time(self, reference_time_s: float) -> float:
        reference_time_s = _finite(reference_time_s, "reference_time_s")
        epsilon_s = self.config.schedule.epsilon_s
        if reference_time_s + epsilon_s < self._last_observed_time_s:
            raise ValueError(
                "reference time moved backwards; call reset() before restarting simulation time"
            )
        self._last_observed_time_s = max(self._last_observed_time_s, reference_time_s)
        return reference_time_s

    def _is_probabilistically_dropped(self) -> bool:
        probability = self.config.transport.dropout_probability
        if probability <= 0.0:
            return False
        if probability >= 1.0:
            return True
        return self._dropout_rng.random() < probability


__all__ = [
    "CaptureScheduleConfig",
    "DeviceClockConfig",
    "EnqueueOutcome",
    "LatencyConfig",
    "OverflowPolicy",
    "PacketDropReason",
    "SensorCapture",
    "SensorPacket",
    "SensorTimingTransportConfig",
    "SensorTimingTransportRuntime",
    "SensorTransportConfig",
    "SensorTransportStats",
]
