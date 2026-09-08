#!/usr/bin/env python3
"""Offline unit tests for ROS-independent sensor timing and transport."""

from __future__ import annotations

import math
import sys
import unittest
from dataclasses import asdict
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim.sensors import (  # noqa: E402
    CaptureScheduleConfig,
    DeviceClockConfig,
    LatencyConfig,
    OverflowPolicy,
    PacketDropReason,
    SensorCapture,
    SensorTimingTransportConfig,
    SensorTimingTransportRuntime,
    SensorTransportConfig,
)


def runtime_config(
    *,
    seed: int = 7,
    dropout_probability: float = 0.0,
    queue_capacity: int = 32,
    overflow_policy: OverflowPolicy = OverflowPolicy.DROP_OLDEST,
    processing_latency: LatencyConfig | None = None,
    transport_latency: LatencyConfig | None = None,
) -> SensorTimingTransportConfig:
    return SensorTimingTransportConfig(
        schedule=CaptureScheduleConfig(rate_hz=10.0),
        transport=SensorTransportConfig(
            processing_latency=processing_latency or LatencyConfig(),
            transport_latency=transport_latency or LatencyConfig(),
            dropout_probability=dropout_probability,
            queue_capacity=queue_capacity,
            overflow_policy=overflow_policy,
        ),
        seed=seed,
    )


class SensorTimingConfigTest(unittest.TestCase):
    def test_validation_rejects_nonphysical_values(self) -> None:
        with self.assertRaises(ValueError):
            CaptureScheduleConfig(rate_hz=0.0)
        with self.assertRaises(ValueError):
            CaptureScheduleConfig(rate_hz=10.0, phase_s=0.1)
        with self.assertRaises(ValueError):
            DeviceClockConfig(drift_ppm=-1_000_000.0)
        with self.assertRaises(ValueError):
            LatencyConfig(mean_s=0.01, jitter_std_s=-0.1)
        with self.assertRaises(ValueError):
            LatencyConfig(mean_s=0.02, max_s=0.01)
        with self.assertRaises(ValueError):
            SensorTransportConfig(dropout_probability=1.01)
        with self.assertRaises(ValueError):
            SensorTransportConfig(queue_capacity=0)
        with self.assertRaises(TypeError):
            SensorTimingTransportConfig(
                schedule=CaptureScheduleConfig(rate_hz=1.0), seed=True
            )
        with self.assertRaises(TypeError):
            SensorTransportConfig(processing_latency=object())
        with self.assertRaises(TypeError):
            SensorTimingTransportConfig(schedule=object())
        with self.assertRaises(ValueError):
            SensorCapture(sequence=-1, capture_time_s=0.0, device_time_s=0.0)
        with self.assertRaises(ValueError):
            SensorCapture(sequence=0, capture_time_s=math.inf, device_time_s=0.0)

    def test_rate_phase_and_affine_device_clock(self) -> None:
        config = SensorTimingTransportConfig(
            schedule=CaptureScheduleConfig(
                rate_hz=4.0,
                phase_s=0.125,
                start_time_s=10.0,
            ),
            clock=DeviceClockConfig(
                offset_s=0.02,
                drift_ppm=100.0,
                reference_time_s=10.0,
            ),
        )
        runtime: SensorTimingTransportRuntime[int] = SensorTimingTransportRuntime(config)

        self.assertEqual(runtime.schedule_until(10.124), [])
        captures = runtime.schedule_until(10.625)
        self.assertEqual([item.sequence for item in captures], [0, 1, 2])
        self.assertEqual(
            [round(item.capture_time_s, 6) for item in captures],
            [10.125, 10.375, 10.625],
        )
        for capture in captures:
            expected = (
                capture.capture_time_s
                + 0.02
                + (capture.capture_time_s - 10.0) * 100.0e-6
            )
            self.assertAlmostEqual(capture.device_time_s, expected, places=12)


class SensorTransportRuntimeTest(unittest.TestCase):
    def test_timestamp_pipeline_is_causal_and_exact_without_jitter(self) -> None:
        runtime: SensorTimingTransportRuntime[str] = SensorTimingTransportRuntime(
            runtime_config(
                processing_latency=LatencyConfig(mean_s=0.012),
                transport_latency=LatencyConfig(mean_s=0.008),
            )
        )
        capture = runtime.schedule_until(0.0)[0]
        outcome = runtime.submit(capture, "sample")

        self.assertTrue(outcome.accepted)
        self.assertAlmostEqual(outcome.packet.capture_time_s, 0.0)
        self.assertAlmostEqual(outcome.packet.transmission_time_s, 0.012)
        self.assertAlmostEqual(outcome.packet.arrival_time_s, 0.020)
        self.assertEqual(runtime.drain_arrived(0.019), [])
        delivered = runtime.drain_arrived(0.020)
        self.assertEqual([packet.payload for packet in delivered], ["sample"])

    def test_seeded_jitter_and_dropout_replay_exactly_after_reset(self) -> None:
        config = runtime_config(
            seed=8842,
            dropout_probability=0.35,
            processing_latency=LatencyConfig(
                mean_s=0.012,
                jitter_std_s=0.004,
                min_s=0.002,
                max_s=0.025,
            ),
            transport_latency=LatencyConfig(
                mean_s=0.020,
                jitter_std_s=0.007,
                min_s=0.001,
                max_s=0.050,
            ),
        )
        runtime: SensorTimingTransportRuntime[int] = SensorTimingTransportRuntime(config)

        def run_once() -> tuple[list[dict[str, object]], dict[str, int]]:
            delivered = runtime.advance(1.0, lambda capture: capture.sequence)
            delivered.extend(runtime.drain_arrived(2.0))
            return [asdict(packet) for packet in delivered], asdict(runtime.stats)

        first_packets, first_stats = run_once()
        runtime.reset()
        second_packets, second_stats = run_once()

        self.assertEqual(first_packets, second_packets)
        self.assertEqual(first_stats, second_stats)
        self.assertGreater(first_stats["probabilistic_drops"], 0)
        for packet in first_packets:
            self.assertGreaterEqual(packet["processing_delay_s"], 0.002)
            self.assertLessEqual(packet["processing_delay_s"], 0.025)
            self.assertGreaterEqual(packet["transport_delay_s"], 0.001)
            self.assertLessEqual(packet["transport_delay_s"], 0.050)

    def test_probability_one_drops_every_packet(self) -> None:
        runtime: SensorTimingTransportRuntime[int] = SensorTimingTransportRuntime(
            runtime_config(dropout_probability=1.0)
        )
        delivered = runtime.advance(0.5, lambda capture: capture.sequence)

        self.assertEqual(delivered, [])
        self.assertEqual(runtime.pending_count, 0)
        self.assertEqual(runtime.stats.captures, 6)
        self.assertEqual(runtime.stats.probabilistic_drops, 6)

    def test_drop_newest_queue_policy_preserves_old_packets(self) -> None:
        config = SensorTimingTransportConfig(
            schedule=CaptureScheduleConfig(rate_hz=1.0),
            transport=SensorTransportConfig(
                queue_capacity=2,
                overflow_policy=OverflowPolicy.DROP_NEWEST,
            ),
        )
        runtime: SensorTimingTransportRuntime[int] = SensorTimingTransportRuntime(config)
        captures = runtime.schedule_until(2.0)
        outcomes = [runtime.submit(capture, capture.sequence) for capture in captures]

        self.assertTrue(outcomes[0].accepted)
        self.assertTrue(outcomes[1].accepted)
        self.assertFalse(outcomes[2].accepted)
        self.assertIs(outcomes[2].rejected_reason, PacketDropReason.QUEUE_OVERFLOW)
        self.assertEqual(
            [packet.sequence for packet in runtime.drain_arrived(2.0)],
            [0, 1],
        )
        self.assertEqual(runtime.stats.queue_overflow_drops, 1)

    def test_drop_oldest_queue_policy_preserves_new_packets(self) -> None:
        config = SensorTimingTransportConfig(
            schedule=CaptureScheduleConfig(rate_hz=1.0),
            transport=SensorTransportConfig(
                queue_capacity=2,
                overflow_policy=OverflowPolicy.DROP_OLDEST,
            ),
        )
        runtime: SensorTimingTransportRuntime[int] = SensorTimingTransportRuntime(config)
        captures = runtime.schedule_until(2.0)
        outcomes = [runtime.submit(capture, capture.sequence) for capture in captures]

        self.assertTrue(outcomes[2].accepted)
        self.assertIsNotNone(outcomes[2].evicted_packet)
        self.assertEqual(outcomes[2].evicted_packet.sequence, 0)
        self.assertEqual(
            [packet.sequence for packet in runtime.drain_arrived(2.0)],
            [1, 2],
        )
        self.assertEqual(runtime.stats.queue_overflow_drops, 1)

    def test_backward_time_is_rejected_and_reset_restarts_schedule(self) -> None:
        runtime: SensorTimingTransportRuntime[int] = SensorTimingTransportRuntime(runtime_config())
        first = runtime.schedule_until(0.2)
        self.assertEqual([capture.sequence for capture in first], [0, 1, 2])
        with self.assertRaises(ValueError):
            runtime.schedule_until(0.1)

        runtime.reset()
        restarted = runtime.schedule_until(0.0)
        self.assertEqual([capture.sequence for capture in restarted], [0])

    def test_discard_pending_preserves_schedule_and_unblocks_bounded_queue(self) -> None:
        config = SensorTimingTransportConfig(
            schedule=CaptureScheduleConfig(rate_hz=10.0),
            transport=SensorTransportConfig(
                transport_latency=LatencyConfig(mean_s=1.0),
                queue_capacity=1,
                overflow_policy=OverflowPolicy.DROP_NEWEST,
            ),
        )
        runtime: SensorTimingTransportRuntime[int] = SensorTimingTransportRuntime(config)
        first_capture = runtime.schedule_until(0.0)[0]
        self.assertTrue(runtime.submit(first_capture, 10).accepted)

        discarded = runtime.discard_pending()
        next_capture = runtime.schedule_until(0.1)[0]
        next_outcome = runtime.submit(next_capture, 20)

        self.assertEqual([packet.payload for packet in discarded], [10])
        self.assertEqual(next_capture.sequence, 1)
        self.assertTrue(next_outcome.accepted)
        self.assertEqual(runtime.pending_count, 1)
        self.assertEqual(runtime.stats.queue_overflow_drops, 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
