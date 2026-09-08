#!/usr/bin/env python3
"""Offline timing and resource tests for the underwater camera runtime."""

from __future__ import annotations

import unittest
from dataclasses import replace
from pathlib import Path
import sys

import numpy as np


CURRENT = Path(__file__).resolve().parents[1]
if str(CURRENT) not in sys.path:
    sys.path.insert(0, str(CURRENT))

from bridge.ros2_camera_sensor_runtime import (  # noqa: E402
    CameraSensorRuntime,
    RenderedCameraFrame,
)
from bridge.underwater_camera_sensor_model import (  # noqa: E402
    CameraElectronicsConfig,
    CameraTimingConfig,
    UnderwaterOpticsConfig,
    load_underwater_camera_profile,
)


def _profile(**timing_overrides):
    profile = load_underwater_camera_profile()
    timing = CameraTimingConfig(**timing_overrides)
    return replace(
        profile,
        optics=UnderwaterOpticsConfig(),
        electronics=CameraElectronicsConfig(),
        timing=timing,
    )


def _runtime(profile, *, rate_hz: float = 10.0, **overrides) -> CameraSensorRuntime:
    return CameraSensorRuntime(
        profile,
        camera_name="stereo_left",
        calibration=profile.calibration.scaled_to(16, 12),
        frame_rate_hz=rate_hz,
        **overrides,
    )


def _frame(time_s: float, value: int = 100) -> RenderedCameraFrame:
    return RenderedCameraFrame(
        rgb=np.full((12, 16, 3), value, dtype=np.uint8),
        capture_time_s=time_s,
    )


class CameraSensorRuntimeTest(unittest.TestCase):
    def test_zero_latency_delivers_capture_immediately(self) -> None:
        runtime = _runtime(_profile())

        deliveries = runtime.advance(2.5, _frame(2.5, 77))

        self.assertEqual(len(deliveries), 1)
        delivery = deliveries[0]
        self.assertEqual(delivery.sequence, 0)
        self.assertEqual(delivery.capture_time_s, 2.5)
        self.assertEqual(delivery.arrival_time_s, 2.5)
        np.testing.assert_array_equal(delivery.rgb, _frame(0.0, 77).rgb)

    def test_capture_rate_rejects_too_early_render_without_duplicate_packet(self) -> None:
        runtime = _runtime(_profile(), rate_hz=10.0)

        self.assertEqual(len(runtime.advance(0.0, _frame(0.0))), 1)
        self.assertEqual(runtime.advance(0.05, _frame(0.05)), ())
        deliveries = runtime.advance(0.1, _frame(0.1))

        self.assertEqual([item.sequence for item in deliveries], [1])
        self.assertEqual(runtime.stats.rate_limited_frames, 1)

    def test_duplicate_async_render_is_not_resubmitted_but_queue_still_drains(self) -> None:
        runtime = _runtime(
            _profile(processing_latency_mean_s=0.1, processing_latency_max_s=0.1)
        )
        rendered = _frame(0.0)

        self.assertEqual(runtime.advance(0.0, rendered), ())
        deliveries = runtime.advance(0.1, rendered)

        self.assertEqual([item.sequence for item in deliveries], [0])
        self.assertEqual(runtime.stats.duplicate_or_stale_frames, 1)
        self.assertEqual(runtime.stats.accepted_captures, 1)

    def test_processing_and_transport_latency_are_additive(self) -> None:
        runtime = _runtime(
            _profile(
                processing_latency_mean_s=0.04,
                processing_latency_max_s=0.04,
                transport_latency_mean_s=0.06,
                transport_latency_max_s=0.06,
            )
        )

        self.assertEqual(runtime.advance(1.0, _frame(1.0)), ())
        self.assertEqual(runtime.advance(1.099, None), ())
        deliveries = runtime.advance(1.1, None)

        self.assertEqual(len(deliveries), 1)
        self.assertAlmostEqual(deliveries[0].transmission_time_s, 1.04)
        self.assertAlmostEqual(deliveries[0].arrival_time_s, 1.1)

    def test_device_clock_offset_and_drift_are_preserved(self) -> None:
        runtime = _runtime(
            _profile(device_clock_offset_s=0.02, device_clock_drift_ppm=100.0)
        )

        delivery = runtime.advance(10.0, _frame(10.0))[0]

        self.assertAlmostEqual(delivery.device_time_s, 10.021)

    def test_probability_one_drops_every_frame(self) -> None:
        runtime = _runtime(_profile(dropout_probability=1.0))

        self.assertEqual(runtime.advance(0.0, _frame(0.0)), ())
        self.assertEqual(runtime.advance(0.1, _frame(0.1)), ())

        self.assertEqual(runtime.stats.probabilistic_drops, 2)
        self.assertEqual(runtime.stats.pending_frames, 0)

    def test_queue_is_bounded_and_drop_oldest_keeps_newest_frames(self) -> None:
        runtime = _runtime(
            _profile(
                processing_latency_mean_s=1.0,
                processing_latency_max_s=1.0,
                queue_capacity=2,
                overflow_policy="drop_oldest",
            )
        )
        self.assertEqual(runtime.advance(0.0, _frame(0.0, 10)), ())
        self.assertEqual(runtime.advance(0.1, _frame(0.1, 20)), ())
        self.assertEqual(runtime.advance(0.2, _frame(0.2, 30)), ())

        deliveries = runtime.advance(1.2, None)

        self.assertEqual([item.sequence for item in deliveries], [1, 2])
        self.assertEqual([int(item.rgb[0, 0, 0]) for item in deliveries], [20, 30])
        self.assertEqual(runtime.stats.queue_overflow_drops, 1)

    def test_seeded_jitter_is_reproducible(self) -> None:
        profile = _profile(
            processing_latency_mean_s=0.05,
            processing_latency_jitter_s=0.01,
            processing_latency_max_s=0.1,
        )
        first = _runtime(profile, seed=42)
        second = _runtime(profile, seed=42)

        first.advance(0.0, _frame(0.0))
        second.advance(0.0, _frame(0.0))

        first_packet = next(iter(first.transport._pending.values()))
        second_packet = next(iter(second.transport._pending.values()))
        self.assertEqual(first_packet.arrival_time_s, second_packet.arrival_time_s)

    def test_seed_override_controls_both_transport_and_pixel_noise(self) -> None:
        base = load_underwater_camera_profile()
        profile = replace(
            base,
            optics=UnderwaterOpticsConfig(),
            electronics=CameraElectronicsConfig(
                shot_noise_electrons_per_unit=400.0,
                read_noise_electrons_rms=2.0,
            ),
            timing=CameraTimingConfig(),
        )
        first = _runtime(profile, seed=11)
        same = _runtime(profile, seed=11)
        different = _runtime(profile, seed=12)

        first_rgb = first.advance(0.0, _frame(0.0))[0].rgb
        same_rgb = same.advance(0.0, _frame(0.0))[0].rgb
        different_rgb = different.advance(0.0, _frame(0.0))[0].rgb

        np.testing.assert_array_equal(first_rgb, same_rgb)
        self.assertFalse(np.array_equal(first_rgb, different_rgb))

    def test_backward_sim_time_resets_pending_frames_and_rng_state(self) -> None:
        runtime = _runtime(
            _profile(processing_latency_mean_s=1.0, processing_latency_max_s=1.0)
        )
        self.assertEqual(runtime.advance(3.0, _frame(3.0)), ())
        self.assertEqual(runtime.stats.pending_frames, 1)

        self.assertEqual(runtime.advance(0.0, _frame(0.0)), ())

        self.assertEqual(runtime.stats.accepted_captures, 1)
        self.assertEqual(runtime.stats.pending_frames, 1)
        self.assertEqual(runtime.advance(1.0, None)[0].capture_time_s, 0.0)

    def test_close_releases_pending_frames_and_model_caches(self) -> None:
        profile = _profile(
            processing_latency_mean_s=1.0,
            processing_latency_max_s=1.0,
        )
        runtime = _runtime(profile)
        runtime.model._vignette_cache[(1, 1)] = np.ones((1, 1), dtype=np.float32)
        runtime.advance(0.0, _frame(0.0))

        runtime.close()
        runtime.close()

        self.assertEqual(runtime.transport.pending_count, 0)
        self.assertEqual(runtime.model._vignette_cache, {})
        self.assertEqual(runtime.advance(2.0, _frame(2.0)), ())


if __name__ == "__main__":
    unittest.main(verbosity=2)
