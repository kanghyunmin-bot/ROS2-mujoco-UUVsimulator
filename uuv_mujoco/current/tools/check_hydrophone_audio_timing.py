#!/usr/bin/env python3
"""Regression checks for continuous synthetic hydrophone PCM timing."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
import sys
from unittest.mock import patch

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from bridge.ros2_hydrophone_sim import (  # noqa: E402
    _add_snr_probe_noise,
    _add_tone,
    _next_audio_frame_count,
)


def _assert(condition: bool, label: str) -> None:
    if not condition:
        raise AssertionError(label)


def check_frame_count_follows_wall_elapsed_time() -> None:
    bridge = SimpleNamespace(_hydrophone_audio_last_wall=None)
    cfg = SimpleNamespace(sample_rate_hz=96000, publish_hz=10.0)
    with patch("bridge.ros2_hydrophone_sim.time.monotonic", side_effect=[100.0, 100.25]):
        first = _next_audio_frame_count(bridge, cfg)
        delayed = _next_audio_frame_count(bridge, cfg)
    _assert(first == 9600, "first buffer must use the nominal duration")
    _assert(delayed == 24000, "delayed publication must preserve all elapsed PCM frames")


def check_new_subscriber_does_not_receive_stale_pcm_backlog() -> None:
    bridge = SimpleNamespace(_hydrophone_audio_last_wall=None)
    cfg = SimpleNamespace(sample_rate_hz=96000, publish_hz=23.4375)
    with patch("bridge.ros2_hydrophone_sim.time.monotonic", side_effect=[100.0, 110.0]):
        first = _next_audio_frame_count(bridge, cfg)
        after_unsubscribed_gap = _next_audio_frame_count(bridge, cfg)
    _assert(first == 4096, "first buffer must be nominal")
    _assert(
        after_unsubscribed_gap == 4096,
        "a new phase receiver must not be given stale catch-up PCM",
    )


def check_frame_count_follows_sim_elapsed_time() -> None:
    bridge = SimpleNamespace(
        _hydrophone_audio_last_wall=None,
        _hydrophone_audio_last_sim_time=None,
    )
    cfg = SimpleNamespace(sample_rate_hz=96000, publish_hz=23.4375)
    first = _next_audio_frame_count(bridge, cfg, sim_time_s=10.0)
    second = _next_audio_frame_count(bridge, cfg, sim_time_s=10.042666666666667)
    reset = _next_audio_frame_count(bridge, cfg, sim_time_s=0.0)
    _assert(first == 4096, "first sim-time buffer must use the nominal duration")
    _assert(second == 4096, "PCM duration must follow MuJoCo time, independent of RTF")
    _assert(reset == 4096, "a MuJoCo clock reset must restart at nominal duration")


def check_range_is_interpolated_per_sample() -> None:
    samples = np.arange(8, dtype=np.float32)
    signal = np.zeros((8, 1), dtype=np.float32)
    ranges = [np.linspace(1.0, 1.2, 8, dtype=np.float32)]
    _add_tone(
        signal,
        samples,
        frequency_hz=1000.0,
        sample_rate_hz=8000,
        sound_speed_mps=1500.0,
        amplitude=0.5,
        phase_rad=0.0,
        ranges_m=ranges,
    )
    _assert(np.all(np.isfinite(signal)), "interpolated range tone must remain finite")
    _assert(float(np.ptp(signal[:, 0])) > 0.1, "interpolated range tone must contain a waveform")


def check_long_running_phase_offset_is_stable() -> None:
    frames = 256
    offset = 96_000 * 60 * 45
    samples = np.arange(frames, dtype=np.float32)
    signal = np.zeros((frames, 1), dtype=np.float32)
    _add_tone(
        signal,
        samples,
        frequency_hz=21164.0,
        sample_rate_hz=96000,
        sound_speed_mps=1500.0,
        amplitude=0.5,
        phase_rad=0.3,
        ranges_m=[2.0],
        sample_offset=offset,
    )
    absolute = np.arange(offset, offset + frames, dtype=np.float64)
    expected = 0.5 * np.sin(
        2.0 * np.pi * 21164.0 * absolute / 96000.0
        + 0.3
        - 2.0 * np.pi * 21164.0 * 2.0 / 1500.0
    )
    _assert(
        float(np.max(np.abs(signal[:, 0] - expected))) < 2.0e-4,
        "float32 synthesis lost long-running absolute phase",
    )


def check_phase_unwrap_speed_contract() -> None:
    frequency_hz = 21164.0
    sound_speed_mps = 1500.0
    analysis_window_s = 4096.0 / 96000.0
    max_unambiguous_radial_speed_mps = (
        0.5 * sound_speed_mps / frequency_hz / analysis_window_s
    )
    _assert(
        0.80 < max_unambiguous_radial_speed_mps < 0.85,
        "unexpected single-frequency phase-unwrapping speed limit",
    )


def _hann_iq_magnitude(signal: np.ndarray, frequency_hz: float, sample_rate_hz: int) -> float:
    count = signal.shape[0]
    samples = np.arange(count, dtype=np.float64)
    weights = np.hanning(count)
    carrier = np.exp(-2.0j * np.pi * frequency_hz * samples / sample_rate_hz)
    return float(abs(np.sum(weights * signal * carrier) / np.sum(weights)))


def check_snr_probe_floor_tracks_target_amplitude() -> None:
    sample_rate_hz = 96000
    frequency_hz = 21164.0
    samples = np.arange(4096, dtype=np.float32)
    cfg = SimpleNamespace(
        frequency_hz=frequency_hz,
        sample_rate_hz=sample_rate_hz,
        sound_speed_mps=1500.0,
        snr_probe_noise_amplitude=0.006,
    )

    def measured_ratio(target_amplitude: float) -> float:
        signal = np.zeros((samples.size, 1), dtype=np.float32)
        _add_snr_probe_noise(signal, samples, cfg=cfg)
        _add_tone(
            signal,
            samples,
            frequency_hz=frequency_hz,
            sample_rate_hz=sample_rate_hz,
            sound_speed_mps=1500.0,
            amplitude=target_amplitude,
            phase_rad=0.0,
            ranges_m=[2.0],
        )
        target = _hann_iq_magnitude(signal[:, 0], frequency_hz, sample_rate_hz)
        sidebands = [
            _hann_iq_magnitude(signal[:, 0], frequency_hz + offset, sample_rate_hz)
            for offset in (-700.0, -450.0, -250.0, 250.0, 450.0, 700.0)
        ]
        return target / float(np.median(sidebands))

    low = measured_ratio(0.10)
    high = measured_ratio(0.20)
    _assert(10.0 < low < 25.0, "unexpected synthetic SNR reference level")
    _assert(1.9 < high / low < 2.1, "synthetic SNR must track pinger amplitude")


def main() -> int:
    check_frame_count_follows_wall_elapsed_time()
    check_new_subscriber_does_not_receive_stale_pcm_backlog()
    check_frame_count_follows_sim_elapsed_time()
    check_range_is_interpolated_per_sample()
    check_long_running_phase_offset_is_stable()
    check_phase_unwrap_speed_contract()
    check_snr_probe_floor_tracks_target_amplitude()
    print("hydrophone_audio_timing=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
