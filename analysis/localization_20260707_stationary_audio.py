#!/usr/bin/env python3
"""Read-only stationary-audio diagnostics for a ROS 2 sqlite3 bag."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sqlite3

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def quadratic_peak(freqs: np.ndarray, power: np.ndarray, index: int) -> tuple[float, float]:
    """Return a log-power parabolic peak estimate around one FFT bin."""
    if index <= 0 or index >= power.size - 1:
        return float(freqs[index]), float(power[index])
    y0, y1, y2 = np.log(np.maximum(power[index - 1:index + 2], 1.0e-30))
    denominator = y0 - 2.0 * y1 + y2
    offset = 0.0 if abs(denominator) < 1.0e-18 else 0.5 * (y0 - y2) / denominator
    offset = float(np.clip(offset, -1.0, 1.0))
    frequency = float(freqs[index] + offset * (freqs[1] - freqs[0]))
    peak_log_power = float(y1 - 0.25 * (y0 - y2) * offset)
    return frequency, float(math.exp(peak_log_power))


def separated_peaks(freqs: np.ndarray, power: np.ndarray, lo: float, hi: float, count: int = 12) -> list[dict]:
    mask = (freqs >= lo) & (freqs <= hi)
    candidates = np.flatnonzero(mask)
    ordered = candidates[np.argsort(power[candidates])[::-1]]
    chosen: list[int] = []
    minimum_bins = max(1, int(round(5.0 / (freqs[1] - freqs[0]))))
    for index in ordered:
        if all(abs(int(index) - prior) >= minimum_bins for prior in chosen):
            chosen.append(int(index))
        if len(chosen) >= count:
            break
    output = []
    for index in chosen:
        frequency, peak_power = quadratic_peak(freqs, power, index)
        output.append({
            "frequency_hz": frequency,
            "psd_dbfs_per_hz": float(10.0 * np.log10(max(peak_power, 1.0e-30))),
        })
    return output


def band_peak(freqs: np.ndarray, power: np.ndarray, lo: float, hi: float) -> tuple[float, float]:
    indices = np.flatnonzero((freqs >= lo) & (freqs <= hi))
    index = int(indices[np.argmax(power[indices])])
    return quadratic_peak(freqs, power, index)


def local_snr_db(freqs: np.ndarray, power: np.ndarray, target_hz: float) -> float:
    signal = np.abs(freqs - target_hz) <= 2.0
    noise = (np.abs(freqs - target_hz) >= 20.0) & (np.abs(freqs - target_hz) <= 100.0)
    if not np.any(signal) or not np.any(noise):
        return float("nan")
    return float(10.0 * np.log10(max(float(np.max(power[signal])), 1.0e-30) / max(float(np.median(power[noise])), 1.0e-30)))


def load_audio(bag_path: Path) -> tuple[np.ndarray, dict]:
    connection = sqlite3.connect(f"file:{bag_path}?mode=ro", uri=True)
    topic = connection.execute(
        "SELECT id, type FROM topics WHERE name = '/audio_stamped'"
    ).fetchone()
    if topic is None:
        raise RuntimeError("/audio_stamped is absent")
    topic_id, message_type = topic
    count = int(connection.execute(
        "SELECT COUNT(*) FROM messages WHERE topic_id = ?", (topic_id,)
    ).fetchone()[0])
    message_class = get_message(message_type)

    # The bag uses fixed 10 ms packets. Allocate after inspecting the first one.
    cursor = connection.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp", (topic_id,)
    )
    first_row = cursor.fetchone()
    if first_row is None:
        raise RuntimeError("/audio_stamped contains no messages")
    first_message = deserialize_message(first_row[1], message_class)
    first_pcm = np.frombuffer(bytes(first_message.audio.data), dtype="<i4")
    if first_pcm.size % 2:
        raise RuntimeError("stereo S32LE payload has an odd sample count")
    frames_per_packet = first_pcm.size // 2
    samples = np.empty(count * frames_per_packet, dtype=np.float32)
    header_times = np.empty(count, dtype=np.float64)
    bag_times = np.empty(count, dtype=np.float64)
    packet_frames = np.empty(count, dtype=np.int32)
    channels_identical = True
    clipping_samples = 0
    write_offset = 0

    rows = [(first_row[0], first_row[1])]
    rows.extend(cursor)
    for packet_index, (bag_timestamp, blob) in enumerate(rows):
        message = deserialize_message(blob, message_class)
        pcm = np.frombuffer(bytes(message.audio.data), dtype="<i4").reshape(-1, 2)
        frame_count = pcm.shape[0]
        if write_offset + frame_count > samples.size:
            raise RuntimeError("audio packet size grew beyond preallocation")
        channel_zero = pcm[:, 0]
        samples[write_offset:write_offset + frame_count] = channel_zero.astype(np.float32) / 2147483648.0
        channels_identical = channels_identical and bool(np.array_equal(channel_zero, pcm[:, 1]))
        clipping_samples += int(np.count_nonzero(np.abs(channel_zero.astype(np.int64)) >= int(0.99 * 2147483647)))
        stamp = message.header.stamp
        header_times[packet_index] = float(stamp.sec) + 1.0e-9 * float(stamp.nanosec)
        bag_times[packet_index] = 1.0e-9 * float(bag_timestamp)
        packet_frames[packet_index] = frame_count
        write_offset += frame_count
    connection.close()
    samples = samples[:write_offset]
    metadata = {
        "packet_count": count,
        "sample_count": int(samples.size),
        "frames_per_packet_mode": int(np.bincount(packet_frames).argmax()),
        "packet_frame_min": int(np.min(packet_frames)),
        "packet_frame_max": int(np.max(packet_frames)),
        "channels_bit_identical": bool(channels_identical),
        "clipping_sample_count": int(clipping_samples),
        "clipping_fraction": float(clipping_samples / max(samples.size, 1)),
        "header_start_s": float(header_times[0]),
        "header_end_s": float(header_times[-1]),
        "header_duration_s": float(header_times[-1] - header_times[0]),
        "header_dt_median_ms": float(1000.0 * np.median(np.diff(header_times))),
        "header_dt_p99_ms": float(1000.0 * np.quantile(np.diff(header_times), 0.99)),
        "header_gap_over_15ms_count": int(np.count_nonzero(np.diff(header_times) > 0.015)),
        "bag_duration_s": float(bag_times[-1] - bag_times[0]),
    }
    return samples, metadata


def analyze(samples: np.ndarray, sample_rate_hz: int = 96000) -> tuple[dict, dict[str, np.ndarray]]:
    one_second = sample_rate_hz
    segment_count = samples.size // one_second
    trimmed = samples[:segment_count * one_second].reshape(segment_count, one_second)
    window = np.hanning(one_second).astype(np.float32)
    nfft = 131072
    frequencies = np.fft.rfftfreq(nfft, 1.0 / sample_rate_hz)
    psd_sum = np.zeros(frequencies.size, dtype=np.float64)
    broad_peak_hz = np.empty(segment_count, dtype=np.float64)
    low_peak_hz = np.empty(segment_count, dtype=np.float64)
    high_peak_hz = np.empty(segment_count, dtype=np.float64)
    rms = np.empty(segment_count, dtype=np.float64)
    nominal_snr_21164 = np.empty(segment_count, dtype=np.float64)
    nominal_snr_27211 = np.empty(segment_count, dtype=np.float64)
    normalization = sample_rate_hz * float(np.sum(window.astype(np.float64) ** 2))

    for index, segment in enumerate(trimmed):
        spectrum = np.fft.rfft(segment * window, n=nfft)
        power = (np.abs(spectrum) ** 2) / normalization
        power[1:-1] *= 2.0
        psd_sum += power
        broad_peak_hz[index] = band_peak(frequencies, power, 18000.0, 30000.0)[0]
        low_peak_hz[index] = band_peak(frequencies, power, 20950.0, 21380.0)[0]
        high_peak_hz[index] = band_peak(frequencies, power, 26990.0, 27430.0)[0]
        rms[index] = float(np.sqrt(np.mean(segment.astype(np.float64) ** 2)))
        nominal_snr_21164[index] = local_snr_db(frequencies, power, 21164.0)
        nominal_snr_27211[index] = local_snr_db(frequencies, power, 27211.0)
    average_psd = psd_sum / max(segment_count, 1)
    dominant_hz, dominant_power = band_peak(frequencies, average_psd, 18000.0, 30000.0)

    # Non-overlapping 4096-sample coherent IQ observations at the dominant tone.
    iq_window_size = 4096
    iq_count = samples.size // iq_window_size
    iq_samples = samples[:iq_count * iq_window_size].reshape(iq_count, iq_window_size)
    iq_window = np.hanning(iq_window_size).astype(np.float64)
    local_time = np.arange(iq_window_size, dtype=np.float64) / sample_rate_hz
    local_oscillator = iq_window * np.exp(-2j * np.pi * dominant_hz * local_time)
    iq = np.empty(iq_count, dtype=np.complex128)
    block_duration_s = iq_window_size / sample_rate_hz
    for index, segment in enumerate(iq_samples):
        absolute_phase = np.exp(-2j * np.pi * dominant_hz * index * block_duration_s)
        iq[index] = absolute_phase * np.dot(segment.astype(np.float64), local_oscillator) / np.sum(iq_window)
    iq_time_s = (np.arange(iq_count, dtype=np.float64) + 0.5) * block_duration_s
    phase_step = np.angle(iq[1:] * np.conj(iq[:-1]))
    frequency_residual_hz = phase_step / (2.0 * np.pi * block_duration_s)
    magnitude = np.abs(iq)

    # Residual phase after independent ten-second linear fits measures non-linear wobble.
    unwrapped_phase = np.unwrap(np.angle(iq))
    phase_residual = np.full(iq_count, np.nan, dtype=np.float64)
    group_size = max(4, int(round(10.0 / block_duration_s)))
    local_slopes_hz = []
    for start in range(0, iq_count, group_size):
        stop = min(start + group_size, iq_count)
        if stop - start < 4:
            continue
        relative_time = iq_time_s[start:stop] - iq_time_s[start]
        coefficients = np.polyfit(relative_time, unwrapped_phase[start:stop], 1)
        fitted = np.polyval(coefficients, relative_time)
        phase_residual[start:stop] = unwrapped_phase[start:stop] - fitted
        local_slopes_hz.append(float(coefficients[0] / (2.0 * np.pi)))
    valid_residual = phase_residual[np.isfinite(phase_residual)]

    result = {
        "sample_rate_hz": sample_rate_hz,
        "analyzed_duration_s": float(samples.size / sample_rate_hz),
        "signal_rms_median": float(np.median(rms)),
        "signal_rms_p95": float(np.quantile(rms, 0.95)),
        "dominant_frequency_hz": dominant_hz,
        "dominant_psd_dbfs_per_hz": float(10.0 * np.log10(max(dominant_power, 1.0e-30))),
        "dominant_frequency_per_second_median_hz": float(np.median(broad_peak_hz)),
        "dominant_frequency_per_second_p05_hz": float(np.quantile(broad_peak_hz, 0.05)),
        "dominant_frequency_per_second_p95_hz": float(np.quantile(broad_peak_hz, 0.95)),
        "dominant_frequency_per_second_std_hz": float(np.std(broad_peak_hz)),
        "low_band_peak_median_hz": float(np.median(low_peak_hz)),
        "high_band_peak_median_hz": float(np.median(high_peak_hz)),
        "snr_21164_db_median": float(np.median(nominal_snr_21164)),
        "snr_21164_db_p95": float(np.quantile(nominal_snr_21164, 0.95)),
        "snr_27211_db_median": float(np.median(nominal_snr_27211)),
        "snr_27211_db_p95": float(np.quantile(nominal_snr_27211, 0.95)),
        "top_spectral_peaks_18_30khz": separated_peaks(frequencies, average_psd, 18000.0, 30000.0),
        "dominant_iq_magnitude_median": float(np.median(magnitude)),
        "dominant_iq_magnitude_p05": float(np.quantile(magnitude, 0.05)),
        "dominant_iq_magnitude_p95": float(np.quantile(magnitude, 0.95)),
        "dominant_frequency_residual_median_hz": float(np.median(frequency_residual_hz)),
        "dominant_frequency_residual_mad_hz": float(np.median(np.abs(frequency_residual_hz - np.median(frequency_residual_hz)))),
        "dominant_frequency_residual_p05_hz": float(np.quantile(frequency_residual_hz, 0.05)),
        "dominant_frequency_residual_p95_hz": float(np.quantile(frequency_residual_hz, 0.95)),
        "ten_second_frequency_slope_median_hz": float(np.median(local_slopes_hz)),
        "ten_second_frequency_slope_p05_hz": float(np.quantile(local_slopes_hz, 0.05)),
        "ten_second_frequency_slope_p95_hz": float(np.quantile(local_slopes_hz, 0.95)),
        "phase_residual_10s_rms_rad": float(np.sqrt(np.mean(valid_residual ** 2))),
        "phase_residual_10s_median_abs_rad": float(np.median(np.abs(valid_residual))),
        "phase_step_over_90deg_fraction": float(np.mean(np.abs(phase_step) > 0.5 * np.pi)),
    }
    arrays = {
        "frequencies_hz": frequencies,
        "average_psd": average_psd,
        "second_time_s": np.arange(segment_count, dtype=np.float64) + 0.5,
        "broad_peak_hz": broad_peak_hz,
        "low_peak_hz": low_peak_hz,
        "high_peak_hz": high_peak_hz,
        "rms": rms,
        "snr_21164_db": nominal_snr_21164,
        "snr_27211_db": nominal_snr_27211,
        "iq_time_s": iq_time_s,
        "iq_magnitude": magnitude,
        "frequency_residual_time_s": iq_time_s[1:],
        "frequency_residual_hz": frequency_residual_hz,
        "phase_residual_rad": phase_residual,
    }
    return result, arrays


def save_plots(output_dir: Path, result: dict, arrays: dict[str, np.ndarray]) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    frequencies = arrays["frequencies_hz"]
    psd = arrays["average_psd"]
    band = (frequencies >= 18000.0) & (frequencies <= 30000.0)
    fig, ax = plt.subplots(figsize=(11, 5))
    ax.plot(frequencies[band], 10.0 * np.log10(np.maximum(psd[band], 1.0e-30)), linewidth=0.8)
    ax.axvline(21164.0, color="tab:green", linestyle="--", label="21,164 Hz")
    ax.axvline(27211.0, color="tab:orange", linestyle="--", label="27,211 Hz")
    ax.axvline(result["dominant_frequency_hz"], color="tab:red", linestyle=":", label="dominant")
    ax.set(title="Average audio PSD (18–30 kHz)", xlabel="Frequency (Hz)", ylabel="PSD (dBFS/Hz)")
    ax.grid(alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_dir / "average_psd_18_30khz.png", dpi=150)
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    time_s = arrays["second_time_s"]
    axes[0].plot(time_s, arrays["broad_peak_hz"], linewidth=0.8)
    axes[0].set_ylabel("Peak (Hz)")
    axes[0].set_title("Per-second dominant frequency")
    axes[1].plot(time_s, arrays["snr_21164_db"], label="21,164 Hz", linewidth=0.8)
    axes[1].plot(time_s, arrays["snr_27211_db"], label="27,211 Hz", linewidth=0.8)
    axes[1].set_ylabel("Local SNR (dB)")
    axes[1].legend()
    axes[2].plot(time_s, arrays["rms"], linewidth=0.8)
    axes[2].set(xlabel="Time from audio start (s)", ylabel="RMS FS")
    for ax in axes:
        ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(output_dir / "frequency_snr_rms_time.png", dpi=150)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    axes[0].plot(arrays["frequency_residual_time_s"], arrays["frequency_residual_hz"], linewidth=0.5)
    axes[0].set(ylabel="Residual frequency (Hz)", title="Dominant-tone coherent phase diagnostics")
    axes[1].plot(arrays["iq_time_s"], arrays["iq_magnitude"], linewidth=0.5)
    axes[1].set(xlabel="Time from audio start (s)", ylabel="IQ magnitude")
    for ax in axes:
        ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(output_dir / "dominant_phase_frequency_time.png", dpi=150)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    samples, metadata = load_audio(args.bag)
    result, arrays = analyze(samples)
    payload = {
        "source": str(args.bag),
        "metadata": metadata,
        "analysis": result,
        "limitations": [
            "No independent ground truth confirms that the vehicle and pinger were mechanically stationary.",
            "The two recorded audio channels are bit-identical, so differential-channel phase cannot be evaluated.",
            "A strong spectral line is not automatically the competition pinger without an ON/OFF or frequency label.",
        ],
    }
    with (args.output_dir / "stationary_audio_summary.json").open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, ensure_ascii=False)
    np.savez_compressed(args.output_dir / "stationary_audio_timeseries.npz", **arrays)
    save_plots(args.output_dir, result, arrays)
    print(json.dumps(payload, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
