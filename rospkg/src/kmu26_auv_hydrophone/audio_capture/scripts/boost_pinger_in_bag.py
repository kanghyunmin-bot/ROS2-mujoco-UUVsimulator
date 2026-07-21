#!/usr/bin/env python3
"""Create a ROS 2 bag with a boosted pinger component on /audio_boosted."""

from __future__ import annotations

import argparse
import math
import sys
from array import array
from pathlib import Path
from typing import Iterable

import numpy as np
from rclpy.serialization import deserialize_message, serialize_message
from rosbag2_py import ConverterOptions, SequentialReader, SequentialWriter, StorageOptions, TopicMetadata
from scipy import signal

from audio_common_msgs.msg import AudioData, AudioInfo


PCM_FORMATS = {
    "S16LE": (np.dtype("<i2"), 32768.0, -32768, 32767),
    "S32LE": (np.dtype("<i4"), 2147483648.0, -2147483648, 2147483647),
}


def parse_frequency_list(value: str) -> list[float]:
    frequencies = []
    for item in value.split(","):
        item = item.strip()
        if item:
            frequencies.append(float(item))
    if not frequencies:
        raise argparse.ArgumentTypeError("at least one frequency is required")
    return frequencies


def topic_metadata_like(topic: TopicMetadata, name: str | None = None) -> TopicMetadata:
    return TopicMetadata(
        name=name if name is not None else topic.name,
        type=topic.type,
        serialization_format=topic.serialization_format,
        offered_qos_profiles=topic.offered_qos_profiles,
    )


def open_reader(bag_uri: str) -> SequentialReader:
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_uri, storage_id="sqlite3"),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    return reader


def dtype_for_format(sample_format: str) -> tuple[np.dtype, float, int, int]:
    try:
        return PCM_FORMATS[sample_format.upper()]
    except KeyError as exc:
        supported = ", ".join(sorted(PCM_FORMATS))
        raise ValueError(f"unsupported sample format {sample_format!r}; supported: {supported}") from exc


def pcm_bytes_to_frames(
    pcm: bytes,
    channels: int,
    sample_format: str,
) -> tuple[np.ndarray, bytes, float, int, int]:
    dtype, scale, min_value, max_value = dtype_for_format(sample_format)
    bytes_per_sample = dtype.itemsize
    valid_sample_count = (len(pcm) // bytes_per_sample // channels) * channels
    valid_byte_count = valid_sample_count * bytes_per_sample
    if valid_sample_count == 0:
        return np.empty((0, channels), dtype=dtype), pcm, scale, min_value, max_value
    samples = np.frombuffer(pcm[:valid_byte_count], dtype=dtype)
    return samples.reshape(-1, channels), pcm[valid_byte_count:], scale, min_value, max_value


def estimate_f0_hz(
    input_bag: str,
    audio_topic: str,
    sample_rate: int,
    channels: int,
    sample_format: str,
    channel_index: int,
    candidates_hz: Iterable[float],
    search_half_width_hz: float,
    seconds: float,
) -> float:
    reader = open_reader(input_bag)
    target_frames = max(1, int(round(seconds * sample_rate)))
    chunks: list[np.ndarray] = []
    collected = 0

    while reader.has_next() and collected < target_frames:
        topic, data, _ = reader.read_next()
        if topic == "/audio_info":
            info = deserialize_message(data, AudioInfo)
            if info.sample_rate:
                sample_rate = int(info.sample_rate)
            if info.channels:
                channels = int(info.channels)
            if info.sample_format:
                sample_format = info.sample_format
            target_frames = max(1, int(round(seconds * sample_rate)))
            continue
        if topic != audio_topic:
            continue

        msg = deserialize_message(data, AudioData)
        frames, _, scale, _, _ = pcm_bytes_to_frames(bytes(msg.data), channels, sample_format)
        if frames.size == 0:
            continue
        if channel_index >= frames.shape[1]:
            raise ValueError(f"channel {channel_index} is outside available channels 0..{frames.shape[1] - 1}")
        chunk = frames[:, channel_index].astype(np.float64) / scale
        remaining = target_frames - collected
        chunks.append(chunk[:remaining])
        collected += min(len(chunk), remaining)

    if not chunks:
        raise RuntimeError(f"no samples found on {audio_topic!r} while estimating f0")

    samples = np.concatenate(chunks)
    samples = samples - np.mean(samples)
    window = np.hanning(len(samples))
    fft_size = 1 << int(math.ceil(math.log2(max(2, len(samples)))))
    spectrum = np.fft.rfft(samples * window, n=fft_size)
    frequencies = np.fft.rfftfreq(fft_size, 1.0 / sample_rate)

    best_frequency = None
    best_magnitude = -1.0
    for candidate_hz in candidates_hz:
        band = np.where(
            (frequencies >= candidate_hz - search_half_width_hz)
            & (frequencies <= candidate_hz + search_half_width_hz)
        )[0]
        if band.size == 0:
            continue
        peak_index = band[int(np.argmax(np.abs(spectrum[band])))]
        peak_magnitude = float(np.abs(spectrum[peak_index]))
        if peak_magnitude > best_magnitude:
            best_magnitude = peak_magnitude
            best_frequency = float(frequencies[peak_index])

    if best_frequency is None:
        raise RuntimeError("failed to estimate f0; no FFT bins matched the candidate bands")
    return best_frequency


class PingerBooster:
    def __init__(
        self,
        sample_rate: int,
        channels: int,
        sample_format: str,
        f0_hz: float,
        lpf_hz: float,
        lpf_order: int,
        gain: float,
        channel: int | None,
        clip: float,
    ) -> None:
        self.sample_rate = sample_rate
        self.channels = channels
        self.sample_format = sample_format
        self.f0_hz = f0_hz
        self.lpf_hz = lpf_hz
        self.lpf_order = lpf_order
        self.gain = gain
        self.channel = channel
        self.clip = clip
        self.phase = 0.0
        self.sos: np.ndarray | None = None
        self.zi: np.ndarray | None = None
        self.configure(sample_rate, channels, sample_format)

    def configure(self, sample_rate: int, channels: int, sample_format: str) -> None:
        if sample_rate <= 0:
            raise ValueError("sample_rate must be positive")
        if channels <= 0:
            raise ValueError("channels must be positive")
        dtype_for_format(sample_format)

        self.sample_rate = sample_rate
        self.channels = channels
        self.sample_format = sample_format
        nyquist = 0.5 * float(sample_rate)
        if not 0.0 < self.lpf_hz < nyquist:
            raise ValueError(f"lpf_hz must be between 0 and Nyquist ({nyquist:g} Hz)")

        self.sos = signal.butter(self.lpf_order, self.lpf_hz / nyquist, btype="lowpass", output="sos")
        self.zi = np.zeros((channels, self.sos.shape[0], 2), dtype=np.complex128)

    def update_from_info(self, info: AudioInfo) -> None:
        sample_rate = int(info.sample_rate) if info.sample_rate else self.sample_rate
        channels = int(info.channels) if info.channels else self.channels
        sample_format = info.sample_format or self.sample_format
        if (
            sample_rate != self.sample_rate
            or channels != self.channels
            or sample_format.upper() != self.sample_format.upper()
        ):
            self.configure(sample_rate, channels, sample_format)

    def channels_to_process(self) -> list[int]:
        if self.channel is None:
            return list(range(self.channels))
        if self.channel < 0 or self.channel >= self.channels:
            raise ValueError(f"channel {self.channel} is outside available channels 0..{self.channels - 1}")
        return [self.channel]

    def process(self, data: bytes) -> bytes:
        frames_i, trailing, scale, min_value, max_value = pcm_bytes_to_frames(
            data,
            self.channels,
            self.sample_format,
        )
        if frames_i.size == 0:
            return data
        if self.sos is None or self.zi is None:
            raise RuntimeError("booster filter state is not initialized")

        frames = frames_i.astype(np.float64) / scale
        frame_count = frames.shape[0]
        phase_step = 2.0 * math.pi * self.f0_hz / float(self.sample_rate)
        phases = self.phase + phase_step * np.arange(frame_count, dtype=np.float64)
        downmix = np.exp(-1j * phases)
        upmix = np.exp(1j * phases)

        boosted = frames.copy()
        for channel_index in self.channels_to_process():
            demodulated = 2.0 * frames[:, channel_index] * downmix
            envelope, self.zi[channel_index] = signal.sosfilt(
                self.sos,
                demodulated,
                zi=self.zi[channel_index],
            )
            tone_estimate = np.real(envelope * upmix)
            boosted[:, channel_index] = frames[:, channel_index] + (self.gain - 1.0) * tone_estimate

        self.phase = float((self.phase + phase_step * frame_count) % (2.0 * math.pi))
        boosted = np.clip(boosted, -self.clip, self.clip)
        boosted_i = np.clip(np.rint(boosted * scale), min_value, max_value).astype(frames_i.dtype)
        return boosted_i.reshape(-1).tobytes() + trailing


def create_boosted_bag(args: argparse.Namespace) -> None:
    input_bag = str(Path(args.input_bag).expanduser())
    output_bag = str(Path(args.output_bag).expanduser())
    if Path(output_bag).exists():
        raise FileExistsError(f"output bag already exists: {output_bag}")

    sample_rate = args.sample_rate
    channels = args.channels
    sample_format = args.sample_format
    f0_hz = args.f0_hz
    if f0_hz is None:
        f0_hz = estimate_f0_hz(
            input_bag=input_bag,
            audio_topic=args.audio_topic,
            sample_rate=sample_rate,
            channels=channels,
            sample_format=sample_format,
            channel_index=args.f0_channel,
            candidates_hz=args.f0_candidates,
            search_half_width_hz=args.f0_search_half_width_hz,
            seconds=args.f0_estimate_seconds,
        )
        print(f"Estimated f0: {f0_hz:.3f} Hz", flush=True)
    else:
        print(f"Using f0: {f0_hz:.3f} Hz", flush=True)

    booster = PingerBooster(
        sample_rate=sample_rate,
        channels=channels,
        sample_format=sample_format,
        f0_hz=f0_hz,
        lpf_hz=args.lpf_hz,
        lpf_order=args.lpf_order,
        gain=args.gain,
        channel=args.channel,
        clip=args.clip,
    )

    reader = open_reader(input_bag)
    writer = SequentialWriter()
    writer.open(
        StorageOptions(uri=output_bag, storage_id="sqlite3"),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )

    existing_topics = reader.get_all_topics_and_types()
    audio_topic_metadata = None
    for topic in existing_topics:
        writer.create_topic(topic_metadata_like(topic))
        if topic.name == args.audio_topic:
            audio_topic_metadata = topic

    if audio_topic_metadata is None:
        raise RuntimeError(f"audio topic {args.audio_topic!r} was not found in input bag")

    writer.create_topic(topic_metadata_like(audio_topic_metadata, args.output_topic))

    total_messages = 0
    boosted_messages = 0
    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        if topic == args.audio_info_topic:
            info = deserialize_message(data, AudioInfo)
            booster.update_from_info(info)

        writer.write(topic, data, timestamp)

        if topic == args.audio_topic:
            audio_msg = deserialize_message(data, AudioData)
            boosted_msg = AudioData()
            boosted_msg.data = array("B", booster.process(bytes(audio_msg.data)))
            writer.write(args.output_topic, serialize_message(boosted_msg), timestamp)
            boosted_messages += 1

        total_messages += 1
        if total_messages % args.progress_interval == 0:
            print(
                f"Processed {total_messages} input messages, wrote {boosted_messages} boosted audio messages",
                flush=True,
            )

    print(
        f"Done. Output: {output_bag} ({boosted_messages} messages on {args.output_topic})",
        flush=True,
    )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Copy a ROS 2 bag and add /audio_boosted by IQ-demodulating around f0, "
            "low-pass filtering the complex envelope, boosting that component, and remodulating."
        )
    )
    parser.add_argument("input_bag", help="input ROS 2 bag directory")
    parser.add_argument("output_bag", help="new output ROS 2 bag directory")
    parser.add_argument("--audio-topic", default="/audio")
    parser.add_argument("--audio-info-topic", default="/audio_info")
    parser.add_argument("--output-topic", default="/audio_boosted")
    parser.add_argument("--f0-hz", type=float, default=None, help="fixed carrier frequency; default estimates from bag")
    parser.add_argument("--f0-candidates", type=parse_frequency_list, default=[21164.0, 27211.0])
    parser.add_argument("--f0-search-half-width-hz", type=float, default=1000.0)
    parser.add_argument("--f0-estimate-seconds", type=float, default=8.0)
    parser.add_argument("--f0-channel", type=int, default=0, help="channel used only for f0 estimation")
    parser.add_argument("--sample-rate", type=int, default=96000)
    parser.add_argument("--channels", type=int, default=2)
    parser.add_argument("--sample-format", default="S32LE", choices=sorted(PCM_FORMATS))
    parser.add_argument("--channel", type=int, default=None, help="boost one channel only; default boosts all channels")
    parser.add_argument("--lpf-hz", type=float, default=500.0, help="complex-envelope low-pass cutoff")
    parser.add_argument("--lpf-order", type=int, default=4)
    parser.add_argument("--gain", type=float, default=3.0, help="target multiplier for the estimated pinger component")
    parser.add_argument("--clip", type=float, default=0.98, help="normalized output clip limit")
    parser.add_argument("--progress-interval", type=int, default=10000)
    return parser


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()
    try:
        create_boosted_bag(args)
    except Exception as exc:  # noqa: BLE001 - keep command-line failures readable.
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
