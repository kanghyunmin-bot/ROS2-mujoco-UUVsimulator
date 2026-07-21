#!/usr/bin/env python3
"""Verify a 21 kHz carrier present only on channel 1 is detected.

The real selector must combine per-channel *power*, not PCM amplitude.  This
test leaves channel 0 as noise and puts the carrier on channel 1, then checks
the narrowed-band candidate and persistence metadata.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import random
import struct
import subprocess
import time

os.environ.setdefault("ROS_DOMAIN_ID", "193")

import rclpy
from audio_common_msgs.msg import AudioData
from rclpy.node import Node
from std_msgs.msg import String


SAMPLE_RATE = 48_000
FREQUENCY_HZ = 21_046.875  # exact 4096-point FFT bin at 48 kHz


class AudioProbe(Node):
    def __init__(self) -> None:
        super().__init__("frequency_selector_runtime_probe")
        self.audio_pub = self.create_publisher(AudioData, "/test/frequency/audio", 20)
        self.candidates: dict = {}
        self.create_subscription(
            String, "/test/frequency/candidates", self._on_candidates, 10
        )
        self.sample_cursor = 0
        self.random = random.Random(260719)

    def _on_candidates(self, message: String) -> None:
        try:
            value = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if isinstance(value, dict):
            self.candidates = value

    def publish_audio(self, frames: int = 2400) -> None:
        payload = bytearray()
        for offset in range(frames):
            sample = self.sample_cursor + offset
            # Channel 0 deliberately has no carrier. Channel 1 has a weak but
            # persistent 21 kHz carrier with independent broadband noise.
            noise0 = self.random.gauss(0.0, 0.020)
            noise1 = self.random.gauss(0.0, 0.020)
            tone = 0.0030 * math.sin(2.0 * math.pi * FREQUENCY_HZ * sample / SAMPLE_RATE)
            for value in (noise0, noise1 + tone):
                integer = max(-(2**31), min(2**31 - 1, int(value * (2**31 - 1))))
                payload.extend(struct.pack("<i", integer))
        self.sample_cursor += frames
        message = AudioData()
        message.data = list(payload)
        self.audio_pub.publish(message)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--selector", required=True)
    args = parser.parse_args()
    process = subprocess.Popen([
        args.selector,
        "--ros-args",
        "-p", "audio_topic:=/test/frequency/audio",
        "-p", "candidate_topic:=/test/frequency/candidates",
        "-p", "selected_frequency_topic:=/test/frequency/selected",
        "-p", "manual_selection_topic:=/test/frequency/manual",
        "-p", f"sample_rate:={SAMPLE_RATE}",
        "-p", "channels:=2",
        "-p", "combine_channels:=true",
        "-p", "monitor_s:=1.5",
        "-p", "min_frequency_hz:=19000.0",
        "-p", "max_frequency_hz:=22000.0",
        "-p", "fft_size:=4096",
        "-p", "fft_hop_size:=2048",
        "-p", "min_snr_db:=6.0",
        "-p", "min_peak_prominence_db:=3.0",
        "-p", "persistent_min_ratio:=0.20",
        "-p", "stdin_selection_enabled:=false",
    ])
    rclpy.init()
    probe = AudioProbe()
    try:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not probe.candidates:
            probe.publish_audio()
            rclpy.spin_once(probe, timeout_sec=0.04)
        ranked = probe.candidates.get("candidates", [])
        if not ranked:
            raise AssertionError(f"selector produced no candidates: {probe.candidates}")
        candidate = ranked[0]
        detected = float(candidate["frequency_hz"])
        if abs(detected - FREQUENCY_HZ) > 15.0:
            raise AssertionError(f"expected {FREQUENCY_HZ} Hz, got {detected} Hz: {ranked}")
        if not bool(candidate.get("qualified", False)):
            raise AssertionError(f"21 kHz candidate was not qualified: {candidate}")
        if float(candidate.get("persistence_ratio", 0.0)) <= 0.0:
            raise AssertionError(f"candidate has no persistence evidence: {candidate}")
        print("frequency_selector_runtime=PASS channel=1 band=19-22kHz")
        return 0
    finally:
        probe.destroy_node()
        rclpy.shutdown()
        process.terminate()
        try:
            process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2.0)


if __name__ == "__main__":
    raise SystemExit(main())
