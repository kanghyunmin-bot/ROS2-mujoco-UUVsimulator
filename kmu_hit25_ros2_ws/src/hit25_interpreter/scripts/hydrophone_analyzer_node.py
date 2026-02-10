#!/usr/bin/env python3
import os
import time
import traceback
import subprocess

import numpy as np
import sounddevice as sd
import rclpy
from rclpy.node import Node

from hit25_interpreter.msg import HydrophoneData


class HydrophoneAnalyzer(Node):
    def __init__(self):
        super().__init__('hydrophone_analyzer_node')

        self.declare_parameter('device_name', 'AMS-22')
        self.declare_parameter('sample_rate', 96000)
        self.declare_parameter('block_size', 4096)
        self.declare_parameter('channels', 1)
        self.declare_parameter('freq_range_min', 10000)
        self.declare_parameter('freq_range_max', 30000)
        self.declare_parameter('target_frequency', 27000.0)
        self.declare_parameter('freq_tolerance', 150.0)
        self.declare_parameter('db_threshold', -40.0)
        self.declare_parameter('db_for_max_confidence', -20.0)
        self.declare_parameter('stop_pulseaudio_on_start', True)
        self.declare_parameter('fallback_to_default', True)
        self.declare_parameter('device_retry_interval', 2.0)
        self.declare_parameter('device_retry_log_interval', 5.0)
        self.declare_parameter('device_retry_max', 3)

        self.device_name = str(self.get_parameter('device_name').value)
        self.sample_rate = int(self.get_parameter('sample_rate').value)
        self.block_size = int(self.get_parameter('block_size').value)
        self.channels = int(self.get_parameter('channels').value)
        self.freq_range_min = float(self.get_parameter('freq_range_min').value)
        self.freq_range_max = float(self.get_parameter('freq_range_max').value)
        self.target_frequency = float(self.get_parameter('target_frequency').value)
        self.freq_tolerance = float(self.get_parameter('freq_tolerance').value)
        self.db_threshold = float(self.get_parameter('db_threshold').value)
        self.db_for_max_confidence = float(self.get_parameter('db_for_max_confidence').value)
        self.stop_pulseaudio_on_start = bool(self.get_parameter('stop_pulseaudio_on_start').value)
        self.fallback_to_default = bool(self.get_parameter('fallback_to_default').value)
        self.device_retry_interval = float(self.get_parameter('device_retry_interval').value)
        self.device_retry_log_interval = float(self.get_parameter('device_retry_log_interval').value)
        self.device_retry_max = int(self.get_parameter('device_retry_max').value)

        self.dtype = np.int16
        self.pub = self.create_publisher(HydrophoneData, '/hydrophone/data', 10)

        self._last_device_log_time = 0.0
        self._device_retry_count = 0

        if self.stop_pulseaudio_on_start:
            self._stop_pulseaudio()

        self.device_id = self.find_device_id(self.device_name, log=False)
        self.get_logger().info(f"Target Frequency set to: {self.target_frequency} Hz")

    def _stop_pulseaudio(self):
        try:
            subprocess.run(["pulseaudio", "-k"], check=False)
            time.sleep(2.0)
        except Exception as exc:
            self.get_logger().warn(f"Failed to stop PulseAudio: {exc}")

    def _list_input_devices(self):
        devices = []
        for i, device in enumerate(sd.query_devices()):
            if device.get('max_input_channels', 0) > 0:
                devices.append((i, device.get('name', '')))
        return devices

    def _pick_default_input_device(self):
        try:
            default_input = sd.default.device[0]
        except Exception:
            default_input = None

        if default_input is not None and default_input >= 0:
            try:
                dev = sd.query_devices(int(default_input))
                if dev.get('max_input_channels', 0) > 0:
                    return int(default_input)
            except Exception:
                pass

        devices = self._list_input_devices()
        if devices:
            return int(devices[0][0])
        return None

    def find_device_id(self, name, log=True):
        try:
            for i, device_name in self._list_input_devices():
                if name in device_name:
                    self.get_logger().info(f"Found device: ID {i}, name '{device_name}'")
                    return i

            if log:
                devices = self._list_input_devices()
                self.get_logger().warn(f"Target '{name}' NOT found. Currently available devices:")
                for i, device_name in devices:
                    self.get_logger().warn(f"  [{i}] {device_name}")

        except Exception as exc:
            self.get_logger().warn(f"Error querying devices: {exc}")
        return None

    def audio_callback(self, indata, frames, callback_time, status):
        if status:
            return

        signal = indata.flatten()
        fft_magnitude = np.abs(np.fft.rfft(signal))
        fft_freqs = np.fft.rfftfreq(len(signal), 1.0 / self.sample_rate)

        mask = (fft_freqs >= self.freq_range_min) & (fft_freqs <= self.freq_range_max)
        freqs_in_range = fft_freqs[mask]
        magnitude_in_range = fft_magnitude[mask]

        if len(magnitude_in_range) == 0:
            return

        dominant_index = int(np.argmax(magnitude_in_range))
        dominant_freq = float(freqs_in_range[dominant_index])
        dominant_magnitude = float(magnitude_in_range[dominant_index])
        epsilon = 1e-12
        decibels = 20.0 * np.log10(dominant_magnitude / (2**15) + epsilon)

        detected_target_freq = 0.0
        if decibels > self.db_threshold:
            if abs(dominant_freq - self.target_frequency) <= self.freq_tolerance:
                detected_target_freq = self.target_frequency

        confidence = 0.0
        if decibels >= self.db_threshold:
            if self.db_for_max_confidence <= self.db_threshold:
                confidence = 1.0
            else:
                confidence = (decibels - self.db_threshold) / (self.db_for_max_confidence - self.db_threshold)
                confidence = max(0.0, min(1.0, confidence))

        msg = HydrophoneData()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "hydrophone_link"
        msg.frequency = dominant_freq
        msg.decibels = float(decibels)
        msg.confidence = float(confidence)
        msg.target_frequency = float(detected_target_freq)
        self.pub.publish(msg)

    def start(self):
        if self.device_id is None:
            self.device_id = self.find_device_id(self.device_name)

        while rclpy.ok():
            if self.device_id is None:
                now = time.time()
                if now - self._last_device_log_time >= self.device_retry_log_interval:
                    self._last_device_log_time = now
                    self.get_logger().warn(f"Device '{self.device_name}' not found yet. Retrying...")
                    self.device_id = self.find_device_id(self.device_name, log=True)

                self._device_retry_count += 1
                if self.fallback_to_default and self._device_retry_count >= self.device_retry_max:
                    default_id = self._pick_default_input_device()
                    if default_id is not None:
                        self.get_logger().warn(
                            f"Falling back to default input device ID {default_id}."
                        )
                        self.device_id = default_id
                        self._device_retry_count = 0
                        continue

                time.sleep(self.device_retry_interval)
                continue

            try:
                self.get_logger().info(f"Opening audio stream on device ID {self.device_id}...")
                stream = sd.InputStream(
                    device=self.device_id,
                    channels=self.channels,
                    samplerate=self.sample_rate,
                    blocksize=self.block_size,
                    dtype=self.dtype,
                    callback=self.audio_callback,
                )
                stream.start()
                self.get_logger().info("Audio stream started successfully!")
                rclpy.spin(self)
                stream.stop()
                stream.close()
                break

            except Exception as exc:
                self.get_logger().error(f"Stream Error: {exc}\n{traceback.format_exc()}")
                self.get_logger().warn("Device unavailable. Retrying in 2 seconds...")
                self.device_id = None
                self._device_retry_count = 0
                time.sleep(2.0)


def main():
    rclpy.init()
    node = None
    try:
        node = HydrophoneAnalyzer()
        node.start()
    except Exception:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
