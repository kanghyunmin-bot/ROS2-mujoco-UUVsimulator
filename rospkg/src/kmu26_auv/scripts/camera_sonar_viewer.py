#!/usr/bin/env python3
# Copyright (c) 2026, KMU Underwater Robot Team.
# SPDX-License-Identifier: MIT

"""Show the vehicle camera and accumulated Ping360 scan without RViz."""

from __future__ import annotations

import json
import signal
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Any

import numpy as np
import rclpy
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QCloseEvent, QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QFrame,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


_SONAR_PALETTE_POINTS = np.asarray(
    [
        (0, 0, 0, 0),
        (18, 8, 3, 0),
        (48, 35, 11, 0),
        (92, 105, 38, 0),
        (148, 210, 88, 4),
        (210, 255, 178, 46),
        (255, 255, 250, 205),
    ],
    dtype=np.float32,
)


def _build_sonar_lut() -> np.ndarray:
    levels = np.arange(256, dtype=np.float32)
    anchors = _SONAR_PALETTE_POINTS[:, 0]
    channels = [
        np.interp(levels, anchors, _SONAR_PALETTE_POINTS[:, channel])
        for channel in range(1, 4)
    ]
    return np.stack(channels, axis=1).astype(np.uint8)


SONAR_LUT = _build_sonar_lut()


def _packed_image(message: Any, channels: int) -> np.ndarray:
    """Return a tightly packed uint8 image while respecting ROS row stride."""
    width = int(message.width)
    height = int(message.height)
    if width <= 0 or height <= 0:
        raise ValueError(f"invalid image dimensions: {width}x{height}")
    packed_step = width * channels
    step = int(message.step) if int(message.step) > 0 else packed_step
    if step < packed_step:
        raise ValueError(f"image step {step} is smaller than packed row {packed_step}")
    raw = np.frombuffer(message.data, dtype=np.uint8)
    required = height * step
    if raw.size < required:
        raise ValueError(f"image payload has {raw.size} bytes, expected at least {required}")
    rows = raw[:required].reshape(height, step)
    packed = rows[:, :packed_step]
    if channels == 1:
        return packed.reshape(height, width)
    return packed.reshape(height, width, channels)


def image_message_to_rgb(message: Any, *, sonar_palette: bool = False) -> np.ndarray:
    """Convert supported ROS image encodings to a contiguous RGB array."""
    encoding = str(message.encoding).strip().lower()
    if encoding in {"mono8", "8uc1"}:
        mono = _packed_image(message, 1)
        if sonar_palette:
            return np.ascontiguousarray(SONAR_LUT[mono])
        return np.ascontiguousarray(np.repeat(mono[:, :, None], 3, axis=2))
    if encoding in {"rgb8", "8uc3"}:
        return np.ascontiguousarray(_packed_image(message, 3))
    if encoding == "bgr8":
        return np.ascontiguousarray(_packed_image(message, 3)[:, :, ::-1])
    if encoding == "rgba8":
        return np.ascontiguousarray(_packed_image(message, 4)[:, :, :3])
    if encoding == "bgra8":
        return np.ascontiguousarray(_packed_image(message, 4)[:, :, 2::-1])
    raise ValueError(f"unsupported image encoding: {message.encoding!r}")


def _rgb_to_qimage(rgb: np.ndarray) -> QImage:
    height, width, _ = rgb.shape
    image = QImage(
        rgb.data,
        width,
        height,
        int(rgb.strides[0]),
        QImage.Format_RGB888,
    )
    return image.copy()


@dataclass(frozen=True)
class FrameSnapshot:
    """One latest-only frame plus host-side health information."""

    sequence: int
    message: Image
    rate_hz: float
    age_s: float


class LatestFrame:
    """Thread-safe latest-only image slot that never queues stale video."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._sequence = 0
        self._message: Image | None = None
        self._arrival_s = 0.0
        self._arrivals: deque[float] = deque(maxlen=180)

    def push(self, message: Image) -> None:
        now = time.monotonic()
        with self._lock:
            self._sequence += 1
            self._message = message
            self._arrival_s = now
            self._arrivals.append(now)

    def snapshot(self) -> FrameSnapshot | None:
        now = time.monotonic()
        with self._lock:
            if self._message is None:
                return None
            rate_hz = 0.0
            if len(self._arrivals) >= 2:
                elapsed = self._arrivals[-1] - self._arrivals[0]
                if elapsed > 1.0e-6:
                    rate_hz = (len(self._arrivals) - 1) / elapsed
            return FrameSnapshot(
                sequence=self._sequence,
                message=self._message,
                rate_hz=rate_hz,
                age_s=max(0.0, now - self._arrival_s),
            )


class CameraSonarNode(Node):
    """Receive only the image surfaces needed by the two-panel viewer."""

    def __init__(self) -> None:
        super().__init__("camera_sonar_viewer")
        self.camera_topic = str(
            self.declare_parameter(
                "camera_topic", "/imx219/camera0/image_raw"
            ).value
        )
        self.sonar_topic = str(
            self.declare_parameter("sonar_topic", "/ping360/scan_image").value
        )
        self.sonar_status_topic = str(
            self.declare_parameter("sonar_status_topic", "/ping360/status").value
        )
        self.camera = LatestFrame()
        self.sonar = LatestFrame()
        self._status_lock = threading.Lock()
        self._sonar_status: dict[str, Any] = {}

        self.create_subscription(
            Image,
            self.camera_topic,
            self.camera.push,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.sonar_topic,
            self.sonar.push,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            self.sonar_status_topic,
            self._on_sonar_status,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"camera={self.camera_topic}, sonar={self.sonar_topic}"
        )

    def _on_sonar_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        if not isinstance(payload, dict):
            return
        with self._status_lock:
            self._sonar_status = payload

    def sonar_status(self) -> dict[str, Any]:
        with self._status_lock:
            return dict(self._sonar_status)


class ImagePanel(QFrame):
    """Responsive image panel with a compact sensor-health readout."""

    def __init__(self, title: str, topic: str, placeholder: str) -> None:
        super().__init__()
        self._image: QImage | None = None
        self.setObjectName("sensorPanel")
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.title = QLabel(title)
        self.title.setObjectName("panelTitle")
        self.health = QLabel("WAITING FOR SENSOR")
        self.health.setObjectName("healthWaiting")
        self.health.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        title_row = QHBoxLayout()
        title_row.addWidget(self.title)
        title_row.addStretch(1)
        title_row.addWidget(self.health)

        self.image = QLabel(placeholder)
        self.image.setObjectName("imageSurface")
        self.image.setAlignment(Qt.AlignCenter)
        self.image.setMinimumSize(360, 300)
        self.image.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.details = QLabel("No frames received")
        self.details.setObjectName("sensorDetails")
        self.topic = QLabel(topic)
        self.topic.setObjectName("topicLabel")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(18, 16, 18, 14)
        layout.setSpacing(10)
        layout.addLayout(title_row)
        layout.addWidget(self.image, 1)
        layout.addWidget(self.details)
        layout.addWidget(self.topic)

    def set_frame(self, image: QImage) -> None:
        self._image = image
        self._redraw()

    def set_status(self, state: str, details: str) -> None:
        state_upper = state.upper()
        self.health.setText(state_upper)
        if state_upper == "LIVE":
            self.health.setObjectName("healthLive")
        elif state_upper == "STALE":
            self.health.setObjectName("healthStale")
        else:
            self.health.setObjectName("healthWaiting")
        self.health.style().unpolish(self.health)
        self.health.style().polish(self.health)
        self.details.setText(details)

    def resizeEvent(self, event: Any) -> None:
        super().resizeEvent(event)
        self._redraw()

    def _redraw(self) -> None:
        if self._image is None:
            return
        pixmap = QPixmap.fromImage(self._image)
        self.image.setPixmap(
            pixmap.scaled(
                self.image.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation,
            )
        )


class CameraSonarWindow(QMainWindow):
    """Single window containing no map or telemetry graph surfaces."""

    def __init__(self, node: CameraSonarNode) -> None:
        super().__init__()
        self._node = node
        self._camera_sequence = 0
        self._sonar_sequence = 0
        self.setWindowTitle("UUV Camera + Ping360")
        self.resize(1580, 820)
        self.setMinimumSize(1000, 620)

        heading = QLabel("UUV SENSOR VIEW")
        heading.setObjectName("windowTitle")
        subtitle = QLabel(
            "IMX219 underwater camera  /  accumulated Ping360 mechanical scan"
        )
        subtitle.setObjectName("windowSubtitle")

        self.camera_panel = ImagePanel(
            "FORWARD CAMERA",
            node.camera_topic,
            "Waiting for IMX219 image...",
        )
        self.sonar_panel = ImagePanel(
            "PING360 SONAR",
            node.sonar_topic,
            "Waiting for accumulated sonar scan...",
        )

        panels = QHBoxLayout()
        panels.setSpacing(14)
        panels.addWidget(self.camera_panel, 1)
        panels.addWidget(self.sonar_panel, 1)

        content = QWidget()
        layout = QVBoxLayout(content)
        layout.setContentsMargins(22, 18, 22, 20)
        layout.setSpacing(12)
        layout.addWidget(heading)
        layout.addWidget(subtitle)
        layout.addLayout(panels, 1)
        self.setCentralWidget(content)
        self.setStyleSheet(_STYLE)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(33)

    def closeEvent(self, event: QCloseEvent) -> None:
        self._timer.stop()
        event.accept()

    def _refresh(self) -> None:
        self._refresh_camera(self._node.camera.snapshot())
        self._refresh_sonar(self._node.sonar.snapshot())

    def _refresh_camera(self, snapshot: FrameSnapshot | None) -> None:
        if snapshot is None:
            self.camera_panel.set_status("WAITING", "No camera frames received")
            return
        if snapshot.sequence != self._camera_sequence:
            self._camera_sequence = snapshot.sequence
            try:
                rgb = image_message_to_rgb(snapshot.message)
                self.camera_panel.set_frame(_rgb_to_qimage(rgb))
            except (ValueError, TypeError) as error:
                self.camera_panel.set_status("ERROR", str(error))
                return
        state = "LIVE" if snapshot.age_s < 1.0 else "STALE"
        self.camera_panel.set_status(
            state,
            (
                f"{snapshot.message.width} x {snapshot.message.height}  |  "
                f"{snapshot.rate_hz:4.1f} FPS  |  {snapshot.message.encoding}"
            ),
        )

    def _refresh_sonar(self, snapshot: FrameSnapshot | None) -> None:
        if snapshot is None:
            self.sonar_panel.set_status("WAITING", "No Ping360 scan frames received")
            return
        if snapshot.sequence != self._sonar_sequence:
            self._sonar_sequence = snapshot.sequence
            try:
                rgb = image_message_to_rgb(snapshot.message, sonar_palette=True)
                self.sonar_panel.set_frame(_rgb_to_qimage(rgb))
            except (ValueError, TypeError) as error:
                self.sonar_panel.set_status("ERROR", str(error))
                return
        state = "LIVE" if snapshot.age_s < 1.5 else "STALE"
        status = self._node.sonar_status()
        settings = status.get("settings", {})
        effective_range = settings.get("effective_range_m")
        frequency = settings.get("transmit_frequency_khz")
        gain = settings.get("gain_setting")
        angle = status.get("angle_deg")
        fields = [
            f"{snapshot.message.width} x {snapshot.message.height}",
            f"{snapshot.rate_hz:4.1f} FPS",
        ]
        if isinstance(effective_range, (int, float)):
            fields.append(f"RANGE {float(effective_range):.1f} m")
        if isinstance(frequency, (int, float)):
            fields.append(f"{int(frequency)} kHz")
        if isinstance(gain, (int, float)):
            fields.append(f"GAIN {int(gain)}")
        if isinstance(angle, (int, float)):
            fields.append(f"BEARING {float(angle):.1f} deg")
        self.sonar_panel.set_status(state, "  |  ".join(fields))


_STYLE = """
QMainWindow, QWidget {
    background: #080b0e;
    color: #e8edf1;
    font-family: "DejaVu Sans";
}
QLabel#windowTitle {
    color: #f4f6f7;
    font-size: 23px;
    font-weight: 700;
    letter-spacing: 2px;
}
QLabel#windowSubtitle {
    color: #77838c;
    font-size: 12px;
    padding-bottom: 4px;
}
QFrame#sensorPanel {
    background: #10151a;
    border: 1px solid #26313a;
    border-radius: 8px;
}
QLabel#panelTitle {
    color: #f0f3f5;
    font-size: 15px;
    font-weight: 700;
}
QLabel#imageSurface {
    background: #000000;
    color: #4c5962;
    border: 1px solid #202a31;
    font-size: 14px;
}
QLabel#sensorDetails {
    color: #bec7cd;
    font-family: "DejaVu Sans Mono";
    font-size: 12px;
}
QLabel#topicLabel {
    color: #65727b;
    font-family: "DejaVu Sans Mono";
    font-size: 11px;
}
QLabel#healthLive {
    color: #65d39b;
    font-size: 11px;
    font-weight: 700;
}
QLabel#healthStale {
    color: #f1b65c;
    font-size: 11px;
    font-weight: 700;
}
QLabel#healthWaiting {
    color: #7d8991;
    font-size: 11px;
    font-weight: 700;
}
"""


def _spin(executor: MultiThreadedExecutor) -> None:
    try:
        executor.spin()
    except (ExternalShutdownException, KeyboardInterrupt):
        pass


def main(args: list[str] | None = None) -> int:
    """Run the ROS subscriber and Qt display in one process."""
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    application = QApplication([sys.argv[0]])

    rclpy.init(args=args)
    node = CameraSonarNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=_spin, args=(executor,), daemon=True)
    spin_thread.start()

    window = CameraSonarWindow(node)
    signal.signal(signal.SIGINT, lambda *_: window.close())
    signal.signal(signal.SIGTERM, lambda *_: window.close())
    window.showMaximized()
    try:
        return int(application.exec())
    finally:
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    raise SystemExit(main())
