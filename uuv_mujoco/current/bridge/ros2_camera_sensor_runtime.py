"""ROS-boundary timing runtime for modeled underwater camera frames."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from sim.sensors.timing_transport import (
    CaptureScheduleConfig,
    DeviceClockConfig,
    LatencyConfig,
    OverflowPolicy,
    SensorCapture,
    SensorTimingTransportConfig,
    SensorTimingTransportRuntime,
    SensorTransportConfig,
)

from .underwater_camera_sensor_model import (
    CameraCalibration,
    CameraModelDiagnostics,
    UnderwaterCameraProfile,
    UnderwaterCameraSensorModel,
)


@dataclass(frozen=True, slots=True)
class RenderedCameraFrame:
    """One ideal renderer output associated with its simulation capture time."""

    rgb: np.ndarray
    capture_time_s: float

    def __post_init__(self) -> None:
        if not isinstance(self.rgb, np.ndarray):
            raise TypeError("rendered camera rgb must be a numpy array")
        if self.rgb.dtype != np.uint8 or self.rgb.ndim != 3 or self.rgb.shape[2] != 3:
            raise ValueError("rendered camera rgb must be a uint8 [height, width, 3] array")
        if not math.isfinite(float(self.capture_time_s)) or float(self.capture_time_s) < 0.0:
            raise ValueError("rendered camera capture_time_s must be finite and non-negative")


@dataclass(frozen=True, slots=True)
class ModeledCameraFrame:
    """Post-processed image payload before simulated link delivery."""

    rgb: np.ndarray
    diagnostics: CameraModelDiagnostics


@dataclass(frozen=True, slots=True)
class CameraFrameDelivery:
    """One modeled frame delivered to the unchanged ROS publisher boundary."""

    sequence: int
    capture_time_s: float
    device_time_s: float
    transmission_time_s: float
    arrival_time_s: float
    rgb: np.ndarray
    diagnostics: CameraModelDiagnostics


@dataclass(frozen=True, slots=True)
class CameraRuntimeStats:
    """Camera-specific timing counters plus the generic bounded-link counters."""

    offered_frames: int
    accepted_captures: int
    duplicate_or_stale_frames: int
    rate_limited_frames: int
    delivered_frames: int
    probabilistic_drops: int
    queue_overflow_drops: int
    pending_frames: int


class CameraSensorRuntime:
    """Apply the image model and bounded deterministic transport to rendered frames."""

    def __init__(
        self,
        profile: UnderwaterCameraProfile,
        *,
        camera_name: str,
        calibration: CameraCalibration,
        frame_rate_hz: float,
        seed: int | None = None,
        dropout_probability: float | None = None,
        processing_latency_mean_s: float | None = None,
        processing_latency_jitter_s: float | None = None,
        transport_latency_mean_s: float | None = None,
        transport_latency_jitter_s: float | None = None,
    ) -> None:
        frame_rate_hz = float(frame_rate_hz)
        if not math.isfinite(frame_rate_hz) or frame_rate_hz <= 0.0:
            raise ValueError("camera frame_rate_hz must be finite and positive")
        self.profile = profile
        self.camera_name = str(camera_name)
        self.calibration = calibration
        self.frame_rate_hz = frame_rate_hz
        self.frame_period_s = 1.0 / frame_rate_hz
        selected_seed = profile.seed if seed is None else seed
        if isinstance(selected_seed, bool) or not isinstance(selected_seed, int):
            raise TypeError("camera runtime seed must be an integer")
        camera_seed_salt = 0x13579 if self.camera_name == "stereo_left" else 0x24680
        self.base_seed = int(selected_seed)
        self.seed = self.base_seed ^ camera_seed_salt
        timing = profile.timing

        selected_dropout = (
            timing.dropout_probability
            if dropout_probability is None
            else float(dropout_probability)
        )
        processing_mean = (
            timing.processing_latency_mean_s
            if processing_latency_mean_s is None
            else float(processing_latency_mean_s)
        )
        processing_jitter = (
            timing.processing_latency_jitter_s
            if processing_latency_jitter_s is None
            else float(processing_latency_jitter_s)
        )
        transport_mean = (
            timing.transport_latency_mean_s
            if transport_latency_mean_s is None
            else float(transport_latency_mean_s)
        )
        transport_jitter = (
            timing.transport_latency_jitter_s
            if transport_latency_jitter_s is None
            else float(transport_latency_jitter_s)
        )
        processing_max = timing.processing_latency_max_s
        if processing_max is not None:
            processing_max = max(float(processing_max), processing_mean)
        transport_max = timing.transport_latency_max_s
        if transport_max is not None:
            transport_max = max(float(transport_max), transport_mean)

        self.model = UnderwaterCameraSensorModel(
            profile,
            calibration=calibration,
            camera_name=self.camera_name,
            enabled=True,
            seed=self.base_seed,
        )
        self.transport = SensorTimingTransportRuntime[ModeledCameraFrame](
            SensorTimingTransportConfig(
                schedule=CaptureScheduleConfig(rate_hz=frame_rate_hz),
                clock=DeviceClockConfig(
                    offset_s=timing.device_clock_offset_s,
                    drift_ppm=timing.device_clock_drift_ppm,
                ),
                transport=SensorTransportConfig(
                    processing_latency=LatencyConfig(
                        mean_s=processing_mean,
                        jitter_std_s=processing_jitter,
                        min_s=0.0,
                        max_s=processing_max,
                    ),
                    transport_latency=LatencyConfig(
                        mean_s=transport_mean,
                        jitter_std_s=transport_jitter,
                        min_s=0.0,
                        max_s=transport_max,
                    ),
                    dropout_probability=selected_dropout,
                    queue_capacity=timing.queue_capacity,
                    overflow_policy=OverflowPolicy(timing.overflow_policy),
                ),
                seed=self.seed,
            )
        )
        self._closed = False
        self._last_reference_time_s = -math.inf
        self._last_source_capture_time_s = -math.inf
        self._next_allowed_capture_time_s = -math.inf
        self._next_sequence = 0
        self._offered_frames = 0
        self._accepted_captures = 0
        self._duplicate_or_stale_frames = 0
        self._rate_limited_frames = 0

    @property
    def stats(self) -> CameraRuntimeStats:
        """Return an immutable runtime and bounded-transport counter snapshot."""

        transport = self.transport.stats
        return CameraRuntimeStats(
            offered_frames=self._offered_frames,
            accepted_captures=self._accepted_captures,
            duplicate_or_stale_frames=self._duplicate_or_stale_frames,
            rate_limited_frames=self._rate_limited_frames,
            delivered_frames=transport.delivered,
            probabilistic_drops=transport.probabilistic_drops,
            queue_overflow_drops=transport.queue_overflow_drops,
            pending_frames=transport.pending,
        )

    def advance(
        self,
        reference_time_s: float,
        rendered: RenderedCameraFrame | None,
    ) -> tuple[CameraFrameDelivery, ...]:
        """Offer at most one new render and drain every frame whose arrival is due."""

        if self._closed:
            return ()
        now_s = float(reference_time_s)
        if not math.isfinite(now_s) or now_s < 0.0:
            raise ValueError("camera reference_time_s must be finite and non-negative")
        epsilon_s = self.transport.config.schedule.epsilon_s
        if now_s + epsilon_s < self._last_reference_time_s:
            self.reset()
        self._last_reference_time_s = now_s

        if rendered is not None:
            self._offer(rendered, now_s, epsilon_s)
        packets = self.transport.drain_arrived(now_s)
        return tuple(
            CameraFrameDelivery(
                sequence=packet.sequence,
                capture_time_s=packet.capture_time_s,
                device_time_s=packet.device_time_s,
                transmission_time_s=packet.transmission_time_s,
                arrival_time_s=packet.arrival_time_s,
                rgb=packet.payload.rgb,
                diagnostics=packet.payload.diagnostics,
            )
            for packet in packets
        )

    def reset(self) -> None:
        """Reset timing, RNG streams, sequence state, and all pending images."""

        self.transport.reset(seed=self.seed)
        self._last_reference_time_s = -math.inf
        self._last_source_capture_time_s = -math.inf
        self._next_allowed_capture_time_s = -math.inf
        self._next_sequence = 0
        self._offered_frames = 0
        self._accepted_captures = 0
        self._duplicate_or_stale_frames = 0
        self._rate_limited_frames = 0

    def close(self) -> None:
        """Drop bounded queued images and release full-frame model caches."""

        if self._closed:
            return
        self.transport.discard_pending()
        self.model.close()
        self._closed = True

    def _offer(
        self,
        rendered: RenderedCameraFrame,
        now_s: float,
        epsilon_s: float,
    ) -> None:
        if not isinstance(rendered, RenderedCameraFrame):
            raise TypeError("rendered must be a RenderedCameraFrame or None")
        capture_time_s = float(rendered.capture_time_s)
        if capture_time_s > now_s + epsilon_s:
            raise ValueError("rendered camera capture time cannot be in the future")
        self._offered_frames += 1
        if capture_time_s <= self._last_source_capture_time_s + epsilon_s:
            self._duplicate_or_stale_frames += 1
            return
        self._last_source_capture_time_s = capture_time_s
        if capture_time_s + epsilon_s < self._next_allowed_capture_time_s:
            self._rate_limited_frames += 1
            return

        sequence = self._next_sequence
        self._next_sequence += 1
        self._next_allowed_capture_time_s = capture_time_s + self.frame_period_s
        processed, diagnostics = self.model.process(rendered.rgb, sequence=sequence)
        capture = SensorCapture(
            sequence=sequence,
            capture_time_s=capture_time_s,
            device_time_s=self.transport.config.clock.timestamp(capture_time_s),
        )
        self.transport.submit(
            capture,
            ModeledCameraFrame(rgb=processed, diagnostics=diagnostics),
        )
        self._accepted_captures += 1


__all__ = [
    "CameraFrameDelivery",
    "CameraRuntimeStats",
    "CameraSensorRuntime",
    "ModeledCameraFrame",
    "RenderedCameraFrame",
]
