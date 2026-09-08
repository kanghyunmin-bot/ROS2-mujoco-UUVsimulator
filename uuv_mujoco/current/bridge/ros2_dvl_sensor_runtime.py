"""A50 sensor-model timing and dead-reckoning runtime for the ROS2 bridge.

This module is the adapter between :class:`A50SensorModel` and the reusable,
ROS-independent sensor timing runtime.  It intentionally does not publish ROS
messages.  Callers receive completed deliveries only after their simulated
processing and transport delay has elapsed.

Environment overrides use the ``ROS2_UUV_DVL_SENSOR_*`` prefix.  Supported
suffixes are ``ENABLE``, ``CONFIG_PATH``, ``SEED``, ``RATE_HZ``,
``CLOCK_OFFSET_S``, ``CLOCK_DRIFT_PPM``, processing/transport latency
``{MEAN,JITTER_STD,MIN,MAX}_S``, ``PACKET_DROPOUT_PROBABILITY``,
``POSITION_RATE_HZ``, ``POSITION_PACKET_DROPOUT_PROBABILITY``,
``QUEUE_CAPACITY``, and ``QUEUE_OVERFLOW_POLICY``.
"""

from __future__ import annotations

import json
import math
import os
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, fields, replace
from pathlib import Path
from typing import Any

import numpy as np

from sim.sensors import (
    CaptureScheduleConfig,
    DeviceClockConfig,
    LatencyConfig,
    OverflowPolicy,
    SensorCapture,
    SensorPacket,
    SensorTimingTransportConfig,
    SensorTimingTransportRuntime,
    SensorTransportConfig,
)

from .dvl_a50_sensor_model import A50SensorConfig, A50SensorModel, A50SensorSample
Vector3 = tuple[float, float, float]

DEFAULT_A50_SENSOR_CONFIG_PATH = (
    Path(__file__).resolve().parents[1]
    / "config"
    / "sensor_models"
    / "a50_uncalibrated_prior.json"
)
EXPECTED_SCHEMA = "uuv_mujoco.sensor_model.a50.v1"
EXPECTED_CALIBRATION_STATUS = "unvalidated_prior"
_ENV_PREFIX = "ROS2_UUV_DVL_SENSOR_"
_ATTITUDE_SEED_SALT = 0x6A09E667F3BCC909
_ATTITUDE_WALK_SEED_SALT = 0x3C6EF372FE94F82B
_POSITION_TIMING_SEED_SALT = 0xBB67AE8584CAA73B
_MODELED_ATTITUDE_SOURCE = "modeled_body_gyro"


@dataclass(frozen=True, slots=True)
class DvlSensorDelivery:
    """One arrived A50 velocity-and-transducer observation."""

    sample: A50SensorSample
    capture_time_s: float
    arrival_time_s: float
    report_period_s: float


@dataclass(frozen=True, slots=True)
class DvlPositionDelivery:
    """One independently captured and transported A50 DR report."""

    capture_time_s: float
    arrival_time_s: float
    report_time_s: float
    position_local_frd_m: Vector3
    position_std_m: float
    attitude_rpy_deg: Vector3
    generation: int


@dataclass(frozen=True, slots=True)
class _DvlCapturePayload:
    sample: A50SensorSample
    angular_velocity_dvl_frd_rad_s: Vector3 | None


@dataclass(frozen=True, slots=True)
class _DvlPositionPayload:
    position_local_frd_m: Vector3
    position_std_m: float
    attitude_rpy_deg: Vector3
    generation: int


@dataclass(frozen=True, slots=True)
class _DeadReckoningPrior:
    report_rate_hz: float
    packet_dropout_probability: float
    attitude_source: str
    attitude_bias_std_deg: Vector3
    attitude_random_walk_std_deg_sqrt_s: Vector3
    attitude_white_noise_std_deg: Vector3
    position_std_initial_m: float
    position_random_walk_std_m_sqrt_s: float

    def __post_init__(self) -> None:
        if not math.isfinite(self.report_rate_hz) or self.report_rate_hz <= 0.0:
            raise ValueError("report_rate_hz must be finite and positive")
        if not 0.0 <= self.packet_dropout_probability <= 1.0:
            raise ValueError("packet_dropout_probability must be in [0, 1]")
        if self.attitude_source != _MODELED_ATTITUDE_SOURCE:
            raise ValueError(
                "dead_reckoning.attitude_source must be "
                f"{_MODELED_ATTITUDE_SOURCE!r}"
            )
        for name in (
            "attitude_bias_std_deg",
            "attitude_random_walk_std_deg_sqrt_s",
            "attitude_white_noise_std_deg",
        ):
            values = getattr(self, name)
            if len(values) != 3 or not all(math.isfinite(value) for value in values):
                raise ValueError(f"{name} must contain three finite values")
            if any(value < 0.0 for value in values):
                raise ValueError(f"{name} values must be non-negative")
        for name in ("position_std_initial_m", "position_random_walk_std_m_sqrt_s"):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")


@dataclass(frozen=True, slots=True)
class _LoadedDvlSensorConfig:
    path: Path
    profile: str
    calibration_status: str
    enabled: bool
    seed: int
    model: A50SensorConfig
    timing: SensorTimingTransportConfig
    dead_reckoning: _DeadReckoningPrior


def configure_dvl_sensor_runtime(bridge: Any) -> None:
    """Load the A50 profile and attach a deterministic runtime to ``bridge``."""

    loaded = _load_sensor_config()
    bridge._dvl_sensor_config_path = str(loaded.path)
    bridge._dvl_sensor_profile = loaded.profile
    bridge._dvl_sensor_calibration_status = loaded.calibration_status
    bridge._dvl_sensor_enabled = loaded.enabled
    # The publish/schedule integration uses this explicit model-level name.
    bridge._dvl_sensor_model_enabled = loaded.enabled
    bridge._dvl_sensor_seed = loaded.seed
    bridge._dvl_sensor_model_config = loaded.model
    bridge._dvl_sensor_timing_config = loaded.timing
    bridge._dvl_sensor_dead_reckoning_prior = loaded.dead_reckoning
    bridge._dvl_sensor_attitude_source = loaded.dead_reckoning.attitude_source
    bridge._dvl_sensor_model = A50SensorModel(loaded.model) if loaded.enabled else None
    bridge._dvl_sensor_timing = (
        SensorTimingTransportRuntime[_DvlCapturePayload](loaded.timing)
        if loaded.enabled
        else None
    )
    bridge._dvl_sensor_position_timing_config = _position_timing_config(loaded)
    bridge._dvl_sensor_position_timing = (
        SensorTimingTransportRuntime[_DvlPositionPayload](
            bridge._dvl_sensor_position_timing_config
        )
        if loaded.enabled
        else None
    )
    bridge._dvl_sensor_runtime_configured = True
    if loaded.enabled:
        reset_dvl_sensor_runtime(bridge)


def reset_dvl_sensor_runtime(bridge: Any) -> None:
    """Reset seeded sensor/timing streams and both measured DR positions."""

    if not getattr(bridge, "_dvl_sensor_runtime_configured", False):
        raise RuntimeError("configure_dvl_sensor_runtime() must be called before reset")
    if not bridge._dvl_sensor_enabled:
        return

    seed = int(bridge._dvl_sensor_seed)
    bridge._dvl_sensor_model.reset(seed)
    bridge._dvl_sensor_timing.reset(seed=seed)
    bridge._dvl_sensor_position_timing.reset(
        seed=seed ^ _POSITION_TIMING_SEED_SALT
    )
    bridge._dvl_sensor_attitude_rng = np.random.default_rng(
        seed ^ _ATTITUDE_SEED_SALT
    )
    bridge._dvl_sensor_attitude_walk_rng = np.random.default_rng(
        seed ^ _ATTITUDE_WALK_SEED_SALT
    )
    prior = bridge._dvl_sensor_dead_reckoning_prior
    bridge._dvl_sensor_attitude_bias_deg = bridge._dvl_sensor_attitude_rng.normal(
        loc=0.0,
        scale=np.asarray(prior.attitude_bias_std_deg, dtype=np.float64),
        size=3,
    )
    bridge._dvl_sensor_attitude_random_walk_deg = np.zeros(3, dtype=np.float64)
    bridge._dvl_sensor_last_advance_time_s = None
    bridge._dvl_sensor_missed_capture_count = 0
    bridge._dvl_sensor_position_missed_capture_count = 0
    bridge._dvl_sensor_new_deliveries = ()
    bridge._dvl_sensor_new_position_deliveries = ()
    bridge._dvl_sensor_last_velocity_report_validity_us = None
    bridge._dvl_sensor_dr_generation = 0
    reset_dvl_dead_reckoning(bridge, increment_generation=False)
    # A full runtime reset can occur while MuJoCo time is non-zero. Anchor the
    # new capture sequence lazily instead of backfilling samples from zero.
    bridge._dvl_sensor_needs_time_anchor = True


def reset_dvl_dead_reckoning(
    bridge: Any,
    *,
    increment_generation: bool = True,
    reset_time_s: float | None = None,
    rot_world_body: Any | None = None,
) -> None:
    """Reset A50 DR state while preserving sensor phase and noise RNGs.

    ``reset_time_s`` and the legacy ``rot_world_body`` argument may be supplied
    together to anchor the new origin at the exact bridge update where a device
    command was observed.  The rotation value is deliberately ignored: A50
    public dead reckoning is driven only by its modeled gyro attitude source.
    Keeping the argument preserves the device-emulator command boundary while
    preventing private MuJoCo truth from entering a public packet.
    """

    if not getattr(bridge, "_dvl_sensor_runtime_configured", False):
        raise RuntimeError("configure_dvl_sensor_runtime() must be called before reset")
    if not bridge._dvl_sensor_enabled:
        return
    if (reset_time_s is None) != (rot_world_body is None):
        raise ValueError(
            "reset_time_s and rot_world_body must be supplied together"
        )
    reset_time = None
    if reset_time_s is not None:
        reset_time = _finite_float(reset_time_s, "reset_time_s")
        if reset_time < 0.0:
            raise ValueError("reset_time_s must be non-negative")
    prior = bridge._dvl_sensor_dead_reckoning_prior
    if increment_generation:
        bridge._dvl_sensor_dr_generation += 1
        # Reports captured before the reset cannot be allowed to occupy the
        # bounded queue or reappear at the new DR origin. Preserve the 5 Hz
        # schedule and RNG streams while removing only in-flight payloads.
        bridge._dvl_sensor_position_timing.discard_pending()
    if reset_time is None:
        bridge._dvl_sensor_last_capture_time_s = None
    else:
        bridge._dvl_sensor_last_capture_time_s = reset_time
    bridge._dvl_sensor_attitude_rot_initial_from_current = np.eye(
        3,
        dtype=np.float64,
    )
    bridge._dvl_sensor_attitude_source_available = False
    bridge._dvl_sensor_position_local_frd_m = np.zeros(3, dtype=np.float64)
    bridge._dvl_sensor_position_variance_m2 = float(prior.position_std_initial_m) ** 2
    bridge._dvl_sensor_last_delivery = None
    bridge._dvl_sensor_last_position_delivery = None
    bridge._dvl_sensor_new_deliveries = ()
    bridge._dvl_sensor_new_position_deliveries = ()
    bridge._dvl_sensor_current_attitude_rpy_deg = _modeled_attitude_rpy_deg(bridge)
    bridge._odom_pos = np.zeros(3, dtype=np.float64)


def calibrate_dvl_gyro(bridge: Any) -> None:
    """Apply an idealized successful gyro calibration to the DR attitude bias."""

    if not getattr(bridge, "_dvl_sensor_runtime_configured", False):
        raise RuntimeError("configure_dvl_sensor_runtime() must be called before calibration")
    if not bridge._dvl_sensor_enabled:
        return
    bridge._dvl_sensor_attitude_bias_deg = np.zeros(3, dtype=np.float64)
    bridge._dvl_sensor_attitude_random_walk_deg = np.zeros(3, dtype=np.float64)
    bridge._dvl_sensor_current_attitude_rpy_deg = _modeled_attitude_rpy_deg(bridge)


def advance_dvl_sensor_runtime(bridge: Any, state: Any) -> DvlSensorDelivery | None:
    """Advance independent velocity and dead-reckoning report transports.

    ``state`` is duck-typed and must expose ``sim_t``, ``dvl_vel_dvl_frd``, and
    ``dvl_altitude_m``.  ``gyro_bmj`` is the explicit modeled attitude source;
    when it is unavailable the last modeled attitude is held. Multiple packets
    that arrive in one call are retained in bridge delivery tuples; the final
    velocity delivery is returned for the direct-ROS compatibility path.
    """

    if not getattr(bridge, "_dvl_sensor_runtime_configured", False):
        raise RuntimeError("configure_dvl_sensor_runtime() must be called before advance")
    if not bridge._dvl_sensor_enabled:
        return None

    sim_t = _finite_float(state.sim_t, "state.sim_t")
    if sim_t < 0.0:
        raise ValueError("state.sim_t must be non-negative")
    timing = bridge._dvl_sensor_timing
    last_advance = bridge._dvl_sensor_last_advance_time_s
    epsilon_s = timing.config.schedule.epsilon_s
    if last_advance is not None and sim_t + epsilon_s < last_advance:
        reset_dvl_sensor_runtime(bridge)
    if bridge._dvl_sensor_needs_time_anchor:
        _anchor_timing_runtime(bridge, sim_t)
    timing = bridge._dvl_sensor_timing
    bridge._dvl_sensor_last_advance_time_s = sim_t

    captures = timing.schedule_until(sim_t)
    if captures and (
        len(captures) > 1
        or captures[-1].capture_time_s + epsilon_s < sim_t
    ):
        # State history is not available across a large bridge time jump. Drop
        # the stale captures instead of attaching current truth to past
        # timestamps, and retain one honestly timestamped current sample.
        bridge._dvl_sensor_missed_capture_count += len(captures) - 1
        latest = captures[-1]
        captures = [
            replace(
                latest,
                capture_time_s=sim_t,
                device_time_s=timing.config.clock.timestamp(sim_t),
            )
        ]
    for capture in captures:
        payload = _capture_payload(bridge, state, capture)
        _apply_internal_capture(bridge, capture, payload)
        timing.submit(capture, payload)

    position_timing = bridge._dvl_sensor_position_timing
    position_captures = position_timing.schedule_until(sim_t)
    position_epsilon_s = position_timing.config.schedule.epsilon_s
    if position_captures and (
        len(position_captures) > 1
        or position_captures[-1].capture_time_s + position_epsilon_s < sim_t
    ):
        bridge._dvl_sensor_position_missed_capture_count += (
            len(position_captures) - 1
        )
        latest = position_captures[-1]
        position_captures = [
            replace(
                latest,
                capture_time_s=sim_t,
                device_time_s=position_timing.config.clock.timestamp(sim_t),
            )
        ]
    for capture in position_captures:
        position_timing.submit(
            capture,
            _capture_position_payload(bridge),
        )

    deliveries = []
    for packet in timing.drain_arrived(sim_t):
        deliveries.append(_deliver_velocity_packet(bridge, packet))
    bridge._dvl_sensor_new_deliveries = tuple(deliveries)
    last_delivery = deliveries[-1] if deliveries else None
    if last_delivery is not None:
        bridge._dvl_sensor_last_delivery = last_delivery

    position_deliveries = []
    for packet in position_timing.drain_arrived(sim_t):
        delivery = _deliver_position_packet(bridge, packet)
        if delivery is not None:
            position_deliveries.append(delivery)
    bridge._dvl_sensor_new_position_deliveries = tuple(position_deliveries)
    if position_deliveries:
        bridge._dvl_sensor_last_position_delivery = position_deliveries[-1]
    return last_delivery


def _anchor_timing_runtime(bridge: Any, start_time_s: float) -> None:
    template = bridge._dvl_sensor_timing_config
    schedule = replace(template.schedule, start_time_s=float(start_time_s))
    anchored_config = replace(template, schedule=schedule)
    bridge._dvl_sensor_timing = SensorTimingTransportRuntime[_DvlCapturePayload](
        anchored_config
    )
    position_template = bridge._dvl_sensor_position_timing_config
    position_schedule = replace(
        position_template.schedule,
        start_time_s=float(start_time_s),
    )
    bridge._dvl_sensor_position_timing = SensorTimingTransportRuntime[
        _DvlPositionPayload
    ](replace(position_template, schedule=position_schedule))
    bridge._dvl_sensor_needs_time_anchor = False


def _capture_payload(
    bridge: Any,
    state: Any,
    capture: SensorCapture,
) -> _DvlCapturePayload:
    velocity_value = state.dvl_vel_dvl_frd
    if velocity_value is None:
        velocity_frd = np.zeros(3, dtype=np.float64)
        altitude_m = None
    else:
        velocity_frd = _vector3_array(velocity_value, "state.dvl_vel_dvl_frd")
        altitude_value = state.dvl_altitude_m
        altitude_m = (
            None
            if altitude_value is None
            else _finite_float(altitude_value, "state.dvl_altitude_m")
        )

    validity_us = _seconds_to_microseconds(capture.device_time_s, "device capture time")
    sample = bridge._dvl_sensor_model.sample(
        velocity_frd,
        altitude_m,
        time_of_validity_us=validity_us,
        time_of_transmission_us=validity_us,
        beam_ranges_m=getattr(state, "dvl_beam_ranges_m", None),
        incidence_cosines=getattr(state, "dvl_beam_incidence_cosines", None),
    )
    angular_velocity_dvl_frd = _modeled_angular_velocity_dvl_frd(bridge, state)
    return _DvlCapturePayload(
        sample=sample,
        angular_velocity_dvl_frd_rad_s=(
            None
            if angular_velocity_dvl_frd is None
            else tuple(float(value) for value in angular_velocity_dvl_frd)
        ),
    )


def _apply_internal_capture(
    bridge: Any,
    capture: SensorCapture,
    payload: _DvlCapturePayload,
) -> None:
    sample = payload.sample
    last_capture = bridge._dvl_sensor_last_capture_time_s
    dt_s = 0.0 if last_capture is None else max(capture.capture_time_s - last_capture, 0.0)
    if last_capture is None or capture.capture_time_s > last_capture:
        bridge._dvl_sensor_last_capture_time_s = capture.capture_time_s

    _advance_modeled_attitude(
        bridge,
        payload.angular_velocity_dvl_frd_rad_s,
        dt_s,
    )
    rot_initial_from_current = _modeled_attitude_rotation(bridge)

    measured_velocity = getattr(sample, "measured_velocity_frd_mps", None)
    velocity_valid = bool(getattr(sample, "velocity_valid", False))
    if dt_s > 0.0:
        if velocity_valid and measured_velocity is not None:
            velocity_dvl_frd = _vector3_array(
                measured_velocity,
                "sample.measured_velocity_frd_mps",
            )
            velocity_initial_frd = rot_initial_from_current @ velocity_dvl_frd
            bridge._dvl_sensor_position_local_frd_m += velocity_initial_frd * dt_s
            # The A50 exposes position_local in the DR frame established at
            # reset.  The compatibility odometry has no independent global
            # attitude source, so it must use that same modeled DR rotation
            # rather than silently reintroducing MuJoCo world truth.
            bridge._odom_pos = (
                _vector3_array(bridge._odom_pos, "bridge._odom_pos")
                + velocity_initial_frd * dt_s
            )
        _advance_position_uncertainty(bridge, sample, dt_s)

    attitude_rpy_deg = _rotation_to_rpy_deg(rot_initial_from_current)
    bridge._dvl_sensor_current_attitude_rpy_deg = attitude_rpy_deg


def _capture_position_payload(bridge: Any) -> _DvlPositionPayload:
    prior = bridge._dvl_sensor_dead_reckoning_prior
    attitude_noise = bridge._dvl_sensor_attitude_rng.normal(
        loc=0.0,
        scale=np.asarray(prior.attitude_white_noise_std_deg, dtype=np.float64),
        size=3,
    )
    attitude_rpy_deg = (
        bridge._dvl_sensor_current_attitude_rpy_deg
        + attitude_noise
    )
    return _DvlPositionPayload(
        position_local_frd_m=tuple(
            float(value) for value in bridge._dvl_sensor_position_local_frd_m
        ),
        position_std_m=math.sqrt(bridge._dvl_sensor_position_variance_m2),
        attitude_rpy_deg=tuple(float(value) for value in attitude_rpy_deg),
        generation=int(bridge._dvl_sensor_dr_generation),
    )


def _deliver_velocity_packet(
    bridge: Any,
    packet: SensorPacket[_DvlCapturePayload],
) -> DvlSensorDelivery:
    transmission_device_s = packet.device_time_s + packet.processing_delay_s
    sample = replace(
        packet.payload.sample,
        time_of_transmission_us=_seconds_to_microseconds(
            transmission_device_s,
            "device transmission time",
        ),
    )
    validity_us = int(sample.time_of_validity_us)
    previous_validity_us = bridge._dvl_sensor_last_velocity_report_validity_us
    report_period_s = 1.0 / float(
        bridge._dvl_sensor_timing_config.schedule.rate_hz
    )
    if previous_validity_us is not None and validity_us >= previous_validity_us:
        report_period_s = (validity_us - previous_validity_us) * 1.0e-6
    bridge._dvl_sensor_last_velocity_report_validity_us = validity_us
    return DvlSensorDelivery(
        sample=sample,
        capture_time_s=float(packet.capture_time_s),
        arrival_time_s=float(packet.arrival_time_s),
        report_period_s=float(report_period_s),
    )


def _deliver_position_packet(
    bridge: Any,
    packet: SensorPacket[_DvlPositionPayload],
) -> DvlPositionDelivery | None:
    payload = packet.payload
    if payload.generation != int(bridge._dvl_sensor_dr_generation):
        return None
    report_time_s = packet.device_time_s + packet.processing_delay_s
    return DvlPositionDelivery(
        capture_time_s=float(packet.capture_time_s),
        arrival_time_s=float(packet.arrival_time_s),
        report_time_s=float(report_time_s),
        position_local_frd_m=payload.position_local_frd_m,
        position_std_m=float(payload.position_std_m),
        attitude_rpy_deg=payload.attitude_rpy_deg,
        generation=int(payload.generation),
    )


def _advance_position_uncertainty(
    bridge: Any,
    sample: A50SensorSample,
    dt_s: float,
) -> None:
    prior = bridge._dvl_sensor_dead_reckoning_prior
    random_walk = float(prior.position_random_walk_std_m_sqrt_s)
    added_variance = random_walk * random_walk * dt_s
    fom_mps = float(getattr(sample, "fom_mps", 0.0))
    if math.isfinite(fom_mps) and fom_mps >= 0.0:
        added_variance += (fom_mps * dt_s) ** 2
    bridge._dvl_sensor_position_variance_m2 += added_variance


def _modeled_angular_velocity_dvl_frd(
    bridge: Any,
    state: Any,
) -> np.ndarray | None:
    """Return the modeled body gyro expressed in the DVL FRD frame."""

    angular_velocity_bmj = getattr(state, "gyro_bmj", None)
    if angular_velocity_bmj is None:
        return None
    angular_velocity_bmj = _vector3_array(
        angular_velocity_bmj,
        "state.gyro_bmj",
    )
    bmj_to_frd = _rotation_matrix(bridge._bmj_to_frd, "bridge._bmj_to_frd")
    body_frd_to_dvl_frd = _rotation_matrix(
        bridge._dvl_body_frd_to_dvl_frd,
        "bridge._dvl_body_frd_to_dvl_frd",
    )
    return body_frd_to_dvl_frd @ bmj_to_frd @ angular_velocity_bmj


def _advance_modeled_attitude(
    bridge: Any,
    angular_velocity_dvl_frd_rad_s: Vector3 | None,
    dt_s: float,
) -> None:
    """Integrate the A50 attitude from modeled angular rate and drift."""

    if dt_s > 0.0 and angular_velocity_dvl_frd_rad_s is not None:
        angular_velocity = _vector3_array(
            angular_velocity_dvl_frd_rad_s,
            "angular_velocity_dvl_frd_rad_s",
        )
        increment = _rotation_vector_matrix(angular_velocity * dt_s)
        rotation = _rotation_matrix(
            bridge._dvl_sensor_attitude_rot_initial_from_current,
            "bridge._dvl_sensor_attitude_rot_initial_from_current",
        )
        bridge._dvl_sensor_attitude_rot_initial_from_current = rotation @ increment
        bridge._dvl_sensor_attitude_source_available = True

    if dt_s > 0.0:
        prior = bridge._dvl_sensor_dead_reckoning_prior
        walk_std = np.asarray(
            prior.attitude_random_walk_std_deg_sqrt_s,
            dtype=np.float64,
        )
        bridge._dvl_sensor_attitude_random_walk_deg += (
            walk_std
            * math.sqrt(dt_s)
            * bridge._dvl_sensor_attitude_walk_rng.normal(size=3)
        )

    bridge._dvl_sensor_current_attitude_rpy_deg = _modeled_attitude_rpy_deg(bridge)


def _modeled_attitude_rotation(bridge: Any) -> np.ndarray:
    nominal_rotation = _rotation_matrix(
        bridge._dvl_sensor_attitude_rot_initial_from_current,
        "bridge._dvl_sensor_attitude_rot_initial_from_current",
    )
    attitude_error_deg = (
        np.asarray(bridge._dvl_sensor_attitude_bias_deg, dtype=np.float64)
        + np.asarray(
            bridge._dvl_sensor_attitude_random_walk_deg,
            dtype=np.float64,
        )
    )
    return nominal_rotation @ _rotation_vector_matrix(np.radians(attitude_error_deg))


def _modeled_attitude_rpy_deg(bridge: Any) -> np.ndarray:
    return _rotation_to_rpy_deg(_modeled_attitude_rotation(bridge))


def _rotation_vector_matrix(rotation_vector_rad: Any) -> np.ndarray:
    vector = _vector3_array(rotation_vector_rad, "rotation_vector_rad")
    angle = float(np.linalg.norm(vector))
    skew = np.array(
        (
            (0.0, -vector[2], vector[1]),
            (vector[2], 0.0, -vector[0]),
            (-vector[1], vector[0], 0.0),
        ),
        dtype=np.float64,
    )
    if angle <= 1.0e-10:
        # Second order keeps the update orthogonal enough for tiny gyro steps
        # without dividing by a nearly-zero angle.
        return np.eye(3, dtype=np.float64) + skew + 0.5 * (skew @ skew)
    sine_scale = math.sin(angle) / angle
    cosine_scale = (1.0 - math.cos(angle)) / (angle * angle)
    return np.eye(3, dtype=np.float64) + sine_scale * skew + cosine_scale * (skew @ skew)


def _rotation_to_rpy_deg(rotation: np.ndarray) -> np.ndarray:
    sin_pitch = float(np.clip(-rotation[2, 0], -1.0, 1.0))
    pitch = math.asin(sin_pitch)
    if abs(math.cos(pitch)) > 1.0e-9:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = math.atan2(-rotation[1, 2], rotation[1, 1])
        yaw = 0.0
    return np.degrees(np.asarray((roll, pitch, yaw), dtype=np.float64))


def _position_timing_config(
    loaded: _LoadedDvlSensorConfig,
) -> SensorTimingTransportConfig:
    """Build a transport that is independent of velocity report delivery."""

    schedule = replace(
        loaded.timing.schedule,
        rate_hz=loaded.dead_reckoning.report_rate_hz,
        phase_s=0.0,
    )
    transport = replace(
        loaded.timing.transport,
        dropout_probability=loaded.dead_reckoning.packet_dropout_probability,
    )
    return replace(
        loaded.timing,
        schedule=schedule,
        transport=transport,
        seed=loaded.seed ^ _POSITION_TIMING_SEED_SALT,
    )


def _load_sensor_config() -> _LoadedDvlSensorConfig:
    config_path = Path(
        _env_raw("CONFIG_PATH", "CONFIG") or DEFAULT_A50_SENSOR_CONFIG_PATH
    ).expanduser()
    try:
        raw = json.loads(config_path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise FileNotFoundError(f"A50 sensor config not found: {config_path}") from exc
    if not isinstance(raw, dict):
        raise ValueError("A50 sensor config root must be a JSON object")
    if raw.get("schema") != EXPECTED_SCHEMA:
        raise ValueError(f"A50 sensor config schema must be {EXPECTED_SCHEMA!r}")
    calibration_status = str(raw.get("calibration_status", ""))
    if calibration_status != EXPECTED_CALIBRATION_STATUS:
        raise ValueError(
            "A50 prior must remain explicitly marked calibration_status="
            f"{EXPECTED_CALIBRATION_STATUS!r}"
        )

    enabled = _env_bool("ENABLE", default=_required_bool(raw, "enabled"))
    seed = _env_int("SEED", default=_required_int(raw, "seed"))
    if seed < 0:
        raise ValueError("A50 sensor seed must be non-negative")

    timing_data = _required_mapping(raw, "timing")
    capture_data = _required_mapping(timing_data, "capture")
    clock_data = _required_mapping(timing_data, "device_clock")
    processing_data = _required_mapping(timing_data, "processing_latency")
    transport_data = _required_mapping(timing_data, "transport_latency")
    packet_data = _required_mapping(raw, "packet_transport")
    queue_data = _required_mapping(packet_data, "queue")

    schedule = CaptureScheduleConfig(
        rate_hz=_env_float("RATE_HZ", default=_required_float(capture_data, "rate_hz")),
        phase_s=_required_float(capture_data, "phase_s"),
        start_time_s=_required_float(capture_data, "start_time_s"),
    )
    clock = DeviceClockConfig(
        offset_s=_env_float(
            "CLOCK_OFFSET_S",
            default=_required_float(clock_data, "offset_s"),
        ),
        drift_ppm=_env_float(
            "CLOCK_DRIFT_PPM",
            default=_required_float(clock_data, "drift_ppm"),
        ),
        reference_time_s=_required_float(clock_data, "reference_time_s"),
    )
    processing_latency = _latency_config(processing_data, "PROCESSING_LATENCY")
    transport_latency = _latency_config(transport_data, "TRANSPORT_LATENCY")
    overflow_raw = _env_raw("QUEUE_OVERFLOW_POLICY") or str(
        queue_data.get("overflow_policy", "")
    )
    try:
        overflow_policy = OverflowPolicy(overflow_raw)
    except ValueError as exc:
        raise ValueError(f"invalid DVL queue overflow policy: {overflow_raw!r}") from exc
    transport = SensorTransportConfig(
        processing_latency=processing_latency,
        transport_latency=transport_latency,
        dropout_probability=_env_float(
            "PACKET_DROPOUT_PROBABILITY",
            "DROPOUT_PROBABILITY",
            default=_required_float(packet_data, "dropout_probability"),
        ),
        queue_capacity=_env_int(
            "QUEUE_CAPACITY",
            default=_required_int(queue_data, "capacity"),
        ),
        overflow_policy=overflow_policy,
    )
    timing = SensorTimingTransportConfig(
        schedule=schedule,
        clock=clock,
        transport=transport,
        seed=seed,
    )

    model_data = dict(_required_mapping(raw, "a50_sensor_model"))
    model_data.pop("seed", None)
    known_model_fields = {item.name for item in fields(A50SensorConfig)}
    unknown_model_fields = sorted(set(model_data) - known_model_fields)
    if unknown_model_fields:
        raise ValueError(
            "unknown A50 sensor model fields: " + ", ".join(unknown_model_fields)
        )
    model = A50SensorConfig(seed=seed, **model_data)

    dead_reckoning_data = _required_mapping(raw, "dead_reckoning")
    dead_reckoning = _DeadReckoningPrior(
        report_rate_hz=_env_float(
            "POSITION_RATE_HZ",
            default=_required_float(dead_reckoning_data, "report_rate_hz"),
        ),
        packet_dropout_probability=_env_float(
            "POSITION_PACKET_DROPOUT_PROBABILITY",
            default=_required_float(
                dead_reckoning_data,
                "packet_dropout_probability",
            ),
        ),
        attitude_source=str(
            dead_reckoning_data.get(
                "attitude_source",
                _MODELED_ATTITUDE_SOURCE,
            )
        ),
        attitude_bias_std_deg=_vector3_tuple(
            dead_reckoning_data.get("attitude_bias_std_deg"),
            "dead_reckoning.attitude_bias_std_deg",
        ),
        attitude_random_walk_std_deg_sqrt_s=_vector3_tuple(
            dead_reckoning_data.get(
                "attitude_random_walk_std_deg_sqrt_s",
                (0.0, 0.0, 0.0),
            ),
            "dead_reckoning.attitude_random_walk_std_deg_sqrt_s",
        ),
        attitude_white_noise_std_deg=_vector3_tuple(
            dead_reckoning_data.get("attitude_white_noise_std_deg"),
            "dead_reckoning.attitude_white_noise_std_deg",
        ),
        position_std_initial_m=_required_float(
            dead_reckoning_data,
            "position_std_initial_m",
        ),
        position_random_walk_std_m_sqrt_s=_required_float(
            dead_reckoning_data,
            "position_random_walk_std_m_sqrt_s",
        ),
    )
    return _LoadedDvlSensorConfig(
        path=config_path.resolve(),
        profile=str(raw.get("profile", "")),
        calibration_status=calibration_status,
        enabled=enabled,
        seed=seed,
        model=model,
        timing=timing,
        dead_reckoning=dead_reckoning,
    )


def _latency_config(data: Mapping[str, Any], env_stem: str) -> LatencyConfig:
    return LatencyConfig(
        mean_s=_env_float(
            f"{env_stem}_MEAN_S",
            default=_required_float(data, "mean_s"),
        ),
        jitter_std_s=_env_float(
            f"{env_stem}_JITTER_STD_S",
            f"{env_stem}_STD_S",
            default=_required_float(data, "jitter_std_s"),
        ),
        min_s=_env_float(
            f"{env_stem}_MIN_S",
            default=_required_float(data, "min_s"),
        ),
        max_s=_env_optional_float(
            f"{env_stem}_MAX_S",
            default=_optional_float(data, "max_s"),
        ),
    )


def _required_mapping(data: Mapping[str, Any], key: str) -> Mapping[str, Any]:
    value = data.get(key)
    if not isinstance(value, Mapping):
        raise ValueError(f"{key} must be a JSON object")
    return value


def _required_bool(data: Mapping[str, Any], key: str) -> bool:
    value = data.get(key)
    if not isinstance(value, bool):
        raise ValueError(f"{key} must be a boolean")
    return value


def _required_int(data: Mapping[str, Any], key: str) -> int:
    value = data.get(key)
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{key} must be an integer")
    return value


def _required_float(data: Mapping[str, Any], key: str) -> float:
    if key not in data:
        raise ValueError(f"missing required numeric field: {key}")
    return _finite_float(data[key], key)


def _optional_float(data: Mapping[str, Any], key: str) -> float | None:
    value = data.get(key)
    return None if value is None else _finite_float(value, key)


def _env_raw(*suffixes: str) -> str | None:
    for suffix in suffixes:
        value = os.environ.get(f"{_ENV_PREFIX}{suffix}")
        if value is not None and value.strip():
            return value.strip()
    return None


def _env_bool(suffix: str, *, default: bool) -> bool:
    raw = _env_raw(suffix)
    if raw is None:
        return default
    normalized = raw.lower()
    if normalized in {"1", "true", "yes", "on", "enable", "enabled"}:
        return True
    if normalized in {"0", "false", "no", "off", "disable", "disabled"}:
        return False
    raise ValueError(f"{_ENV_PREFIX}{suffix} must be a boolean flag")


def _env_int(*suffixes: str, default: int) -> int:
    raw = _env_raw(*suffixes)
    if raw is None:
        return default
    try:
        return int(raw)
    except ValueError as exc:
        joined = "/".join(f"{_ENV_PREFIX}{suffix}" for suffix in suffixes)
        raise ValueError(f"{joined} must be an integer") from exc


def _env_float(*suffixes: str, default: float) -> float:
    raw = _env_raw(*suffixes)
    return default if raw is None else _finite_float(raw, "/".join(suffixes))


def _env_optional_float(*suffixes: str, default: float | None) -> float | None:
    raw = _env_raw(*suffixes)
    if raw is None:
        return default
    if raw.lower() in {"none", "null", "unbounded"}:
        return None
    return _finite_float(raw, "/".join(suffixes))


def _finite_float(value: Any, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be numeric") from exc
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _vector3_tuple(value: Any, name: str) -> Vector3:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise ValueError(f"{name} must contain three numeric values")
    result = tuple(_finite_float(item, name) for item in value)
    if len(result) != 3:
        raise ValueError(f"{name} must contain three numeric values")
    return result


def _vector3_array(value: Any, name: str) -> np.ndarray:
    try:
        result = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must contain three numeric values") from exc
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite 3-vector")
    return result.copy()


def _rotation_matrix(value: Any, name: str) -> np.ndarray:
    try:
        result = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a 3x3 rotation matrix") from exc
    if result.shape != (3, 3) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite 3x3 rotation matrix")
    if not np.allclose(result.T @ result, np.eye(3), atol=1.0e-5):
        raise ValueError(f"{name} must be orthonormal")
    if not math.isclose(float(np.linalg.det(result)), 1.0, abs_tol=1.0e-5):
        raise ValueError(f"{name} must be a proper rotation")
    return result.copy()


def _seconds_to_microseconds(value_s: float, name: str) -> int:
    value_s = _finite_float(value_s, name)
    if value_s < 0.0:
        raise ValueError(f"{name} must be non-negative for the A50 protocol clock")
    return int(round(value_s * 1_000_000.0))


__all__ = [
    "DEFAULT_A50_SENSOR_CONFIG_PATH",
    "DvlPositionDelivery",
    "DvlSensorDelivery",
    "advance_dvl_sensor_runtime",
    "calibrate_dvl_gyro",
    "configure_dvl_sensor_runtime",
    "reset_dvl_dead_reckoning",
    "reset_dvl_sensor_runtime",
]
