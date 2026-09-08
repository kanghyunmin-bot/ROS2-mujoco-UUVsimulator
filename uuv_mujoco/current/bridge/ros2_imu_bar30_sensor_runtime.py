"""Timed IMU and Bar30 sensor-model integration for the ROS2/SITL bridge.

Sensor captures are applied to the FCU-facing SITL feed as soon as they are
made.  A separate deterministic transport then models when (or whether) that
same capture reaches host-facing ROS publishers.  This prevents ROS packet
loss from unrealistically deleting measurements already consumed inside the
flight controller.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, fields, replace
import json
import math
import os
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

from .bar30_sensor_model import Bar30SensorConfig, Bar30SensorModel, Bar30SensorSample
from .imu_sensor_model import ImuSensorConfig, ImuSensorModel, ImuSensorSample
from .ros2_math import rotmat_to_quat_wxyz
from .ros2_sitl_sensor_types import Bar30VerticalState, BaseKinematicState, ImuDvlState


DEFAULT_IMU_BAR30_SENSOR_CONFIG_PATH = (
    Path(__file__).resolve().parents[1]
    / "config"
    / "sensor_models"
    / "imu_bar30_uncalibrated_prior.json"
)
EXPECTED_SCHEMA = "uuv_mujoco.sensor_model.imu_bar30.v1"
EXPECTED_CALIBRATION_STATUS = "unvalidated_prior"
_COMMON_CONFIG_ENV = "ROS2_UUV_IMU_BAR30_SENSOR_CONFIG_PATH"
_IMU_ENV_PREFIX = "ROS2_UUV_IMU_SENSOR_"
_BAR30_ENV_PREFIX = "ROS2_UUV_BAR30_SENSOR_"


@dataclass(frozen=True, slots=True)
class ImuSensorDelivery:
    """One IMU capture delivered to the ROS host boundary."""

    sample: ImuSensorSample
    capture_time_s: float
    device_time_s: float
    transmission_time_s: float
    arrival_time_s: float


@dataclass(frozen=True, slots=True)
class Bar30SensorDelivery:
    """One Bar30 capture delivered to the ROS host boundary."""

    sample: Bar30SensorSample
    capture_time_s: float
    device_time_s: float
    transmission_time_s: float
    arrival_time_s: float


@dataclass(frozen=True, slots=True)
class _LoadedSensor:
    enabled: bool
    seed: int
    model: object
    timing: SensorTimingTransportConfig
    ambient_temperature_c: float | None = None
    ros_output_rate_hz: float | None = None


@dataclass(frozen=True, slots=True)
class _LoadedProfile:
    path: Path
    profile: str
    calibration_status: str
    imu: _LoadedSensor
    bar30: _LoadedSensor


def configure_imu_bar30_sensor_runtime(bridge: Any) -> None:
    """Attach deterministic IMU and Bar30 models and transports to ``bridge``."""

    loaded = _load_profile()
    bridge._imu_bar30_sensor_config_path = str(loaded.path)
    bridge._imu_bar30_sensor_profile = loaded.profile
    bridge._imu_bar30_sensor_calibration_status = loaded.calibration_status

    bridge._imu_sensor_model_enabled = loaded.imu.enabled
    bridge._imu_sensor_seed = loaded.imu.seed
    bridge._imu_sensor_ros_output_rate_hz = loaded.imu.ros_output_rate_hz
    bridge._imu_sensor_model_config = loaded.imu.model
    bridge._imu_sensor_timing_config = loaded.imu.timing
    bridge._imu_sensor_model = (
        ImuSensorModel(loaded.imu.model) if loaded.imu.enabled else None
    )
    bridge._imu_sensor_timing = (
        SensorTimingTransportRuntime[ImuSensorSample](loaded.imu.timing)
        if loaded.imu.enabled
        else None
    )

    bridge._bar30_sensor_model_enabled = loaded.bar30.enabled
    bridge._bar30_sensor_seed = loaded.bar30.seed
    bridge._bar30_sensor_model_config = loaded.bar30.model
    bridge._bar30_sensor_timing_config = loaded.bar30.timing
    bridge._bar30_sensor_ambient_temperature_c = float(
        loaded.bar30.ambient_temperature_c
        if loaded.bar30.ambient_temperature_c is not None
        else loaded.bar30.model.default_temperature_c
    )
    bridge._bar30_sensor_model = (
        Bar30SensorModel(loaded.bar30.model) if loaded.bar30.enabled else None
    )
    bridge._bar30_sensor_timing = (
        SensorTimingTransportRuntime[Bar30SensorSample](loaded.bar30.timing)
        if loaded.bar30.enabled
        else None
    )
    bridge._imu_bar30_sensor_runtime_configured = True
    reset_imu_bar30_sensor_runtime(bridge)


def reset_imu_bar30_sensor_runtime(bridge: Any) -> None:
    """Reset sensor states, schedules, queues, and seeded random streams."""

    if not getattr(bridge, "_imu_bar30_sensor_runtime_configured", False):
        raise RuntimeError(
            "configure_imu_bar30_sensor_runtime() must be called before reset"
        )
    if bridge._imu_sensor_model_enabled:
        bridge._imu_sensor_model.reset(bridge._imu_sensor_seed)
        bridge._imu_sensor_timing.reset(seed=bridge._imu_sensor_seed)
    if bridge._bar30_sensor_model_enabled:
        bridge._bar30_sensor_model.reset(bridge._bar30_sensor_seed)
        bridge._bar30_sensor_timing.reset(seed=bridge._bar30_sensor_seed)
    bridge._imu_sensor_latest_capture = None
    bridge._imu_sensor_next_ros_capture_s = None
    bridge._bar30_sensor_latest_capture = None
    bridge._imu_sensor_new_deliveries = ()
    bridge._bar30_sensor_new_deliveries = ()
    bridge._imu_sensor_last_delivery = None
    bridge._bar30_sensor_last_delivery = None
    bridge._imu_sensor_missed_capture_count = 0
    bridge._bar30_sensor_missed_capture_count = 0
    bridge._imu_bar30_sensor_last_advance_time_s = None
    # Construction and MuJoCo reset can occur at non-zero time. Avoid
    # backfilling captures for states that were never observed by the model.
    bridge._imu_bar30_sensor_needs_time_anchor = True


def advance_imu_bar30_sensor_runtime(
    bridge: Any,
    base: BaseKinematicState,
    imu_dvl: ImuDvlState,
    vertical: Bar30VerticalState,
) -> tuple[
    ImuDvlState,
    Bar30VerticalState,
    tuple[ImuSensorDelivery, ...],
    tuple[Bar30SensorDelivery, ...],
]:
    """Advance both independent sensor streams and apply captures to SITL."""

    if not getattr(bridge, "_imu_bar30_sensor_runtime_configured", False):
        raise RuntimeError(
            "configure_imu_bar30_sensor_runtime() must be called before advance"
        )
    sim_t = _finite_float(base.sim_t, "base.sim_t")
    if sim_t < 0.0:
        raise ValueError("base.sim_t must be non-negative")
    last_time = bridge._imu_bar30_sensor_last_advance_time_s
    if last_time is not None and sim_t + 1.0e-9 < last_time:
        reset_imu_bar30_sensor_runtime(bridge)
    if bridge._imu_bar30_sensor_needs_time_anchor:
        _anchor_timing_runtimes(bridge, sim_t)
    bridge._imu_bar30_sensor_last_advance_time_s = sim_t

    measured_imu = _advance_imu(bridge, base, imu_dvl, sim_t)
    measured_vertical = _advance_bar30(bridge, vertical, sim_t)
    return (
        measured_imu,
        measured_vertical,
        tuple(bridge._imu_sensor_new_deliveries),
        tuple(bridge._bar30_sensor_new_deliveries),
    )


def _advance_imu(
    bridge: Any,
    base: BaseKinematicState,
    imu_dvl: ImuDvlState,
    sim_t: float,
) -> ImuDvlState:
    if not bridge._imu_sensor_model_enabled:
        bridge._imu_sensor_new_deliveries = ()
        return imu_dvl
    timing = bridge._imu_sensor_timing
    captures = _captures_for_current_state(
        timing,
        sim_t,
        bridge,
        "_imu_sensor_missed_capture_count",
    )
    if imu_dvl.gyro_bmj is not None and imu_dvl.acc_bmj is not None:
        quat_ros = rotmat_to_quat_wxyz(base.base_rot_enu @ bridge._bmj_to_flu.T)
        gyro_ros = bridge._bmj_to_flu @ np.asarray(imu_dvl.gyro_bmj, dtype=np.float64)
        # Keep the physical sensor model ahead of the legacy ROS-only output
        # scale/bias shim. Applying that shim here would also alter the SITL
        # FCU input and make a ROS presentation setting change plant behavior.
        accel_ros = bridge._bmj_to_flu @ np.asarray(
            imu_dvl.acc_bmj, dtype=np.float64
        )
        for capture in captures:
            sample = bridge._imu_sensor_model.sample(
                quat_ros,
                gyro_ros,
                accel_ros,
                sample_time_s=capture.capture_time_s,
            )
            bridge._imu_sensor_latest_capture = sample
            # FCU consumes every capture; host telemetry selects a subset
            # before latency/dropout modeling, without changing FCU sensing.
            rate = bridge._imu_sensor_ros_output_rate_hz
            next_host = bridge._imu_sensor_next_ros_capture_s
            if rate is None or next_host is None or capture.capture_time_s + 1e-9 >= next_host:
                timing.submit(capture, sample)
                if rate is not None:
                    period = 1.0 / rate
                    anchor = capture.capture_time_s if next_host is None else next_host
                    skipped = max(0, math.floor((capture.capture_time_s - anchor + 1e-9) / period))
                    bridge._imu_sensor_next_ros_capture_s = anchor + (skipped + 1) * period

    deliveries = tuple(
        _imu_delivery(packet) for packet in timing.drain_arrived(sim_t)
    )
    bridge._imu_sensor_new_deliveries = deliveries
    if deliveries:
        bridge._imu_sensor_last_delivery = deliveries[-1]

    latest = bridge._imu_sensor_latest_capture
    if latest is None:
        return imu_dvl
    measured_gyro_bmj = bridge._bmj_to_flu.T @ np.asarray(
        latest.angular_velocity_rad_s, dtype=np.float64
    )
    measured_accel_bmj = bridge._bmj_to_flu.T @ np.asarray(
        latest.linear_acceleration_mps2, dtype=np.float64
    )
    return replace(
        imu_dvl,
        gyro_bmj=measured_gyro_bmj,
        acc_bmj=measured_accel_bmj,
    )


def _advance_bar30(
    bridge: Any,
    vertical: Bar30VerticalState,
    sim_t: float,
) -> Bar30VerticalState:
    if not bridge._bar30_sensor_model_enabled:
        bridge._bar30_sensor_new_deliveries = ()
        return vertical
    timing = bridge._bar30_sensor_timing
    captures = _captures_for_current_state(
        timing,
        sim_t,
        bridge,
        "_bar30_sensor_missed_capture_count",
    )
    for capture in captures:
        sample = bridge._bar30_sensor_model.sample(
            vertical.bar30_pressure_pa,
            sample_time_s=capture.capture_time_s,
            ambient_temperature_c=bridge._bar30_sensor_ambient_temperature_c,
        )
        bridge._bar30_sensor_latest_capture = sample
        timing.submit(capture, sample)

    deliveries = tuple(
        _bar30_delivery(packet) for packet in timing.drain_arrived(sim_t)
    )
    bridge._bar30_sensor_new_deliveries = deliveries
    if deliveries:
        bridge._bar30_sensor_last_delivery = deliveries[-1]

    latest = bridge._bar30_sensor_latest_capture
    if latest is None:
        return vertical
    measured_pressure_pa = float(latest.measured_pressure_pa)
    measured_depth_m = float(
        bridge._baro_pressure_law.frontend_depth_m_from_pressure(measured_pressure_pa)
    )
    physical_depth_m = max(
        0.0,
        (measured_pressure_pa - float(bridge._bar30_surface_pressure_pa))
        / (float(bridge._bar30_water_density) * float(bridge._bar30_gravity)),
    )
    if bridge._sitl_baro_depth_contract == "frontend_match":
        sitl_depth_m = float(
            bridge._baro_pressure_law.sitl_depth_m_for_frontend_match(
                measured_pressure_pa
            )
        )
    else:
        sitl_depth_m = float(physical_depth_m)
    vertical_estimate = replace(
        vertical.vertical_estimate,
        depth_m=sitl_depth_m,
        pressure_pa=measured_pressure_pa,
        alt_m=float(bridge._sitl_home_alt_m - sitl_depth_m),
    )
    return replace(
        vertical,
        vertical_estimate=vertical_estimate,
        bar30_pressure_pa=measured_pressure_pa,
        ros_depth_m=measured_depth_m,
    )


def _captures_for_current_state(
    timing: SensorTimingTransportRuntime,
    sim_t: float,
    bridge: Any,
    missed_count_attr: str,
) -> list[SensorCapture]:
    captures = timing.schedule_until(sim_t)
    epsilon_s = timing.config.schedule.epsilon_s
    if captures and (
        len(captures) > 1
        or captures[-1].capture_time_s + epsilon_s < sim_t
    ):
        setattr(bridge, missed_count_attr, getattr(bridge, missed_count_attr) + len(captures) - 1)
        latest = captures[-1]
        captures = [
            replace(
                latest,
                capture_time_s=sim_t,
                device_time_s=timing.config.clock.timestamp(sim_t),
            )
        ]
    return captures


def _imu_delivery(packet: SensorPacket[ImuSensorSample]) -> ImuSensorDelivery:
    return ImuSensorDelivery(
        sample=packet.payload,
        capture_time_s=float(packet.capture_time_s),
        device_time_s=float(packet.device_time_s),
        transmission_time_s=float(packet.transmission_time_s),
        arrival_time_s=float(packet.arrival_time_s),
    )


def _bar30_delivery(packet: SensorPacket[Bar30SensorSample]) -> Bar30SensorDelivery:
    return Bar30SensorDelivery(
        sample=packet.payload,
        capture_time_s=float(packet.capture_time_s),
        device_time_s=float(packet.device_time_s),
        transmission_time_s=float(packet.transmission_time_s),
        arrival_time_s=float(packet.arrival_time_s),
    )


def _anchor_timing_runtimes(bridge: Any, start_time_s: float) -> None:
    if bridge._imu_sensor_model_enabled:
        config = bridge._imu_sensor_timing_config
        bridge._imu_sensor_timing = SensorTimingTransportRuntime[ImuSensorSample](
            replace(
                config,
                schedule=replace(config.schedule, start_time_s=float(start_time_s)),
            )
        )
    if bridge._bar30_sensor_model_enabled:
        config = bridge._bar30_sensor_timing_config
        bridge._bar30_sensor_timing = SensorTimingTransportRuntime[Bar30SensorSample](
            replace(
                config,
                schedule=replace(config.schedule, start_time_s=float(start_time_s)),
            )
        )
    bridge._imu_bar30_sensor_needs_time_anchor = False


def _load_profile() -> _LoadedProfile:
    config_path = Path(
        os.environ.get(_COMMON_CONFIG_ENV)
        or os.environ.get(f"{_IMU_ENV_PREFIX}CONFIG_PATH")
        or os.environ.get(f"{_BAR30_ENV_PREFIX}CONFIG_PATH")
        or DEFAULT_IMU_BAR30_SENSOR_CONFIG_PATH
    ).expanduser()
    try:
        raw = json.loads(config_path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise FileNotFoundError(f"IMU/Bar30 sensor config not found: {config_path}") from exc
    if not isinstance(raw, Mapping):
        raise ValueError("IMU/Bar30 sensor config root must be a JSON object")
    if raw.get("schema") != EXPECTED_SCHEMA:
        raise ValueError(f"IMU/Bar30 sensor config schema must be {EXPECTED_SCHEMA!r}")
    calibration_status = str(raw.get("calibration_status", ""))
    if calibration_status != EXPECTED_CALIBRATION_STATUS:
        raise ValueError(
            "IMU/Bar30 prior must remain explicitly marked calibration_status="
            f"{EXPECTED_CALIBRATION_STATUS!r}"
        )
    imu = _load_sensor(
        _required_mapping(raw, "imu"),
        prefix=_IMU_ENV_PREFIX,
        model_type=ImuSensorConfig,
    )
    bar30 = _load_sensor(
        _required_mapping(raw, "bar30"),
        prefix=_BAR30_ENV_PREFIX,
        model_type=Bar30SensorConfig,
    )
    return _LoadedProfile(
        path=config_path.resolve(),
        profile=str(raw.get("profile", "")),
        calibration_status=calibration_status,
        imu=imu,
        bar30=bar30,
    )


def _load_sensor(
    data: Mapping[str, Any],
    *,
    prefix: str,
    model_type: type[ImuSensorConfig] | type[Bar30SensorConfig],
) -> _LoadedSensor:
    enabled = _env_bool(prefix, "ENABLE", _required_bool(data, "enabled"))
    seed = _env_int(prefix, "SEED", _required_int(data, "seed"))
    if seed < 0:
        raise ValueError(f"{prefix}SEED must be non-negative")
    timing_data = _required_mapping(data, "timing")
    capture_data = _required_mapping(timing_data, "capture")
    clock_data = _required_mapping(timing_data, "device_clock")
    packet_data = _required_mapping(data, "packet_transport")
    queue_data = _required_mapping(packet_data, "queue")
    rate_hz = _env_float(prefix, "RATE_HZ", _required_float(capture_data, "rate_hz"))
    timing = SensorTimingTransportConfig(
        schedule=CaptureScheduleConfig(
            rate_hz=rate_hz,
            phase_s=_required_float(capture_data, "phase_s"),
            start_time_s=_required_float(capture_data, "start_time_s"),
        ),
        clock=DeviceClockConfig(
            offset_s=_env_float(prefix, "CLOCK_OFFSET_S", _required_float(clock_data, "offset_s")),
            drift_ppm=_env_float(prefix, "CLOCK_DRIFT_PPM", _required_float(clock_data, "drift_ppm")),
            reference_time_s=_required_float(clock_data, "reference_time_s"),
        ),
        transport=SensorTransportConfig(
            processing_latency=_latency_config(
                _required_mapping(timing_data, "processing_latency"),
                prefix,
                "PROCESSING_LATENCY",
            ),
            transport_latency=_latency_config(
                _required_mapping(timing_data, "transport_latency"),
                prefix,
                "TRANSPORT_LATENCY",
            ),
            dropout_probability=_env_float(
                prefix,
                "PACKET_DROPOUT_PROBABILITY",
                _required_float(packet_data, "dropout_probability"),
            ),
            queue_capacity=_env_int(
                prefix,
                "QUEUE_CAPACITY",
                _required_int(queue_data, "capacity"),
            ),
            overflow_policy=_overflow_policy(prefix, queue_data),
        ),
        seed=seed,
    )
    model_data = dict(_required_mapping(data, "model"))
    model_data.pop("basis", None)
    model_data.pop("seed", None)
    model_data["nominal_rate_hz"] = rate_hz
    known_fields = {field.name for field in fields(model_type)}
    unknown_fields = sorted(set(model_data) - known_fields)
    if unknown_fields:
        raise ValueError(
            f"unknown {model_type.__name__} fields: " + ", ".join(unknown_fields)
        )
    model = model_type(seed=seed, **model_data)
    ambient_temperature_c = None
    if model_type is Bar30SensorConfig:
        ambient_temperature_c = _env_float(
            prefix,
            "AMBIENT_TEMPERATURE_C",
            _required_float(data, "ambient_temperature_c"),
        )
    host_rate = data.get("ros_output_rate_hz")
    if model_type is ImuSensorConfig and (host_rate is not None or f"{prefix}ROS_RATE_HZ" in os.environ):
        host_rate = _env_float(prefix, "ROS_RATE_HZ", rate_hz if host_rate is None else float(host_rate))
        if not math.isfinite(host_rate) or host_rate <= 0.0:
            raise ValueError("IMU ROS output rate must be positive and finite")
    return _LoadedSensor(
        ros_output_rate_hz=host_rate,
        enabled=enabled,
        seed=seed,
        model=model,
        timing=timing,
        ambient_temperature_c=ambient_temperature_c,
    )


def _latency_config(
    data: Mapping[str, Any],
    prefix: str,
    stem: str,
) -> LatencyConfig:
    return LatencyConfig(
        mean_s=_env_float(prefix, f"{stem}_MEAN_S", _required_float(data, "mean_s")),
        jitter_std_s=_env_float(
            prefix,
            f"{stem}_JITTER_STD_S",
            _required_float(data, "jitter_std_s"),
        ),
        min_s=_env_float(prefix, f"{stem}_MIN_S", _required_float(data, "min_s")),
        max_s=_env_optional_float(prefix, f"{stem}_MAX_S", _optional_float(data, "max_s")),
    )


def _overflow_policy(prefix: str, queue_data: Mapping[str, Any]) -> OverflowPolicy:
    raw = os.environ.get(f"{prefix}QUEUE_OVERFLOW_POLICY") or str(
        queue_data.get("overflow_policy", "")
    )
    try:
        return OverflowPolicy(raw.strip())
    except ValueError as exc:
        raise ValueError(f"invalid {prefix}QUEUE_OVERFLOW_POLICY={raw!r}") from exc


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


def _env_bool(prefix: str, suffix: str, default: bool) -> bool:
    raw = os.environ.get(f"{prefix}{suffix}")
    if raw is None or not raw.strip():
        return default
    normalized = raw.strip().lower()
    if normalized in {"1", "true", "yes", "on", "enabled"}:
        return True
    if normalized in {"0", "false", "no", "off", "disabled"}:
        return False
    raise ValueError(f"{prefix}{suffix} must be a boolean flag")


def _env_int(prefix: str, suffix: str, default: int) -> int:
    raw = os.environ.get(f"{prefix}{suffix}")
    if raw is None or not raw.strip():
        return default
    try:
        return int(raw)
    except ValueError as exc:
        raise ValueError(f"{prefix}{suffix} must be an integer") from exc


def _env_float(prefix: str, suffix: str, default: float) -> float:
    raw = os.environ.get(f"{prefix}{suffix}")
    return default if raw is None or not raw.strip() else _finite_float(raw, suffix)


def _env_optional_float(
    prefix: str,
    suffix: str,
    default: float | None,
) -> float | None:
    raw = os.environ.get(f"{prefix}{suffix}")
    if raw is None or not raw.strip():
        return default
    if raw.strip().lower() in {"none", "null", "unbounded"}:
        return None
    return _finite_float(raw, suffix)


def _finite_float(value: Any, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be numeric") from exc
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


__all__ = [
    "Bar30SensorDelivery",
    "DEFAULT_IMU_BAR30_SENSOR_CONFIG_PATH",
    "ImuSensorDelivery",
    "advance_imu_bar30_sensor_runtime",
    "configure_imu_bar30_sensor_runtime",
    "reset_imu_bar30_sensor_runtime",
]
