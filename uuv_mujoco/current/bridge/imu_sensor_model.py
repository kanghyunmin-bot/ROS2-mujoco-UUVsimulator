"""Deterministic, ROS-independent MEMS IMU error model.

The model consumes attitude, angular rate, and specific force in one common
orthonormal sensor frame.  It combines a fixed cross-axis/scale matrix with
turn-on bias, first-order Gauss-Markov bias instability, unbounded random
walk, bandwidth-scaled white noise, saturation, and output quantization.

The defaults are intentionally not tied to an unidentified flight-controller
IMU.  They are simulation priors and must be replaced by Allan-deviation and
rate-table results from the actual vehicle before quantitative sim-to-real
claims are made.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np


Vector3 = tuple[float, float, float]
Matrix3 = tuple[float, float, float, float, float, float, float, float, float]


def _vector3(values: Sequence[float], name: str, *, nonnegative: bool = False) -> Vector3:
    result = tuple(float(value) for value in values)
    if len(result) != 3 or not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must contain three finite values")
    if nonnegative and any(value < 0.0 for value in result):
        raise ValueError(f"{name} must contain non-negative values")
    return result  # type: ignore[return-value]


def _matrix3(values: Sequence[float], name: str) -> Matrix3:
    result = tuple(float(value) for value in values)
    if len(result) != 9 or not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must contain nine finite row-major values")
    matrix = np.asarray(result, dtype=np.float64).reshape(3, 3)
    if abs(float(np.linalg.det(matrix))) <= 1.0e-9:
        raise ValueError(f"{name} must be non-singular")
    return result  # type: ignore[return-value]


@dataclass(frozen=True, slots=True)
class ImuSensorConfig:
    """Configuration for a sampled three-axis MEMS IMU prior."""

    seed: int = 2611
    nominal_rate_hz: float = 50.0
    gyro_scale_cross_axis_matrix: Matrix3 = (
        1.0002, 0.0008, -0.0004,
        -0.0006, 0.9998, 0.0007,
        0.0003, -0.0005, 1.0001,
    )
    accel_scale_cross_axis_matrix: Matrix3 = (
        1.0010, 0.0010, -0.0005,
        -0.0008, 0.9990, 0.0009,
        0.0004, -0.0007, 1.0005,
    )
    gyro_constant_bias_rad_s: Vector3 = (0.0, 0.0, 0.0)
    gyro_turn_on_bias_std_rad_s: Vector3 = (0.002, 0.002, 0.003)
    gyro_bias_instability_std_rad_s: Vector3 = (0.0005, 0.0005, 0.0008)
    gyro_bias_correlation_time_s: float = 300.0
    gyro_random_walk_rad_s_per_sqrt_s: Vector3 = (2.0e-5, 2.0e-5, 3.0e-5)
    gyro_white_noise_density_rad_s_sqrt_hz: Vector3 = (3.5e-4, 3.5e-4, 5.0e-4)
    gyro_saturation_rad_s: Vector3 = (34.9066, 34.9066, 34.9066)
    gyro_quantization_rad_s: Vector3 = (0.001, 0.001, 0.001)
    accel_constant_bias_mps2: Vector3 = (0.0, 0.0, 0.0)
    accel_turn_on_bias_std_mps2: Vector3 = (0.03, 0.03, 0.04)
    accel_bias_instability_std_mps2: Vector3 = (0.01, 0.01, 0.015)
    accel_bias_correlation_time_s: float = 300.0
    accel_random_walk_mps2_per_sqrt_s: Vector3 = (5.0e-4, 5.0e-4, 7.0e-4)
    accel_white_noise_density_mps2_sqrt_hz: Vector3 = (0.012, 0.012, 0.016)
    accel_saturation_mps2: Vector3 = (156.9064, 156.9064, 156.9064)
    accel_quantization_mps2: Vector3 = (0.001, 0.001, 0.001)
    attitude_turn_on_bias_std_rad: Vector3 = (0.003, 0.003, 0.006)
    attitude_white_noise_std_rad: Vector3 = (0.001, 0.001, 0.002)

    def __post_init__(self) -> None:
        if isinstance(self.seed, bool) or not isinstance(self.seed, int):
            raise TypeError("seed must be an integer")
        if self.seed < 0:
            raise ValueError("seed must be non-negative")
        if not math.isfinite(self.nominal_rate_hz) or self.nominal_rate_hz <= 0.0:
            raise ValueError("nominal_rate_hz must be finite and positive")
        object.__setattr__(
            self,
            "gyro_scale_cross_axis_matrix",
            _matrix3(self.gyro_scale_cross_axis_matrix, "gyro_scale_cross_axis_matrix"),
        )
        object.__setattr__(
            self,
            "accel_scale_cross_axis_matrix",
            _matrix3(self.accel_scale_cross_axis_matrix, "accel_scale_cross_axis_matrix"),
        )
        vector_fields = (
            "gyro_constant_bias_rad_s",
            "gyro_turn_on_bias_std_rad_s",
            "gyro_bias_instability_std_rad_s",
            "gyro_random_walk_rad_s_per_sqrt_s",
            "gyro_white_noise_density_rad_s_sqrt_hz",
            "gyro_saturation_rad_s",
            "gyro_quantization_rad_s",
            "accel_constant_bias_mps2",
            "accel_turn_on_bias_std_mps2",
            "accel_bias_instability_std_mps2",
            "accel_random_walk_mps2_per_sqrt_s",
            "accel_white_noise_density_mps2_sqrt_hz",
            "accel_saturation_mps2",
            "accel_quantization_mps2",
            "attitude_turn_on_bias_std_rad",
            "attitude_white_noise_std_rad",
        )
        signed_fields = {"gyro_constant_bias_rad_s", "accel_constant_bias_mps2"}
        for name in vector_fields:
            object.__setattr__(
                self,
                name,
                _vector3(getattr(self, name), name, nonnegative=name not in signed_fields),
            )
        for name in ("gyro_bias_correlation_time_s", "accel_bias_correlation_time_s"):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        for name in ("gyro_saturation_rad_s", "accel_saturation_mps2"):
            if any(value <= 0.0 for value in getattr(self, name)):
                raise ValueError(f"{name} must contain positive values")


@dataclass(frozen=True, slots=True)
class ImuSensorSample:
    """One sampled IMU observation in the configured sensor frame."""

    sample_index: int
    sample_time_s: float
    orientation_wxyz: tuple[float, float, float, float]
    angular_velocity_rad_s: Vector3
    linear_acceleration_mps2: Vector3
    gyro_bias_rad_s: Vector3
    accel_bias_mps2: Vector3
    orientation_covariance_diag_rad2: Vector3
    angular_velocity_covariance_diag_rad2_s2: Vector3
    linear_acceleration_covariance_diag_m2_s4: Vector3
    gyro_saturated: tuple[bool, bool, bool]
    accel_saturated: tuple[bool, bool, bool]


class ImuSensorModel:
    """Seeded sampled IMU model with distinct bias and white-noise terms."""

    def __init__(self, config: ImuSensorConfig | None = None) -> None:
        self.config = config or ImuSensorConfig()
        self.reset()

    def reset(self, seed: int | None = None) -> None:
        """Reset all stochastic states for deterministic replay."""

        selected_seed = self.config.seed if seed is None else int(seed)
        self._rng = np.random.default_rng(selected_seed)
        cfg = self.config
        self._gyro_turn_on_bias = self._rng.normal(
            0.0, np.asarray(cfg.gyro_turn_on_bias_std_rad_s, dtype=np.float64)
        )
        self._accel_turn_on_bias = self._rng.normal(
            0.0, np.asarray(cfg.accel_turn_on_bias_std_mps2, dtype=np.float64)
        )
        self._attitude_turn_on_bias = self._rng.normal(
            0.0, np.asarray(cfg.attitude_turn_on_bias_std_rad, dtype=np.float64)
        )
        self._gyro_bias_instability = np.zeros(3, dtype=np.float64)
        self._accel_bias_instability = np.zeros(3, dtype=np.float64)
        self._gyro_random_walk = np.zeros(3, dtype=np.float64)
        self._accel_random_walk = np.zeros(3, dtype=np.float64)
        self._last_sample_time_s: float | None = None
        self._start_sample_time_s: float | None = None
        self._sample_index = 0

    def sample(
        self,
        orientation_wxyz: Sequence[float],
        angular_velocity_rad_s: Sequence[float],
        linear_acceleration_mps2: Sequence[float],
        *,
        sample_time_s: float,
    ) -> ImuSensorSample:
        """Generate one IMU observation at ``sample_time_s`` [s]."""

        time_s = float(sample_time_s)
        if not math.isfinite(time_s) or time_s < 0.0:
            raise ValueError("sample_time_s must be finite and non-negative")
        if self._last_sample_time_s is not None and time_s < self._last_sample_time_s - 1.0e-12:
            raise ValueError("sample_time_s moved backwards; call reset() first")
        dt_s = (
            1.0 / self.config.nominal_rate_hz
            if self._last_sample_time_s is None
            else max(time_s - self._last_sample_time_s, 1.0e-12)
        )
        self._last_sample_time_s = time_s
        if self._start_sample_time_s is None:
            self._start_sample_time_s = time_s

        quat = _normalized_quaternion(orientation_wxyz)
        gyro_truth = np.asarray(_vector3(angular_velocity_rad_s, "angular_velocity_rad_s"))
        accel_truth = np.asarray(_vector3(linear_acceleration_mps2, "linear_acceleration_mps2"))
        cfg = self.config

        self._gyro_bias_instability = _gauss_markov_step(
            self._gyro_bias_instability,
            np.asarray(cfg.gyro_bias_instability_std_rad_s),
            cfg.gyro_bias_correlation_time_s,
            dt_s,
            self._rng.normal(size=3),
        )
        self._accel_bias_instability = _gauss_markov_step(
            self._accel_bias_instability,
            np.asarray(cfg.accel_bias_instability_std_mps2),
            cfg.accel_bias_correlation_time_s,
            dt_s,
            self._rng.normal(size=3),
        )
        self._gyro_random_walk += (
            np.asarray(cfg.gyro_random_walk_rad_s_per_sqrt_s)
            * math.sqrt(dt_s)
            * self._rng.normal(size=3)
        )
        self._accel_random_walk += (
            np.asarray(cfg.accel_random_walk_mps2_per_sqrt_s)
            * math.sqrt(dt_s)
            * self._rng.normal(size=3)
        )

        # Noise density is mapped through the configured sensor bandwidth,
        # not the caller's update delay. A delayed bridge update must not
        # silently turn the IMU into a lower-bandwidth device.
        noise_period_s = 1.0 / cfg.nominal_rate_hz
        gyro_white_std = np.asarray(
            cfg.gyro_white_noise_density_rad_s_sqrt_hz
        ) / math.sqrt(2.0 * noise_period_s)
        accel_white_std = np.asarray(
            cfg.accel_white_noise_density_mps2_sqrt_hz
        ) / math.sqrt(2.0 * noise_period_s)
        gyro_bias = (
            np.asarray(cfg.gyro_constant_bias_rad_s)
            + self._gyro_turn_on_bias
            + self._gyro_bias_instability
            + self._gyro_random_walk
        )
        accel_bias = (
            np.asarray(cfg.accel_constant_bias_mps2)
            + self._accel_turn_on_bias
            + self._accel_bias_instability
            + self._accel_random_walk
        )
        gyro_unclipped = (
            np.asarray(cfg.gyro_scale_cross_axis_matrix).reshape(3, 3) @ gyro_truth
            + gyro_bias
            + gyro_white_std * self._rng.normal(size=3)
        )
        accel_unclipped = (
            np.asarray(cfg.accel_scale_cross_axis_matrix).reshape(3, 3) @ accel_truth
            + accel_bias
            + accel_white_std * self._rng.normal(size=3)
        )
        gyro, gyro_saturated = _saturate_and_quantize(
            gyro_unclipped,
            np.asarray(cfg.gyro_saturation_rad_s),
            np.asarray(cfg.gyro_quantization_rad_s),
        )
        accel, accel_saturated = _saturate_and_quantize(
            accel_unclipped,
            np.asarray(cfg.accel_saturation_mps2),
            np.asarray(cfg.accel_quantization_mps2),
        )

        attitude_error = self._attitude_turn_on_bias + np.asarray(
            cfg.attitude_white_noise_std_rad
        ) * self._rng.normal(size=3)
        measured_quat = _quaternion_multiply(quat, _rotation_vector_quaternion(attitude_error))
        measured_quat /= np.linalg.norm(measured_quat)

        elapsed_s = time_s - float(self._start_sample_time_s)
        gyro_covariance = (
            gyro_white_std**2
            + np.asarray(cfg.gyro_turn_on_bias_std_rad_s) ** 2
            + np.asarray(cfg.gyro_bias_instability_std_rad_s) ** 2
            + np.asarray(cfg.gyro_random_walk_rad_s_per_sqrt_s) ** 2 * elapsed_s
            + np.asarray(cfg.gyro_quantization_rad_s) ** 2 / 12.0
        )
        accel_covariance = (
            accel_white_std**2
            + np.asarray(cfg.accel_turn_on_bias_std_mps2) ** 2
            + np.asarray(cfg.accel_bias_instability_std_mps2) ** 2
            + np.asarray(cfg.accel_random_walk_mps2_per_sqrt_s) ** 2 * elapsed_s
            + np.asarray(cfg.accel_quantization_mps2) ** 2 / 12.0
        )
        attitude_covariance = (
            np.asarray(cfg.attitude_turn_on_bias_std_rad) ** 2
            + np.asarray(cfg.attitude_white_noise_std_rad) ** 2
        )

        sample = ImuSensorSample(
            sample_index=self._sample_index,
            sample_time_s=time_s,
            orientation_wxyz=tuple(float(value) for value in measured_quat),
            angular_velocity_rad_s=tuple(float(value) for value in gyro),
            linear_acceleration_mps2=tuple(float(value) for value in accel),
            gyro_bias_rad_s=tuple(float(value) for value in gyro_bias),
            accel_bias_mps2=tuple(float(value) for value in accel_bias),
            orientation_covariance_diag_rad2=tuple(float(value) for value in attitude_covariance),
            angular_velocity_covariance_diag_rad2_s2=tuple(
                float(value) for value in gyro_covariance
            ),
            linear_acceleration_covariance_diag_m2_s4=tuple(
                float(value) for value in accel_covariance
            ),
            gyro_saturated=tuple(bool(value) for value in gyro_saturated),
            accel_saturated=tuple(bool(value) for value in accel_saturated),
        )
        self._sample_index += 1
        return sample


def _normalized_quaternion(values: Sequence[float]) -> np.ndarray:
    quat = np.asarray(tuple(float(value) for value in values), dtype=np.float64)
    if quat.shape != (4,) or not np.all(np.isfinite(quat)):
        raise ValueError("orientation_wxyz must contain four finite values")
    norm = float(np.linalg.norm(quat))
    if norm <= 1.0e-12:
        raise ValueError("orientation_wxyz must have non-zero norm")
    return quat / norm


def _gauss_markov_step(
    previous: np.ndarray,
    stationary_std: np.ndarray,
    correlation_time_s: float,
    dt_s: float,
    normal: np.ndarray,
) -> np.ndarray:
    phi = math.exp(-dt_s / float(correlation_time_s))
    return phi * previous + stationary_std * math.sqrt(max(0.0, 1.0 - phi * phi)) * normal


def _saturate_and_quantize(
    values: np.ndarray,
    saturation: np.ndarray,
    quantum: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    saturated = np.abs(values) > saturation
    result = np.clip(values, -saturation, saturation)
    nonzero = quantum > 0.0
    result[nonzero] = np.round(result[nonzero] / quantum[nonzero]) * quantum[nonzero]
    return result, saturated


def _rotation_vector_quaternion(rotation_vector: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(rotation_vector))
    if angle <= 1.0e-15:
        return np.array((1.0, 0.0, 0.0, 0.0), dtype=np.float64)
    half_angle = 0.5 * angle
    xyz = rotation_vector * (math.sin(half_angle) / angle)
    return np.array((math.cos(half_angle), xyz[0], xyz[1], xyz[2]), dtype=np.float64)


def _quaternion_multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return np.array(
        (
            lw * rw - lx * rx - ly * ry - lz * rz,
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
        ),
        dtype=np.float64,
    )


__all__ = ["ImuSensorConfig", "ImuSensorModel", "ImuSensorSample"]
