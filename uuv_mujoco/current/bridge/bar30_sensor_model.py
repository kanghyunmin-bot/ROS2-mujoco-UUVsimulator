"""Deterministic, ROS-independent Bar30 pressure error model."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True, slots=True)
class Bar30SensorConfig:
    """Configuration for a Bar30/MS5837-30BA-compatible pressure prior."""

    seed: int = 2630
    nominal_rate_hz: float = 10.0
    reference_pressure_pa: float = 101_325.0
    pressure_scale_factor: float = 1.0
    constant_offset_pa: float = 0.0
    turn_on_offset_std_pa: float = 50.0
    bias_instability_std_pa: float = 25.0
    bias_correlation_time_s: float = 600.0
    random_walk_pa_per_sqrt_s: float = 0.5
    white_noise_std_pa: float = 8.0
    reference_temperature_c: float = 20.0
    default_temperature_c: float = 20.0
    temperature_coefficient_pa_per_c: float = 4.0
    temperature_time_constant_s: float = 30.0
    min_pressure_pa: float = 0.0
    max_pressure_pa: float = 3_000_000.0
    quantization_pa: float = 20.0

    def __post_init__(self) -> None:
        for name in self.__dataclass_fields__:
            if name == "seed":
                continue
            value = float(getattr(self, name))
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
        if isinstance(self.seed, bool) or not isinstance(self.seed, int):
            raise TypeError("seed must be an integer")
        if self.seed < 0:
            raise ValueError("seed must be non-negative")
        for name in (
            "nominal_rate_hz",
            "bias_correlation_time_s",
            "temperature_time_constant_s",
            "max_pressure_pa",
        ):
            if float(getattr(self, name)) <= 0.0:
                raise ValueError(f"{name} must be positive")
        for name in (
            "turn_on_offset_std_pa",
            "bias_instability_std_pa",
            "random_walk_pa_per_sqrt_s",
            "white_noise_std_pa",
            "quantization_pa",
        ):
            if float(getattr(self, name)) < 0.0:
                raise ValueError(f"{name} must be non-negative")
        if self.max_pressure_pa <= self.min_pressure_pa:
            raise ValueError("pressure range must satisfy min_pressure_pa < max_pressure_pa")


@dataclass(frozen=True, slots=True)
class Bar30SensorSample:
    """One sampled absolute-pressure observation."""

    sample_index: int
    sample_time_s: float
    true_pressure_pa: float
    measured_pressure_pa: float
    sensor_temperature_c: float
    total_bias_pa: float
    variance_pa2: float
    saturated: bool


class Bar30SensorModel:
    """Seeded pressure model with offset, drift, thermal response, and ADC effects."""

    def __init__(self, config: Bar30SensorConfig | None = None) -> None:
        self.config = config or Bar30SensorConfig()
        self.reset()

    def reset(self, seed: int | None = None) -> None:
        """Reset stochastic and thermal states for deterministic replay."""

        selected_seed = self.config.seed if seed is None else int(seed)
        self._rng = np.random.default_rng(selected_seed)
        self._turn_on_offset_pa = float(
            self._rng.normal(0.0, self.config.turn_on_offset_std_pa)
        )
        self._bias_instability_pa = 0.0
        self._random_walk_pa = 0.0
        self._sensor_temperature_c = float(self.config.default_temperature_c)
        self._last_sample_time_s: float | None = None
        self._start_sample_time_s: float | None = None
        self._sample_index = 0

    def sample(
        self,
        true_pressure_pa: float,
        *,
        sample_time_s: float,
        ambient_temperature_c: float | None = None,
    ) -> Bar30SensorSample:
        """Generate one pressure observation at ``sample_time_s`` [s]."""

        pressure_pa = float(true_pressure_pa)
        time_s = float(sample_time_s)
        temperature_c = (
            float(self.config.default_temperature_c)
            if ambient_temperature_c is None
            else float(ambient_temperature_c)
        )
        for value, name in (
            (pressure_pa, "true_pressure_pa"),
            (time_s, "sample_time_s"),
            (temperature_c, "ambient_temperature_c"),
        ):
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
        if time_s < 0.0:
            raise ValueError("sample_time_s must be non-negative")
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

        cfg = self.config
        phi = math.exp(-dt_s / cfg.bias_correlation_time_s)
        self._bias_instability_pa = (
            phi * self._bias_instability_pa
            + cfg.bias_instability_std_pa
            * math.sqrt(max(0.0, 1.0 - phi * phi))
            * float(self._rng.normal())
        )
        self._random_walk_pa += (
            cfg.random_walk_pa_per_sqrt_s * math.sqrt(dt_s) * float(self._rng.normal())
        )
        thermal_alpha = 1.0 - math.exp(-dt_s / cfg.temperature_time_constant_s)
        self._sensor_temperature_c += thermal_alpha * (
            temperature_c - self._sensor_temperature_c
        )
        thermal_offset_pa = cfg.temperature_coefficient_pa_per_c * (
            self._sensor_temperature_c - cfg.reference_temperature_c
        )
        total_bias_pa = (
            cfg.constant_offset_pa
            + self._turn_on_offset_pa
            + self._bias_instability_pa
            + self._random_walk_pa
            + thermal_offset_pa
        )
        gauge_pressure_pa = pressure_pa - cfg.reference_pressure_pa
        noisy_pressure_pa = (
            cfg.reference_pressure_pa
            + cfg.pressure_scale_factor * gauge_pressure_pa
            + total_bias_pa
            + cfg.white_noise_std_pa * float(self._rng.normal())
        )
        saturated = (
            noisy_pressure_pa < cfg.min_pressure_pa
            or noisy_pressure_pa > cfg.max_pressure_pa
        )
        measured_pressure_pa = float(
            np.clip(noisy_pressure_pa, cfg.min_pressure_pa, cfg.max_pressure_pa)
        )
        if cfg.quantization_pa > 0.0:
            measured_pressure_pa = (
                round(measured_pressure_pa / cfg.quantization_pa) * cfg.quantization_pa
            )
            measured_pressure_pa = float(
                np.clip(measured_pressure_pa, cfg.min_pressure_pa, cfg.max_pressure_pa)
            )

        elapsed_s = time_s - float(self._start_sample_time_s)
        variance_pa2 = (
            cfg.white_noise_std_pa**2
            + cfg.turn_on_offset_std_pa**2
            + cfg.bias_instability_std_pa**2
            + cfg.random_walk_pa_per_sqrt_s**2 * elapsed_s
            + cfg.quantization_pa**2 / 12.0
        )
        sample = Bar30SensorSample(
            sample_index=self._sample_index,
            sample_time_s=time_s,
            true_pressure_pa=pressure_pa,
            measured_pressure_pa=measured_pressure_pa,
            sensor_temperature_c=float(self._sensor_temperature_c),
            total_bias_pa=float(total_bias_pa),
            variance_pa2=float(variance_pa2),
            saturated=bool(saturated),
        )
        self._sample_index += 1
        return sample


__all__ = ["Bar30SensorConfig", "Bar30SensorModel", "Bar30SensorSample"]
