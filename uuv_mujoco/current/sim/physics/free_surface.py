"""Deterministic shared free-surface geometry and orbital-water contract.

The model intentionally stops at bounded linear wave kinematics.  It provides
one surface definition that buoyancy, immersion, pressure, thruster, and
rendering code can share; it does not inject random or standalone wave forces.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping

import numpy as np


MAX_HARMONICS = 16
MAX_COMPONENT_AMPLITUDE_M = 2.0
MAX_TOTAL_AMPLITUDE_M = 4.0
MAX_FREQUENCY_HZ = 2.0
MIN_WAVENUMBER_RAD_PER_M = 1.0e-6
MAX_WAVENUMBER_RAD_PER_M = 50.0
MAX_ORBITAL_SPEED_MPS = 20.0
MIN_WATER_DEPTH_M = 1.0e-3
MAX_WATER_DEPTH_M = 1.0e4
MAX_GRAVITY_MPS2 = 100.0

_DEFAULT_MAX_ORBITAL_SPEED_MPS = 5.0
_UP_WORLD = np.array([0.0, 0.0, 1.0], dtype=np.float64)
_ZERO_WORLD = np.zeros(3, dtype=np.float64)
_TWO_PI = 2.0 * math.pi


def _finite(value: Any, *, label: str) -> float:
    if isinstance(value, (bool, np.bool_)):
        raise ValueError(f"{label} must be numeric")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be numeric") from exc
    if not math.isfinite(result):
        raise ValueError(f"{label} must be finite")
    return result


def _nonempty_string(value: Any, *, label: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{label} must be a non-empty string")
    return value.strip()


def _vector2(value: Any, *, label: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (2,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{label} must contain two finite numbers")
    result = result.copy()
    result.setflags(write=False)
    return result


@dataclass(frozen=True)
class HarmonicWave:
    """One deterministic linear-wave component in world coordinates."""

    amplitude_m: float
    frequency_hz: float
    wave_vector_rad_per_m: np.ndarray
    phase_rad: float

    @property
    def angular_frequency_rad_per_s(self) -> float:
        """Return this component's angular frequency [rad/s]."""

        return _TWO_PI * self.frequency_hz

    @property
    def wavenumber_rad_per_m(self) -> float:
        """Return the horizontal wavenumber magnitude [rad/m]."""

        return float(np.linalg.norm(self.wave_vector_rad_per_m))


@dataclass(frozen=True)
class FreeSurfaceConfig:
    """Validated free-surface parameters."""

    mode: str
    reference_height_world_m: float
    calibration_status: str
    provenance: str
    wave_kinematics: str
    water_depth_m: float
    gravity_mps2: float
    harmonics: tuple[HarmonicWave, ...]
    max_orbital_speed_mps: float

    @property
    def active(self) -> bool:
        """Return whether the air-water boundary is active."""

        return self.mode != "disabled"


@dataclass(frozen=True)
class FreeSurfaceSample:
    """Surface geometry and water motion evaluated at one world point."""

    height_world_m: float
    normal_world: np.ndarray
    orbital_velocity_world_mps: np.ndarray


def parse_free_surface_config(value: Mapping[str, Any] | None) -> FreeSurfaceConfig:
    """Parse a free-surface mapping into a bounded immutable configuration.

    Supported modes are ``disabled``, ``flat``, and ``harmonic``.  Disabled is
    the exact no-effect default.  Flat and harmonic modes require explicit
    calibration status and provenance so an unmeasured prior cannot silently
    appear to be calibrated evidence.
    """

    if value is None:
        return _disabled_config()
    if not isinstance(value, Mapping):
        raise ValueError("free_surface must be an object")

    raw_mode = value.get("mode", "disabled")
    if not isinstance(raw_mode, str):
        raise ValueError("free_surface.mode must be a string")
    mode = raw_mode.strip().lower()
    if mode not in {"disabled", "flat", "harmonic"}:
        raise ValueError("free_surface.mode must be disabled, flat, or harmonic")
    if mode == "disabled":
        return _disabled_config()

    calibration_status = _nonempty_string(
        value.get("calibration_status"),
        label="free_surface.calibration_status",
    )
    provenance = _nonempty_string(
        value.get("provenance"),
        label="free_surface.provenance",
    )
    reference_height = _finite(
        value.get("reference_height_world_m", 0.0),
        label="free_surface.reference_height_world_m",
    )
    max_orbital_speed = _finite(
        value.get("max_orbital_speed_mps", _DEFAULT_MAX_ORBITAL_SPEED_MPS),
        label="free_surface.max_orbital_speed_mps",
    )
    if not 0.0 < max_orbital_speed <= MAX_ORBITAL_SPEED_MPS:
        raise ValueError(
            "free_surface.max_orbital_speed_mps must be in "
            f"(0, {MAX_ORBITAL_SPEED_MPS}]"
        )

    raw_wave_kinematics = value.get("wave_kinematics", "prescribed")
    if not isinstance(raw_wave_kinematics, str):
        raise ValueError("free_surface.wave_kinematics must be a string")
    wave_kinematics = raw_wave_kinematics.strip().lower()
    if wave_kinematics not in {"prescribed", "finite_depth_gravity"}:
        raise ValueError(
            "free_surface.wave_kinematics must be prescribed or "
            "finite_depth_gravity"
        )
    water_depth_m = 0.0
    gravity_mps2 = 9.80665
    if wave_kinematics == "finite_depth_gravity":
        water_depth_m = _finite(
            value.get("water_depth_m"),
            label="free_surface.water_depth_m",
        )
        gravity_mps2 = _finite(
            value.get("gravity_mps2", gravity_mps2),
            label="free_surface.gravity_mps2",
        )
        if not MIN_WATER_DEPTH_M <= water_depth_m <= MAX_WATER_DEPTH_M:
            raise ValueError(
                "free_surface.water_depth_m must be in "
                f"[{MIN_WATER_DEPTH_M:g}, {MAX_WATER_DEPTH_M:g}]"
            )
        if not 0.0 < gravity_mps2 <= MAX_GRAVITY_MPS2:
            raise ValueError(
                f"free_surface.gravity_mps2 must be in (0, {MAX_GRAVITY_MPS2:g}]"
            )

    raw_harmonics = value.get("harmonics", ())
    if not isinstance(raw_harmonics, (list, tuple)):
        raise ValueError("free_surface.harmonics must be an array")
    if mode == "flat" and raw_harmonics:
        raise ValueError("flat free_surface cannot define harmonics")
    harmonics = _parse_harmonics(
        raw_harmonics,
        wave_kinematics=wave_kinematics,
        water_depth_m=water_depth_m,
        gravity_mps2=gravity_mps2,
    )
    if mode == "harmonic" and not harmonics:
        raise ValueError("harmonic free_surface requires at least one harmonic")

    return FreeSurfaceConfig(
        mode=mode,
        reference_height_world_m=reference_height,
        calibration_status=calibration_status,
        provenance=provenance,
        wave_kinematics=wave_kinematics,
        water_depth_m=water_depth_m,
        gravity_mps2=gravity_mps2,
        harmonics=harmonics,
        max_orbital_speed_mps=max_orbital_speed,
    )


class FreeSurface:
    """Evaluate one shared deterministic air-water boundary."""

    def __init__(self, config: FreeSurfaceConfig) -> None:
        self.config = config

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any] | None) -> "FreeSurface":
        """Construct from a direct free-surface configuration mapping."""

        return cls(parse_free_surface_config(value))

    @classmethod
    def from_profile(
        cls,
        sim_profile: Mapping[str, Any],
        *,
        reference_height_world_m: float | None = None,
    ) -> "FreeSurface":
        """Construct from a profile and optional runtime waterline override."""

        if not isinstance(sim_profile, Mapping):
            raise ValueError("sim_profile must be an object")
        payload = sim_profile.get("free_surface")
        if reference_height_world_m is not None and isinstance(payload, Mapping):
            payload = dict(payload)
            payload["reference_height_world_m"] = _finite(
                reference_height_world_m,
                label="reference_height_world_m",
            )
        return cls.from_mapping(payload)

    @property
    def active(self) -> bool:
        """Return whether the air-water boundary is active."""

        return self.config.active

    @property
    def mode(self) -> str:
        """Return ``disabled``, ``flat``, or ``harmonic``."""

        return self.config.mode

    def height_world_m(self, x_world_m: float, y_world_m: float, time_s: float) -> float:
        """Return the free-surface world-z height [m]."""

        x, y, time_value = _query_coordinates(x_world_m, y_world_m, time_s)
        cfg = self.config
        if cfg.mode != "harmonic":
            return cfg.reference_height_world_m

        height = cfg.reference_height_world_m
        for wave in cfg.harmonics:
            height += wave.amplitude_m * math.cos(_phase(wave, x, y, time_value))
        return float(height)

    def surface_height_and_normal(
        self,
        x_world_m: float,
        y_world_m: float,
        time_s: float,
    ) -> tuple[float, np.ndarray]:
        """Return surface world-z height [m] and upward world-frame unit normal."""

        x, y, time_value = _query_coordinates(x_world_m, y_world_m, time_s)
        cfg = self.config
        if cfg.mode != "harmonic":
            return cfg.reference_height_world_m, _UP_WORLD.copy()

        height = cfg.reference_height_world_m
        gradient_xy = np.zeros(2, dtype=np.float64)
        for wave in cfg.harmonics:
            phase = _phase(wave, x, y, time_value)
            height += wave.amplitude_m * math.cos(phase)
            gradient_xy -= (
                wave.amplitude_m
                * math.sin(phase)
                * wave.wave_vector_rad_per_m
            )
        normal = np.array([-gradient_xy[0], -gradient_xy[1], 1.0], dtype=np.float64)
        normal /= float(np.linalg.norm(normal))
        return float(height), normal

    def normal_world(self, x_world_m: float, y_world_m: float, time_s: float) -> np.ndarray:
        """Return the upward free-surface unit normal in world coordinates."""

        return self.surface_height_and_normal(x_world_m, y_world_m, time_s)[1]

    def orbital_velocity_world_mps(
        self,
        x_world_m: float,
        y_world_m: float,
        z_world_m: float,
        time_s: float,
    ) -> np.ndarray:
        """Return bounded linear-wave orbital water velocity [m/s].

        Prescribed waves use deep-water exponential decay. Finite-depth
        gravity waves satisfy the dispersion relation and bottom boundary.
        Velocity is zero for disabled/flat surfaces and at dry points above
        the instantaneous harmonic surface.
        """

        x, y, time_value = _query_coordinates(x_world_m, y_world_m, time_s)
        z = _finite(z_world_m, label="z_world_m")
        cfg = self.config
        if cfg.mode != "harmonic":
            return _ZERO_WORLD.copy()

        height = self.height_world_m(x, y, time_value)
        if z > height:
            return _ZERO_WORLD.copy()

        velocity = np.zeros(3, dtype=np.float64)
        relative_z = min(z - cfg.reference_height_world_m, 0.0)
        for wave in cfg.harmonics:
            wavenumber = wave.wavenumber_rad_per_m
            if cfg.wave_kinematics == "finite_depth_gravity":
                horizontal_factor, vertical_factor = _finite_depth_factors(
                    wavenumber,
                    relative_z,
                    cfg.water_depth_m,
                )
            else:
                attenuation = math.exp(wavenumber * relative_z)
                horizontal_factor = attenuation
                vertical_factor = attenuation
            speed_amplitude = wave.amplitude_m * wave.angular_frequency_rad_per_s
            phase = _phase(wave, x, y, time_value)
            direction_xy = wave.wave_vector_rad_per_m / wavenumber
            velocity[:2] += (
                speed_amplitude
                * horizontal_factor
                * math.cos(phase)
                * direction_xy
            )
            velocity[2] += speed_amplitude * vertical_factor * math.sin(phase)

        if not np.all(np.isfinite(velocity)):
            raise FloatingPointError("free-surface orbital velocity is non-finite")
        speed = math.hypot(float(velocity[0]), float(velocity[1]), float(velocity[2]))
        if not math.isfinite(speed):
            raise FloatingPointError("free-surface orbital speed is non-finite")
        if speed > cfg.max_orbital_speed_mps:
            velocity *= cfg.max_orbital_speed_mps / speed
        return velocity

    def sample(
        self,
        x_world_m: float,
        y_world_m: float,
        z_world_m: float,
        time_s: float,
    ) -> FreeSurfaceSample:
        """Return height, normal, and orbital velocity at ``(x, y, z, t)``."""

        height, normal = self.surface_height_and_normal(x_world_m, y_world_m, time_s)
        velocity = self.orbital_velocity_world_mps(
            x_world_m,
            y_world_m,
            z_world_m,
            time_s,
        )
        return FreeSurfaceSample(
            height_world_m=height,
            normal_world=normal,
            orbital_velocity_world_mps=velocity,
        )


def _disabled_config() -> FreeSurfaceConfig:
    return FreeSurfaceConfig(
        mode="disabled",
        reference_height_world_m=0.0,
        calibration_status="disabled",
        provenance="disabled",
        wave_kinematics="prescribed",
        water_depth_m=0.0,
        gravity_mps2=9.80665,
        harmonics=(),
        max_orbital_speed_mps=_DEFAULT_MAX_ORBITAL_SPEED_MPS,
    )


def _parse_harmonics(
    value: list[Any] | tuple[Any, ...],
    *,
    wave_kinematics: str,
    water_depth_m: float,
    gravity_mps2: float,
) -> tuple[HarmonicWave, ...]:
    if len(value) > MAX_HARMONICS:
        raise ValueError(f"free_surface supports at most {MAX_HARMONICS} harmonics")
    harmonics: list[HarmonicWave] = []
    total_amplitude = 0.0
    for index, raw in enumerate(value):
        label = f"free_surface.harmonics[{index}]"
        if not isinstance(raw, Mapping):
            raise ValueError(f"{label} must be an object")
        amplitude = _finite(raw.get("amplitude_m"), label=f"{label}.amplitude_m")
        if not 0.0 < amplitude <= MAX_COMPONENT_AMPLITUDE_M:
            raise ValueError(
                f"{label}.amplitude_m must be in (0, {MAX_COMPONENT_AMPLITUDE_M}]"
            )
        total_amplitude += amplitude
        if total_amplitude > MAX_TOTAL_AMPLITUDE_M:
            raise ValueError(
                "free_surface total harmonic amplitude must not exceed "
                f"{MAX_TOTAL_AMPLITUDE_M} m"
            )

        wave_vector = _vector2(
            raw.get("wave_vector_rad_per_m"),
            label=f"{label}.wave_vector_rad_per_m",
        )
        wavenumber = float(np.linalg.norm(wave_vector))
        if not MIN_WAVENUMBER_RAD_PER_M <= wavenumber <= MAX_WAVENUMBER_RAD_PER_M:
            raise ValueError(
                f"{label} wavenumber magnitude must be in "
                f"[{MIN_WAVENUMBER_RAD_PER_M}, {MAX_WAVENUMBER_RAD_PER_M}] rad/m"
            )
        if wave_kinematics == "finite_depth_gravity":
            frequency = math.sqrt(
                gravity_mps2
                * wavenumber
                * math.tanh(wavenumber * water_depth_m)
            ) / _TWO_PI
            if "frequency_hz" in raw:
                supplied_frequency = _finite(
                    raw.get("frequency_hz"),
                    label=f"{label}.frequency_hz",
                )
                if not math.isclose(
                    supplied_frequency,
                    frequency,
                    rel_tol=0.02,
                    abs_tol=1.0e-6,
                ):
                    raise ValueError(
                        f"{label}.frequency_hz violates finite-depth gravity "
                        f"dispersion; expected {frequency:.6g} Hz"
                    )
        else:
            frequency = _finite(
                raw.get("frequency_hz"),
                label=f"{label}.frequency_hz",
            )
        if not 0.0 < frequency <= MAX_FREQUENCY_HZ:
            raise ValueError(f"{label}.frequency_hz must be in (0, {MAX_FREQUENCY_HZ}]")
        phase = _finite(raw.get("phase_rad", 0.0), label=f"{label}.phase_rad")
        harmonics.append(
            HarmonicWave(
                amplitude_m=amplitude,
                frequency_hz=frequency,
                wave_vector_rad_per_m=wave_vector,
                phase_rad=phase,
            )
        )
    return tuple(harmonics)


def _finite_depth_factors(
    wavenumber_rad_per_m: float,
    relative_z_m: float,
    water_depth_m: float,
) -> tuple[float, float]:
    """Return finite-depth horizontal and vertical orbital factors."""

    kh = wavenumber_rad_per_m * water_depth_m
    if kh > 50.0:
        attenuation = math.exp(wavenumber_rad_per_m * relative_z_m)
        return attenuation, attenuation
    height_above_bottom = float(
        np.clip(relative_z_m + water_depth_m, 0.0, water_depth_m)
    )
    kz = wavenumber_rad_per_m * height_above_bottom
    denominator = math.sinh(kh)
    if not math.isfinite(denominator) or denominator <= 0.0:
        raise FloatingPointError("finite-depth wave normalization is invalid")
    horizontal = math.cosh(kz) / denominator
    vertical = math.sinh(kz) / denominator
    if not math.isfinite(horizontal) or not math.isfinite(vertical):
        raise FloatingPointError("finite-depth orbital factors are non-finite")
    return horizontal, vertical


def _query_coordinates(x_world_m: Any, y_world_m: Any, time_s: Any) -> tuple[float, float, float]:
    return (
        _finite(x_world_m, label="x_world_m"),
        _finite(y_world_m, label="y_world_m"),
        _finite(time_s, label="time_s"),
    )


def _phase(wave: HarmonicWave, x: float, y: float, time_s: float) -> float:
    spatial_phase = float(wave.wave_vector_rad_per_m @ np.array([x, y], dtype=np.float64))
    phase = spatial_phase - wave.angular_frequency_rad_per_s * time_s + wave.phase_rad
    return math.remainder(phase, _TWO_PI)


__all__ = [
    "FreeSurface",
    "FreeSurfaceConfig",
    "FreeSurfaceSample",
    "HarmonicWave",
    "MAX_COMPONENT_AMPLITUDE_M",
    "MAX_FREQUENCY_HZ",
    "MAX_HARMONICS",
    "MAX_ORBITAL_SPEED_MPS",
    "MAX_TOTAL_AMPLITUDE_M",
    "MAX_WAVENUMBER_RAD_PER_M",
    "MIN_WATER_DEPTH_M",
    "MIN_WAVENUMBER_RAD_PER_M",
    "parse_free_surface_config",
]
