"""Deterministic, ROS-independent four-beam DVL-A50 error model.

The model operates entirely in the DVL forward-right-down (FRD) frame.  It is
intended to sit between simulator truth and a ROS message builder, but it does
not import ROS or MuJoCo.

Water Linked publishes the A50's four-beam convex Janus geometry, 22.5 degree
beam angle, and 0.05--50 m ideal altitude range.  Its TCP JSON protocol defines
per-transducer velocity and distance in m/s and m, while RSSI and NSD are both
in dBm.  Those facts and units are reflected directly here.  The configured
noise distributions, signal-loss curve, dropout probabilities, and conversion
from acoustic diagnostics to velocity uncertainty are uncalibrated simulation
priors; they must be identified from bags for a particular sensor and site.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Collection, Sequence

import numpy as np


Vector3 = tuple[float, float, float]
Vector4 = tuple[float, float, float, float]


def _float_tuple(values: Sequence[float], length: int, name: str) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != length:
        raise ValueError(f"{name} must contain {length} values")
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must contain only finite values")
    return result


@dataclass(frozen=True)
class A50SensorConfig:
    """Configuration for :class:`A50SensorModel`.

    The geometry and operating range defaults are published A50 values.  RSSI
    and NSD use the dBm units defined by the Water Linked protocol.  All noise,
    signal-level, quality, and dropout defaults are uncalibrated priors.
    """

    seed: int = 2608
    beam_tilt_deg: float = 22.5
    beam_azimuths_deg: Vector4 = (45.0, 135.0, 225.0, 315.0)
    min_range_m: float = 0.05
    max_range_m: float = 50.0
    range_attenuation_m: float = 25.0
    min_incidence_cosine: float = 0.15
    incidence_exponent: float = 1.5
    min_quality: float = 0.01
    noise_quality_floor: float = 0.02
    velocity_bias_frd_mps: Vector3 = (0.0, 0.0, 0.0)
    beam_velocity_bias_mps: Vector4 = (0.0, 0.0, 0.0, 0.0)
    white_noise_std_mps: float = 0.002
    velocity_noise_per_meter_mps: float = 0.0001
    range_noise_std_m: float = 0.005
    range_noise_fraction: float = 0.0005
    rssi_at_1m_dbm: float = -40.0
    rssi_min_dbm: float = -120.0
    rssi_max_dbm: float = -10.0
    nsd_mean_dbm: float = -94.0
    nsd_std_db: float = 3.0
    nsd_min_dbm: float = -120.0
    nsd_max_dbm: float = -60.0
    beam_dropout_probabilities: Vector4 = (0.0, 0.0, 0.0, 0.0)
    min_valid_beams: int = 3
    transmission_delay_us: int = 2_000

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "beam_azimuths_deg",
            _float_tuple(self.beam_azimuths_deg, 4, "beam_azimuths_deg"),
        )
        object.__setattr__(
            self,
            "velocity_bias_frd_mps",
            _float_tuple(self.velocity_bias_frd_mps, 3, "velocity_bias_frd_mps"),
        )
        object.__setattr__(
            self,
            "beam_velocity_bias_mps",
            _float_tuple(self.beam_velocity_bias_mps, 4, "beam_velocity_bias_mps"),
        )
        object.__setattr__(
            self,
            "beam_dropout_probabilities",
            _float_tuple(
                self.beam_dropout_probabilities,
                4,
                "beam_dropout_probabilities",
            ),
        )

        finite_fields = (
            "beam_tilt_deg",
            "min_range_m",
            "max_range_m",
            "range_attenuation_m",
            "min_incidence_cosine",
            "incidence_exponent",
            "min_quality",
            "noise_quality_floor",
            "white_noise_std_mps",
            "velocity_noise_per_meter_mps",
            "range_noise_std_m",
            "range_noise_fraction",
            "rssi_at_1m_dbm",
            "rssi_min_dbm",
            "rssi_max_dbm",
            "nsd_mean_dbm",
            "nsd_std_db",
            "nsd_min_dbm",
            "nsd_max_dbm",
        )
        for name in finite_fields:
            if not math.isfinite(float(getattr(self, name))):
                raise ValueError(f"{name} must be finite")
        if not 0.0 < self.beam_tilt_deg < 90.0:
            raise ValueError("beam_tilt_deg must be between 0 and 90 degrees")
        if self.min_range_m < 0.0 or self.max_range_m <= self.min_range_m:
            raise ValueError("range limits must satisfy 0 <= min_range_m < max_range_m")
        if self.range_attenuation_m <= 0.0:
            raise ValueError("range_attenuation_m must be positive")
        if not 0.0 <= self.min_incidence_cosine < 1.0:
            raise ValueError("min_incidence_cosine must be in [0, 1)")
        if self.incidence_exponent <= 0.0:
            raise ValueError("incidence_exponent must be positive")
        if not 0.0 <= self.min_quality <= 1.0:
            raise ValueError("min_quality must be in [0, 1]")
        if not 0.0 < self.noise_quality_floor <= 1.0:
            raise ValueError("noise_quality_floor must be in (0, 1]")
        nonnegative_fields = (
            "white_noise_std_mps",
            "velocity_noise_per_meter_mps",
            "range_noise_std_m",
            "range_noise_fraction",
            "nsd_std_db",
        )
        for name in nonnegative_fields:
            if float(getattr(self, name)) < 0.0:
                raise ValueError(f"{name} must be non-negative")
        if self.rssi_max_dbm <= self.rssi_min_dbm:
            raise ValueError("rssi_max_dbm must be greater than rssi_min_dbm")
        if not self.rssi_min_dbm <= self.rssi_at_1m_dbm <= self.rssi_max_dbm:
            raise ValueError("rssi_at_1m_dbm must be within the RSSI clamp")
        if self.nsd_max_dbm <= self.nsd_min_dbm:
            raise ValueError("nsd_max_dbm must be greater than nsd_min_dbm")
        if not self.nsd_min_dbm <= self.nsd_mean_dbm <= self.nsd_max_dbm:
            raise ValueError("nsd_mean_dbm must be within the NSD clamp")
        if not all(0.0 <= probability <= 1.0 for probability in self.beam_dropout_probabilities):
            raise ValueError("beam dropout probabilities must be in [0, 1]")
        if not 3 <= int(self.min_valid_beams) <= 4:
            raise ValueError("min_valid_beams must be 3 or 4 for a 3-D solution")
        if int(self.transmission_delay_us) < 0:
            raise ValueError("transmission_delay_us must be non-negative")


@dataclass(frozen=True)
class A50BeamSample:
    """One simulated bottom-track beam observation.

    ``rssi_dbm`` and ``nsd_dbm`` map to the Water Linked transducer fields.
    ``velocity_std_mps`` is the separate modeled radial-velocity uncertainty
    used for reconstruction and must not be copied into the ROS ``nsd`` field.
    """

    beam_id: int
    azimuth_deg: float
    tilt_deg: float
    direction_frd: Vector3
    true_radial_velocity_mps: float
    measured_radial_velocity_mps: float | None
    true_range_m: float | None
    measured_range_m: float | None
    incidence_cosine: float
    quality: float
    rssi_dbm: float
    nsd_dbm: float
    velocity_std_mps: float
    valid: bool
    dropout_reason: str | None


@dataclass(frozen=True)
class A50SensorSample:
    """A four-beam observation and its reconstructed DVL velocity."""

    sample_index: int
    true_velocity_frd_mps: Vector3
    measured_velocity_frd_mps: Vector3 | None
    covariance_frd_mps2: tuple[float, ...] | None
    altitude_estimate_m: float | None
    beams: tuple[A50BeamSample, ...]
    valid_beam_count: int
    velocity_valid: bool
    fom_mps: float
    time_of_validity_us: int
    time_of_transmission_us: int

    @property
    def velocity_error_frd_mps(self) -> Vector3 | None:
        """Return measured minus true velocity [m/s], if velocity is valid."""

        if self.measured_velocity_frd_mps is None:
            return None
        return tuple(
            measured - truth
            for measured, truth in zip(
                self.measured_velocity_frd_mps,
                self.true_velocity_frd_mps,
            )
        )


def beam_directions_frd(config: A50SensorConfig | None = None) -> tuple[Vector3, ...]:
    """Return unit beam directions in forward-right-down coordinates."""

    cfg = config or A50SensorConfig()
    tilt_rad = math.radians(cfg.beam_tilt_deg)
    horizontal = math.sin(tilt_rad)
    down = math.cos(tilt_rad)
    return tuple(
        (
            horizontal * math.cos(math.radians(azimuth_deg)),
            horizontal * math.sin(math.radians(azimuth_deg)),
            down,
        )
        for azimuth_deg in cfg.beam_azimuths_deg
    )


class A50SensorModel:
    """Seeded four-beam bottom-track velocity and quality model."""

    def __init__(self, config: A50SensorConfig | None = None) -> None:
        self.config = config or A50SensorConfig()
        self._directions = np.asarray(beam_directions_frd(self.config), dtype=np.float64)
        self.reset()

    @property
    def directions_frd(self) -> tuple[Vector3, ...]:
        """Return the model's immutable beam geometry."""

        return tuple(tuple(float(value) for value in row) for row in self._directions)

    def reset(self, seed: int | None = None) -> None:
        """Reset the deterministic random sequence and sample counter."""

        self._rng = np.random.default_rng(self.config.seed if seed is None else int(seed))
        self._sample_index = 0

    def sample(
        self,
        velocity_frd_mps: Sequence[float],
        altitude_m: float | None,
        *,
        time_of_validity_us: int,
        time_of_transmission_us: int | None = None,
        bottom_normal_frd: Sequence[float] = (0.0, 0.0, -1.0),
        beam_ranges_m: Sequence[float | None] | None = None,
        incidence_cosines: Sequence[float] | None = None,
        forced_dropout_beams: Collection[int] = (),
    ) -> A50SensorSample:
        """Generate one deterministic A50-style sample.

        Args:
            velocity_frd_mps: True DVL-frame velocity [m/s].
            altitude_m: Vertical distance to the plane below the sensor [m].
                It may be ``None`` when explicit ``beam_ranges_m`` are given.
            time_of_validity_us: Center-of-ping/sample time [Unix or simulation us].
            time_of_transmission_us: Report transmission time [us].  When
                omitted, :attr:`A50SensorConfig.transmission_delay_us` is added.
            bottom_normal_frd: Unit-normal direction of a planar bottom.  Sign
                is irrelevant; incidence uses its absolute dot product.
            beam_ranges_m: Optional raycast ranges [m], one per beam.
            incidence_cosines: Optional incidence cosines, one per beam.
            forced_dropout_beams: Beam IDs to suppress for deterministic
                fault scenarios, in addition to configured random dropout.

        Returns:
            A complete ROS-independent beam and velocity observation.
        """

        truth_velocity = np.asarray(
            _float_tuple(velocity_frd_mps, 3, "velocity_frd_mps"),
            dtype=np.float64,
        )
        validity_us = int(time_of_validity_us)
        if validity_us < 0:
            raise ValueError("time_of_validity_us must be non-negative")
        transmission_us = (
            validity_us + int(self.config.transmission_delay_us)
            if time_of_transmission_us is None
            else int(time_of_transmission_us)
        )
        if transmission_us < validity_us:
            raise ValueError("time_of_transmission_us cannot precede validity time")

        normal = np.asarray(_float_tuple(bottom_normal_frd, 3, "bottom_normal_frd"))
        normal_norm = float(np.linalg.norm(normal))
        if normal_norm <= 1.0e-12:
            raise ValueError("bottom_normal_frd must have non-zero length")
        normal /= normal_norm

        ranges, incidences = self._resolve_geometry(
            altitude_m,
            normal,
            beam_ranges_m=beam_ranges_m,
            incidence_cosines=incidence_cosines,
        )
        forced = {int(beam_id) for beam_id in forced_dropout_beams}
        if any(beam_id < 0 or beam_id >= 4 for beam_id in forced):
            raise ValueError("forced dropout beam IDs must be in [0, 3]")

        # Draw fixed-size arrays before evaluating validity so a seed describes
        # one stable sequence even when geometry makes different beams invalid.
        dropout_draws = self._rng.random(4)
        nsd_standard_normals = self._rng.normal(size=4)
        velocity_standard_normals = self._rng.normal(size=4)
        range_standard_normals = self._rng.normal(size=4)

        biased_velocity = truth_velocity + np.asarray(
            self.config.velocity_bias_frd_mps,
            dtype=np.float64,
        )
        beam_samples: list[A50BeamSample] = []
        for beam_id, direction in enumerate(self._directions):
            true_range = ranges[beam_id]
            incidence = incidences[beam_id]
            quality = self._quality(true_range, incidence)
            nsd_dbm = self._nsd_dbm(float(nsd_standard_normals[beam_id]))
            velocity_std = self._velocity_std(true_range, quality, nsd_dbm)
            rssi_dbm = self._rssi_dbm(true_range, incidence)
            reason = self._dropout_reason(
                beam_id,
                true_range,
                incidence,
                quality,
                forced,
                float(dropout_draws[beam_id]),
            )
            true_radial = float(np.dot(truth_velocity, direction))
            measured_radial: float | None = None
            measured_range: float | None = None
            if reason is None:
                measured_radial = (
                    float(np.dot(biased_velocity, direction))
                    + float(self.config.beam_velocity_bias_mps[beam_id])
                    + velocity_std * float(velocity_standard_normals[beam_id])
                )
                range_sigma = self._range_sigma(float(true_range), quality)
                measured_range = max(
                    0.0,
                    float(true_range) + range_sigma * float(range_standard_normals[beam_id]),
                )
            beam_samples.append(
                A50BeamSample(
                    beam_id=beam_id,
                    azimuth_deg=float(self.config.beam_azimuths_deg[beam_id]),
                    tilt_deg=float(self.config.beam_tilt_deg),
                    direction_frd=tuple(float(value) for value in direction),
                    true_radial_velocity_mps=true_radial,
                    measured_radial_velocity_mps=measured_radial,
                    true_range_m=true_range,
                    measured_range_m=measured_range,
                    incidence_cosine=incidence,
                    quality=quality,
                    rssi_dbm=rssi_dbm,
                    nsd_dbm=nsd_dbm,
                    velocity_std_mps=velocity_std,
                    valid=reason is None,
                    dropout_reason=reason,
                )
            )

        measured_velocity, covariance, fom = self._solve_velocity(beam_samples)
        valid_count = sum(beam.valid for beam in beam_samples)
        velocity_valid = measured_velocity is not None
        altitude_estimate = self._estimate_altitude(beam_samples)
        result = A50SensorSample(
            sample_index=self._sample_index,
            true_velocity_frd_mps=tuple(float(value) for value in truth_velocity),
            measured_velocity_frd_mps=(
                None
                if measured_velocity is None
                else tuple(float(value) for value in measured_velocity)
            ),
            covariance_frd_mps2=(
                None
                if covariance is None
                else tuple(float(value) for value in covariance.reshape(-1))
            ),
            altitude_estimate_m=altitude_estimate,
            beams=tuple(beam_samples),
            valid_beam_count=valid_count,
            velocity_valid=velocity_valid,
            fom_mps=fom,
            time_of_validity_us=validity_us,
            time_of_transmission_us=transmission_us,
        )
        self._sample_index += 1
        return result

    def _resolve_geometry(
        self,
        altitude_m: float | None,
        normal: np.ndarray,
        *,
        beam_ranges_m: Sequence[float | None] | None,
        incidence_cosines: Sequence[float] | None,
    ) -> tuple[tuple[float | None, ...], tuple[float, ...]]:
        if beam_ranges_m is not None:
            if len(beam_ranges_m) != 4:
                raise ValueError("beam_ranges_m must contain 4 values")
            ranges = tuple(
                None if value is None else float(value)
                for value in beam_ranges_m
            )
            if any(value is not None and not math.isfinite(value) for value in ranges):
                raise ValueError("beam_ranges_m values must be finite or None")
        else:
            if altitude_m is None or not math.isfinite(float(altitude_m)) or altitude_m <= 0.0:
                ranges = (None, None, None, None)
            else:
                plane_point = np.array((0.0, 0.0, float(altitude_m)), dtype=np.float64)
                numerator = float(np.dot(normal, plane_point))
                computed: list[float | None] = []
                for direction in self._directions:
                    denominator = float(np.dot(normal, direction))
                    distance = numerator / denominator if abs(denominator) > 1.0e-12 else -1.0
                    computed.append(
                        distance
                        if distance > 0.0 and math.isfinite(distance)
                        else None
                    )
                ranges = tuple(computed)

        if incidence_cosines is not None:
            incidences = _float_tuple(incidence_cosines, 4, "incidence_cosines")
            if any(value < 0.0 or value > 1.0 for value in incidences):
                raise ValueError("incidence cosines must be in [0, 1]")
        else:
            incidences = tuple(
                min(1.0, abs(float(np.dot(normal, direction))))
                for direction in self._directions
            )
        return ranges, incidences

    def _quality(self, distance_m: float | None, incidence: float) -> float:
        if distance_m is None or distance_m < self.config.min_range_m:
            return 0.0
        if distance_m > self.config.max_range_m or incidence <= self.config.min_incidence_cosine:
            return 0.0
        range_quality = math.exp(
            -2.0 * max(0.0, distance_m - self.config.min_range_m)
            / self.config.range_attenuation_m
        )
        normalized_incidence = (
            (incidence - self.config.min_incidence_cosine)
            / (1.0 - self.config.min_incidence_cosine)
        )
        incidence_quality = normalized_incidence ** self.config.incidence_exponent
        return float(np.clip(range_quality * incidence_quality, 0.0, 1.0))

    def _velocity_std(
        self,
        distance_m: float | None,
        quality: float,
        nsd_dbm: float,
    ) -> float:
        if distance_m is None:
            return math.inf
        baseline = (
            self.config.white_noise_std_mps
            + self.config.velocity_noise_per_meter_mps * max(distance_m, 0.0)
        )
        quality_scale = 1.0 / math.sqrt(
            max(quality, self.config.noise_quality_floor)
        )
        # NSD is an acoustic power density, not a velocity deviation.  This
        # amplitude-ratio coupling is deliberately only a prior until bag data
        # is available to identify the real relationship.
        nsd_amplitude_scale = 10.0 ** (
            (nsd_dbm - self.config.nsd_mean_dbm) / 20.0
        )
        return baseline * quality_scale * nsd_amplitude_scale

    def _nsd_dbm(self, standard_normal: float) -> float:
        value = self.config.nsd_mean_dbm + self.config.nsd_std_db * standard_normal
        return float(np.clip(value, self.config.nsd_min_dbm, self.config.nsd_max_dbm))

    def _range_sigma(self, distance_m: float, quality: float) -> float:
        baseline = self.config.range_noise_std_m + self.config.range_noise_fraction * distance_m
        return baseline / math.sqrt(max(quality, self.config.noise_quality_floor))

    def _rssi_dbm(self, distance_m: float | None, incidence: float) -> float:
        if distance_m is None or distance_m <= 0.0 or incidence <= 0.0:
            return float(self.config.rssi_min_dbm)
        # Forty dB/decade approximates two-way spherical spreading.  The
        # incidence term represents a weaker projected bottom return.
        value = (
            self.config.rssi_at_1m_dbm
            - 40.0 * math.log10(max(distance_m, 1.0e-6))
            + 20.0 * math.log10(max(incidence, 1.0e-6))
        )
        return float(np.clip(value, self.config.rssi_min_dbm, self.config.rssi_max_dbm))

    def _dropout_reason(
        self,
        beam_id: int,
        distance_m: float | None,
        incidence: float,
        quality: float,
        forced: set[int],
        dropout_draw: float,
    ) -> str | None:
        if distance_m is None:
            return "no_return"
        if distance_m < self.config.min_range_m or distance_m > self.config.max_range_m:
            return "range"
        if incidence <= self.config.min_incidence_cosine:
            return "incidence"
        if quality < self.config.min_quality:
            return "quality"
        if beam_id in forced:
            return "forced"
        if dropout_draw < self.config.beam_dropout_probabilities[beam_id]:
            return "random"
        return None

    def _solve_velocity(
        self,
        beams: Sequence[A50BeamSample],
    ) -> tuple[np.ndarray | None, np.ndarray | None, float]:
        valid = [beam for beam in beams if beam.valid]
        if len(valid) < self.config.min_valid_beams:
            return None, None, math.inf
        design = np.asarray([beam.direction_frd for beam in valid], dtype=np.float64)
        if np.linalg.matrix_rank(design) < 3:
            return None, None, math.inf
        radial = np.asarray(
            [beam.measured_radial_velocity_mps for beam in valid],
            dtype=np.float64,
        )
        sigma = np.asarray(
            [max(beam.velocity_std_mps, 1.0e-12) for beam in valid]
        )
        weights = 1.0 / np.square(sigma)
        information = design.T @ (weights[:, None] * design)
        covariance = np.linalg.inv(information)
        estimate = covariance @ design.T @ (weights * radial)
        residual = radial - design @ estimate
        residual_rms = float(math.sqrt(float(np.mean(np.square(residual)))))
        largest_variance = float(max(np.linalg.eigvalsh(covariance)))
        fom = math.sqrt(max(largest_variance, 0.0)) + residual_rms
        return estimate, covariance, fom

    def _estimate_altitude(self, beams: Sequence[A50BeamSample]) -> float | None:
        vertical_ranges = [
            float(beam.measured_range_m) * float(beam.direction_frd[2])
            for beam in beams
            if beam.valid and beam.measured_range_m is not None
        ]
        if not vertical_ranges:
            return None
        return float(np.median(np.asarray(vertical_ranges, dtype=np.float64)))


__all__ = [
    "A50BeamSample",
    "A50SensorConfig",
    "A50SensorModel",
    "A50SensorSample",
    "beam_directions_frd",
]
