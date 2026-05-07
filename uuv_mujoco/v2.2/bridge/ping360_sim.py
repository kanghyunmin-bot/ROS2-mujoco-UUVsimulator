"""Synthetic Blue Robotics Ping360 model for the MuJoCo runtime."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import mujoco
import numpy as np


PING360_GRADS_PER_REV = 400
PING360_DEG_PER_GRAD = 360.0 / PING360_GRADS_PER_REV
SAMPLE_PERIOD_TICK_S = 25.0e-9


@dataclass
class Ping360Config:
    enabled: bool = True
    site_name: str = "ping360_site"
    frame_id: str = "ping360_link"
    interface_mode: str = "ethernet"
    requested_range_m: float = 2.0
    speed_of_sound_mps: float = 1500.0
    gain_setting: int = 0
    transmit_frequency_khz: int = 750
    auto_transmit_duration: bool = True
    transmit_duration_us: int = 11
    start_angle_grad: int = 0
    stop_angle_grad: int = 399
    sector_size_deg: float = 360.0
    angle_offset_grad: int = 200
    num_steps: int = 1
    sector_bounce: bool = True
    horizontal_beamwidth_deg: float = 2.0
    vertical_beamwidth_deg: float = 25.0
    horizontal_ray_count: int = 3
    vertical_ray_count: int = 7
    noise_floor: float = 2.0
    speckle_std: float = 4.0
    absorption_db_per_m: float = 0.12
    range_power_loss: float = 1.35
    image_size_px: int = 640
    image_display_gain: float = 12.0
    publish_image: bool = True
    publish_scan: bool = True
    publish_status: bool = True

    # Firmware / physical limits, kept close to Ping-Viewer and Ping Protocol.
    min_range_m: float = 0.75
    max_range_m: float = 50.0
    min_number_of_samples: int = 200
    max_number_of_samples: int = 1200
    min_sample_period_ticks: int = 80
    max_sample_period_ticks: int = 40000
    min_transmit_duration_us: int = 5
    max_transmit_duration_us: int = 500
    min_transmit_frequency_khz: int = 500
    max_transmit_frequency_khz: int = 1000
    practical_min_transmit_frequency_khz: int = 650
    practical_max_transmit_frequency_khz: int = 850
    min_num_steps: int = 1
    max_num_steps: int = 10

    @classmethod
    def from_file(cls, path: str | Path | None, overrides: dict[str, Any] | None = None) -> "Ping360Config":
        data: dict[str, Any] = {}
        if path:
            cfg_path = Path(path).expanduser()
            if cfg_path.is_file():
                data = json.loads(cfg_path.read_text(encoding="utf-8"))
        if overrides:
            data.update({key: value for key, value in overrides.items() if value is not None})
        known = {field_name for field_name in cls.__dataclass_fields__}
        return cls(**{key: value for key, value in data.items() if key in known})


@dataclass
class Ping360EffectiveSettings:
    requested_range_m: float
    effective_range_m: float
    speed_of_sound_mps: float
    number_of_samples: int
    sample_period_ticks: int
    transmit_duration_us: int
    transmit_duration_max_us: int
    transmit_frequency_khz: int
    gain_setting: int
    num_steps: int
    angular_resolution_deg: float
    start_angle_grad: int
    stop_angle_grad: int
    sector_size_grad: int
    sector_size_deg: float
    profile_period_s: float
    scan_period_s: float
    interface_mode: str
    range_resolution_m: float
    blind_bins: int
    quality_flags: list[str] = field(default_factory=list)

    def as_dict(self) -> dict[str, Any]:
        return {
            "requested_range_m": self.requested_range_m,
            "effective_range_m": self.effective_range_m,
            "speed_of_sound_mps": self.speed_of_sound_mps,
            "number_of_samples": self.number_of_samples,
            "sample_period_ticks": self.sample_period_ticks,
            "sample_period_us": self.sample_period_ticks * 0.025,
            "transmit_duration_us": self.transmit_duration_us,
            "transmit_duration_max_us": self.transmit_duration_max_us,
            "transmit_frequency_khz": self.transmit_frequency_khz,
            "gain_setting": self.gain_setting,
            "num_steps": self.num_steps,
            "angular_resolution_deg": self.angular_resolution_deg,
            "start_angle_grad": self.start_angle_grad,
            "start_angle_deg": self.start_angle_grad * PING360_DEG_PER_GRAD,
            "stop_angle_grad": self.stop_angle_grad,
            "stop_angle_deg": self.stop_angle_grad * PING360_DEG_PER_GRAD,
            "sector_size_grad": self.sector_size_grad,
            "sector_size_deg": self.sector_size_deg,
            "profile_period_s": self.profile_period_s,
            "scan_period_s": self.scan_period_s,
            "interface_mode": self.interface_mode,
            "range_resolution_m": self.range_resolution_m,
            "blind_bins": self.blind_bins,
            "quality_flags": self.quality_flags,
        }


@dataclass
class Ping360Sample:
    sim_time_s: float
    angle_grad: int
    profile: np.ndarray
    image: np.ndarray
    ranges_m: np.ndarray
    intensities: np.ndarray
    settings: Ping360EffectiveSettings
    ping_number: int
    updated: bool

    def status_dict(self) -> dict[str, Any]:
        return {
            "sim_time_s": self.sim_time_s,
            "angle_grad": self.angle_grad,
            "angle_deg": self.angle_grad * PING360_DEG_PER_GRAD,
            "ping_number": self.ping_number,
            "settings": self.settings.as_dict(),
        }


class Ping360Simulator:
    """Generate Ping360-like profile scans from MuJoCo ray casts."""

    _INTERFACE_FULL_SCAN_S = {
        "usb": ((1.0, 4.17), (50.0, 33.0)),
        "ethernet": ((1.0, 3.42), (50.0, 33.0)),
        "rs485": ((1.0, 4.26), (50.0, 33.0)),
    }

    def __init__(self, model: mujoco.MjModel, config: Ping360Config) -> None:
        self.model = model
        self.config = config
        self.site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, config.site_name)
        self.base_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        self._geomgroup = np.ones(6, dtype=np.uint8)
        self._geomgroup[5] = 0
        self._rng = np.random.default_rng(360)
        self._latest: Ping360Sample | None = None
        self._next_profile_t = -1.0
        self._ping_number = 0
        self._full_angle_grad = 0
        self._sector_rel_grad = 0
        self._sector_direction = 1
        self._image = np.zeros((PING360_GRADS_PER_REV, 1), dtype=np.uint8)
        self._ranges = np.full(PING360_GRADS_PER_REV, np.inf, dtype=np.float32)
        self._intensities = np.zeros(PING360_GRADS_PER_REV, dtype=np.float32)
        self.settings = self._build_effective_settings()
        self._resize_buffers(self.settings.number_of_samples)

    @property
    def active(self) -> bool:
        return bool(self.config.enabled and self.site_id >= 0)

    def update_config(self, config: Ping360Config) -> None:
        self.config = config
        self.site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, config.site_name)
        self.settings = self._build_effective_settings()
        self._resize_buffers(self.settings.number_of_samples)

    def update(self, data: mujoco.MjData, sim_t: float) -> Ping360Sample | None:
        if not self.active:
            return None
        sim_t = float(sim_t)
        self.settings = self._build_effective_settings()
        self._resize_buffers(self.settings.number_of_samples)
        if self._next_profile_t < 0.0:
            self._next_profile_t = sim_t
        if self._latest is not None and sim_t + 1.0e-9 < self._next_profile_t:
            return Ping360Sample(
                sim_time_s=sim_t,
                angle_grad=self._latest.angle_grad,
                profile=self._latest.profile,
                image=self._latest.image,
                ranges_m=self._latest.ranges_m,
                intensities=self._latest.intensities,
                settings=self.settings,
                ping_number=self._latest.ping_number,
                updated=False,
            )

        angle_grad = self._current_angle_grad()
        profile, nearest_range, peak_intensity = self._scan_profile(data, angle_grad, self.settings)
        self._image[angle_grad, :] = profile
        self._ranges[angle_grad] = nearest_range if nearest_range is not None else np.inf
        self._intensities[angle_grad] = peak_intensity
        self._ping_number += 1
        self._advance_angle()
        self._next_profile_t = sim_t + self.settings.profile_period_s
        self._latest = Ping360Sample(
            sim_time_s=sim_t,
            angle_grad=angle_grad,
            profile=profile,
            image=self._image.copy(),
            ranges_m=self._ranges.copy(),
            intensities=self._intensities.copy(),
            settings=self.settings,
            ping_number=self._ping_number,
            updated=True,
        )
        return self._latest

    def _resize_buffers(self, number_of_samples: int) -> None:
        if self._image.shape[1] == number_of_samples:
            return
        self._image = np.zeros((PING360_GRADS_PER_REV, number_of_samples), dtype=np.uint8)
        self._ranges = np.full(PING360_GRADS_PER_REV, np.inf, dtype=np.float32)
        self._intensities = np.zeros(PING360_GRADS_PER_REV, dtype=np.float32)
        self._latest = None

    def _build_effective_settings(self) -> Ping360EffectiveSettings:
        cfg = self.config
        quality_flags: list[str] = []
        requested_range = float(cfg.requested_range_m)
        range_m = float(np.clip(requested_range, cfg.min_range_m, cfg.max_range_m))
        if abs(range_m - requested_range) > 1.0e-6:
            quality_flags.append("range_clamped_to_physical_limits")

        speed_of_sound = float(np.clip(cfg.speed_of_sound_mps, 1300.0, 1700.0))
        num_samples = int(cfg.max_number_of_samples)
        sample_period = self._calculate_sample_period(range_m, num_samples, speed_of_sound)
        while sample_period < cfg.min_sample_period_ticks and num_samples > cfg.min_number_of_samples:
            num_samples -= 1
            sample_period = self._calculate_sample_period(range_m, num_samples, speed_of_sound)
        sample_period = int(np.clip(sample_period, cfg.min_sample_period_ticks, cfg.max_sample_period_ticks))
        if num_samples < cfg.max_number_of_samples:
            quality_flags.append("short_range_reduced_sample_count")

        effective_range = sample_period * SAMPLE_PERIOD_TICK_S * num_samples * speed_of_sound / 2.0
        if effective_range + 1.0e-6 < cfg.min_range_m:
            quality_flags.append("effective_range_below_blind_zone")

        transmit_duration_max = min(
            int(cfg.max_transmit_duration_us),
            int(sample_period * SAMPLE_PERIOD_TICK_S * 64.0e6),
        )
        auto_duration = int(round(8000.0 * effective_range / speed_of_sound))
        sample_interval_us = sample_period * SAMPLE_PERIOD_TICK_S * 1.0e6
        auto_duration = max(int(round(2.5 * sample_interval_us)), auto_duration)
        requested_duration = auto_duration if cfg.auto_transmit_duration else int(cfg.transmit_duration_us)
        transmit_duration = int(
            np.clip(
                requested_duration,
                cfg.min_transmit_duration_us,
                max(cfg.min_transmit_duration_us, transmit_duration_max),
            )
        )
        if transmit_duration != requested_duration:
            quality_flags.append("transmit_duration_clamped")
        if transmit_duration >= transmit_duration_max:
            quality_flags.append("transmit_duration_at_firmware_limit")

        transmit_frequency = int(np.clip(cfg.transmit_frequency_khz, cfg.min_transmit_frequency_khz, cfg.max_transmit_frequency_khz))
        if not (cfg.practical_min_transmit_frequency_khz <= transmit_frequency <= cfg.practical_max_transmit_frequency_khz):
            quality_flags.append("transmit_frequency_outside_practical_band")

        gain = int(np.clip(cfg.gain_setting, 0, 2))
        if gain >= 2:
            quality_flags.append("high_gain_saturation_risk")

        num_steps = int(np.clip(cfg.num_steps, cfg.min_num_steps, cfg.max_num_steps))
        if num_steps > 1:
            quality_flags.append("angular_resolution_reduced_for_speed")

        start_angle = int(round(float(cfg.start_angle_grad))) % PING360_GRADS_PER_REV
        stop_angle = int(round(float(cfg.stop_angle_grad))) % PING360_GRADS_PER_REV
        sector_grad = ((stop_angle - start_angle) % PING360_GRADS_PER_REV) + 1
        profile_period = self._profile_period_s(effective_range, num_steps, cfg.interface_mode)
        scan_period = math.ceil(sector_grad / num_steps) * profile_period
        range_resolution = effective_range / max(num_samples, 1)
        if range_resolution > 0.025:
            quality_flags.append("range_resolution_coarser_than_2_5cm")

        blind_bins = int(math.ceil(cfg.min_range_m / max(range_resolution, 1.0e-9)))
        return Ping360EffectiveSettings(
            requested_range_m=requested_range,
            effective_range_m=effective_range,
            speed_of_sound_mps=speed_of_sound,
            number_of_samples=num_samples,
            sample_period_ticks=sample_period,
            transmit_duration_us=transmit_duration,
            transmit_duration_max_us=transmit_duration_max,
            transmit_frequency_khz=transmit_frequency,
            gain_setting=gain,
            num_steps=num_steps,
            angular_resolution_deg=num_steps * PING360_DEG_PER_GRAD,
            start_angle_grad=start_angle,
            stop_angle_grad=stop_angle,
            sector_size_grad=sector_grad,
            sector_size_deg=sector_grad * PING360_DEG_PER_GRAD,
            profile_period_s=profile_period,
            scan_period_s=scan_period,
            interface_mode=self._normalize_interface(cfg.interface_mode),
            range_resolution_m=range_resolution,
            blind_bins=blind_bins,
            quality_flags=quality_flags,
        )

    @staticmethod
    def _calculate_sample_period(range_m: float, num_samples: int, speed_of_sound: float) -> int:
        if range_m <= 0.0 or num_samples <= 0 or speed_of_sound <= 0.0:
            return 88
        return int(2.0 * range_m / (num_samples * speed_of_sound * SAMPLE_PERIOD_TICK_S))

    def _profile_period_s(self, effective_range_m: float, num_steps: int, interface_mode: str) -> float:
        one_step_full = self._full_scan_period_one_grad_s(effective_range_m, interface_mode)
        motor_full_s = 2.4
        listen_full_s = max(0.0, one_step_full - motor_full_s)
        motor_per_grad_s = motor_full_s / PING360_GRADS_PER_REV
        listen_per_ping_s = listen_full_s / PING360_GRADS_PER_REV
        return max(0.001, motor_per_grad_s * num_steps + listen_per_ping_s)

    def _full_scan_period_one_grad_s(self, effective_range_m: float, interface_mode: str) -> float:
        mode = self._normalize_interface(interface_mode)
        (r0, t0), (r1, t1) = self._INTERFACE_FULL_SCAN_S[mode]
        r = float(np.clip(effective_range_m, r0, r1))
        if r1 <= r0:
            return t1
        alpha = (r - r0) / (r1 - r0)
        return (1.0 - alpha) * t0 + alpha * t1

    @classmethod
    def _normalize_interface(cls, interface_mode: str) -> str:
        mode = str(interface_mode or "ethernet").strip().lower()
        return mode if mode in cls._INTERFACE_FULL_SCAN_S else "ethernet"

    def _current_angle_grad(self) -> int:
        if self.settings.sector_size_grad >= PING360_GRADS_PER_REV:
            return (self.settings.start_angle_grad + self._full_angle_grad) % PING360_GRADS_PER_REV
        rel = int(np.clip(self._sector_rel_grad, 0, max(0, self.settings.sector_size_grad - 1)))
        return (self.settings.start_angle_grad + rel) % PING360_GRADS_PER_REV

    def _advance_angle(self) -> None:
        steps = int(self.settings.num_steps)
        if self.settings.sector_size_grad >= PING360_GRADS_PER_REV:
            self._full_angle_grad = (self._full_angle_grad + steps) % PING360_GRADS_PER_REV
            return
        max_rel = max(0, self.settings.sector_size_grad - 1)
        if not self.config.sector_bounce:
            self._sector_rel_grad = (self._sector_rel_grad + steps) % max(1, self.settings.sector_size_grad)
            return
        next_rel = self._sector_rel_grad + self._sector_direction * steps
        if next_rel < 0 or next_rel > max_rel:
            self._sector_direction *= -1
            next_rel = self._sector_rel_grad + self._sector_direction * steps
        self._sector_rel_grad = int(np.clip(next_rel, 0, max_rel))

    def _scan_profile(
        self,
        data: mujoco.MjData,
        angle_grad: int,
        settings: Ping360EffectiveSettings,
    ) -> tuple[np.ndarray, float | None, float]:
        n = settings.number_of_samples
        profile = np.zeros(n, dtype=np.float64)
        if self.site_id < 0:
            return profile.astype(np.uint8), None, 0.0

        origin = np.asarray(data.site_xpos[self.site_id], dtype=np.float64).copy()
        site_rot = np.asarray(data.site_xmat[self.site_id], dtype=np.float64).reshape(3, 3).copy()
        angle_rad = 2.0 * math.pi * float(angle_grad) / PING360_GRADS_PER_REV
        dirs = self._beam_directions_local(angle_rad)
        nearest = math.inf
        peak = 0.0
        for local_dir, weight in dirs:
            world_dir = site_rot @ local_dir
            norm = float(np.linalg.norm(world_dir))
            if norm <= 1.0e-12:
                continue
            world_dir = world_dir / norm
            dist, geom_id = self._raycast(data, origin, world_dir, settings.effective_range_m)
            if dist is None:
                continue
            if dist < self.config.min_range_m or dist > settings.effective_range_m:
                continue
            nearest = min(nearest, dist)
            reflectivity = self._geom_reflectivity(geom_id)
            amp = self._return_strength(reflectivity, dist, settings) * weight
            peak = max(peak, amp)
            self._accumulate_return(profile, dist, amp, settings)

        blind_bins = min(settings.blind_bins, n)
        if blind_bins > 0:
            profile[:blind_bins] = 0.0
        if self.config.noise_floor > 0.0:
            profile[blind_bins:] += float(self.config.noise_floor)
        if self.config.speckle_std > 0.0:
            profile[blind_bins:] += self._rng.normal(0.0, float(self.config.speckle_std), max(0, n - blind_bins))
        profile = np.clip(profile, 0.0, 255.0)
        return profile.astype(np.uint8), (nearest if math.isfinite(nearest) else None), float(min(255.0, peak))

    def _beam_directions_local(self, angle_rad: float) -> list[tuple[np.ndarray, float]]:
        h_count = max(1, int(self.config.horizontal_ray_count))
        v_count = max(1, int(self.config.vertical_ray_count))
        h_span = math.radians(float(self.config.horizontal_beamwidth_deg))
        v_span = math.radians(float(self.config.vertical_beamwidth_deg))
        h_offsets = np.linspace(-0.5 * h_span, 0.5 * h_span, h_count)
        v_offsets = np.linspace(-0.5 * v_span, 0.5 * v_span, v_count)
        rays: list[tuple[np.ndarray, float]] = []
        for h in h_offsets:
            for v in v_offsets:
                yaw = angle_rad + float(h)
                cv = math.cos(float(v))
                local = np.array([math.cos(yaw) * cv, math.sin(yaw) * cv, math.sin(float(v))], dtype=np.float64)
                weight = math.cos(float(h)) ** 2 * math.cos(float(v)) ** 2
                rays.append((local / max(np.linalg.norm(local), 1.0e-12), float(max(weight, 0.05))))
        total = sum(weight for _, weight in rays)
        if total > 1.0e-12:
            rays = [(direction, weight / total) for direction, weight in rays]
        return rays

    def _raycast(
        self,
        data: mujoco.MjData,
        origin: np.ndarray,
        direction: np.ndarray,
        cutoff: float,
    ) -> tuple[float | None, int]:
        geom_id = np.array([-1], dtype=np.int32)
        try:
            distance = float(
                mujoco.mj_ray(
                    self.model,
                    data,
                    origin,
                    direction,
                    self._geomgroup,
                    1,
                    int(self.base_body_id),
                    geom_id,
                )
            )
        except TypeError:
            distance = float(mujoco.mj_ray(self.model, data, origin, direction, None, 1, int(self.base_body_id), geom_id))
        if distance < 0.0 or distance > cutoff or not math.isfinite(distance):
            return None, -1
        return distance, int(geom_id[0])

    def _geom_reflectivity(self, geom_id: int) -> float:
        if geom_id < 0:
            return 0.0
        name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, int(geom_id)) or ""
        if "wall" in name:
            return 230.0
        if "floor" in name:
            return 190.0
        if "pool" in name:
            return 180.0
        return 150.0

    def _return_strength(self, reflectivity: float, distance_m: float, settings: Ping360EffectiveSettings) -> float:
        gain = (0.9, 1.45, 2.2)[settings.gain_setting]
        frequency_penalty = 1.0 - min(abs(settings.transmit_frequency_khz - 750.0) / 500.0, 0.5)
        absorption = 10.0 ** (-(float(self.config.absorption_db_per_m) * distance_m) / 20.0)
        spreading = 1.0 / max(distance_m, 1.0) ** float(self.config.range_power_loss)
        pulse_gain = math.sqrt(max(settings.transmit_duration_us, 1.0) / 11.0)
        return reflectivity * gain * frequency_penalty * absorption * spreading * pulse_gain

    def _accumulate_return(
        self,
        profile: np.ndarray,
        distance_m: float,
        amplitude: float,
        settings: Ping360EffectiveSettings,
    ) -> None:
        bin_width = max(settings.range_resolution_m, 1.0e-9)
        center = int(round(distance_m / bin_width))
        if center < 0 or center >= profile.size:
            return
        pulse_length_m = settings.speed_of_sound_mps * settings.transmit_duration_us * 1.0e-6 / 2.0
        sigma_bins = max(0.75, pulse_length_m / bin_width)
        radius = int(max(1, min(12, math.ceil(3.0 * sigma_bins))))
        lo = max(0, center - radius)
        hi = min(profile.size, center + radius + 1)
        idx = np.arange(lo, hi, dtype=np.float64)
        weights = np.exp(-0.5 * ((idx - center) / sigma_bins) ** 2)
        if weights.size > 0 and float(weights.max()) > 0.0:
            weights /= float(weights.max())
        profile[lo:hi] += amplitude * weights
