"""Deterministic underwater RGB camera post-processing model.

The model is deliberately independent from MuJoCo and ROS.  It transforms an
ideal ``rgb8`` renderer output at the sensor boundary, which keeps the scene
renderer and the public ROS image contract unchanged.  The bundled parameters
are an engineering prior, not a calibration of either physical IMX219 camera.
"""

from __future__ import annotations

import ast
import json
import math
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np


DEFAULT_CAMERA_PROFILE_PATH = (
    Path(__file__).resolve().parents[1]
    / "config"
    / "sensor_models"
    / "imx219_underwater_uncalibrated_prior.json"
)


def _finite(value: Any, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite, got {value!r}")
    return result


def _float_tuple(
    values: Sequence[Any],
    name: str,
    *,
    length: int | None = None,
    minimum_length: int | None = None,
) -> tuple[float, ...]:
    if isinstance(values, (str, bytes)) or not isinstance(values, Sequence):
        raise TypeError(f"{name} must be a numeric sequence")
    result = tuple(_finite(value, f"{name}[{index}]") for index, value in enumerate(values))
    if length is not None and len(result) != length:
        raise ValueError(f"{name} must contain {length} values, got {len(result)}")
    if minimum_length is not None and len(result) < minimum_length:
        raise ValueError(
            f"{name} must contain at least {minimum_length} values, got {len(result)}"
        )
    return result


@dataclass(frozen=True, slots=True)
class CameraCalibration:
    """Pinhole intrinsics and ROS CameraInfo-compatible distortion data."""

    width: int
    height: int
    distortion_model: str
    d: tuple[float, ...]
    k: tuple[float, ...]
    r: tuple[float, ...]
    p: tuple[float, ...]
    calibration_status: str = "unvalidated_prior"
    source: str = ""

    def __post_init__(self) -> None:
        if isinstance(self.width, bool) or int(self.width) < 1:
            raise ValueError("camera calibration width must be a positive integer")
        if isinstance(self.height, bool) or int(self.height) < 1:
            raise ValueError("camera calibration height must be a positive integer")
        model = str(self.distortion_model).strip()
        if model not in {"plumb_bob", "rational_polynomial"}:
            raise ValueError(
                "distortion_model must be 'plumb_bob' or 'rational_polynomial'"
            )
        _float_tuple(self.d, "d", minimum_length=5)
        _float_tuple(self.k, "k", length=9)
        _float_tuple(self.r, "r", length=9)
        _float_tuple(self.p, "p", length=12)
        if float(self.k[0]) <= 0.0 or float(self.k[4]) <= 0.0:
            raise ValueError("camera focal lengths must be positive")
        if not str(self.calibration_status).strip():
            raise ValueError("calibration_status must be explicit")

    @property
    def has_distortion(self) -> bool:
        """Whether any supported distortion coefficient is non-zero."""

        return any(abs(value) > 1.0e-15 for value in self.d[:8])

    def scaled_to(self, width: int, height: int) -> CameraCalibration:
        """Scale intrinsics and projection data to a different image size."""

        width = int(width)
        height = int(height)
        if width < 1 or height < 1:
            raise ValueError("scaled camera dimensions must be positive")
        if width == self.width and height == self.height:
            return self
        sx = width / float(self.width)
        sy = height / float(self.height)
        k = list(self.k)
        for index in (0, 1, 2):
            k[index] *= sx
        for index in (3, 4, 5):
            k[index] *= sy
        p = list(self.p)
        for index in (0, 1, 2, 3):
            p[index] *= sx
        for index in (4, 5, 6, 7):
            p[index] *= sy
        return CameraCalibration(
            width=width,
            height=height,
            distortion_model=self.distortion_model,
            d=self.d,
            k=tuple(k),
            r=self.r,
            p=tuple(p),
            calibration_status=self.calibration_status,
            source=self.source,
        )


@dataclass(frozen=True, slots=True)
class UnderwaterOpticsConfig:
    """Homogeneous-water image formation and low-cost lens effects."""

    optical_path_length_m: float = 0.0
    attenuation_coefficients_rgb_per_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    backscatter_coefficients_rgb_per_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    veiling_light_rgb: tuple[float, float, float] = (0.0, 0.0, 0.0)
    backscatter_strength: float = 0.0
    blur_radius_px: int = 0
    vignetting_strength: float = 0.0

    def __post_init__(self) -> None:
        if _finite(self.optical_path_length_m, "optical_path_length_m") < 0.0:
            raise ValueError("optical_path_length_m must be non-negative")
        for name, values in (
            ("attenuation_coefficients_rgb_per_m", self.attenuation_coefficients_rgb_per_m),
            ("backscatter_coefficients_rgb_per_m", self.backscatter_coefficients_rgb_per_m),
        ):
            parsed = _float_tuple(values, name, length=3)
            if any(value < 0.0 for value in parsed):
                raise ValueError(f"{name} values must be non-negative")
        veiling = _float_tuple(self.veiling_light_rgb, "veiling_light_rgb", length=3)
        if any(value < 0.0 or value > 1.0 for value in veiling):
            raise ValueError("veiling_light_rgb values must be in [0, 1]")
        strength = _finite(self.backscatter_strength, "backscatter_strength")
        if strength < 0.0 or strength > 1.0:
            raise ValueError("backscatter_strength must be in [0, 1]")
        if isinstance(self.blur_radius_px, bool) or int(self.blur_radius_px) < 0:
            raise ValueError("blur_radius_px must be a non-negative integer")
        if int(self.blur_radius_px) > 8:
            raise ValueError("blur_radius_px must be no greater than 8")
        vignette = _finite(self.vignetting_strength, "vignetting_strength")
        if vignette < 0.0 or vignette > 1.0:
            raise ValueError("vignetting_strength must be in [0, 1]")


@dataclass(frozen=True, slots=True)
class CameraElectronicsConfig:
    """Exposure, gain, sensor noise, and ADC quantization parameters."""

    exposure_scale: float = 1.0
    analog_gain: float = 1.0
    black_level: float = 0.0
    shot_noise_electrons_per_unit: float = 0.0
    read_noise_electrons_rms: float = 0.0
    quantization_bits: int = 8

    def __post_init__(self) -> None:
        if _finite(self.exposure_scale, "exposure_scale") <= 0.0:
            raise ValueError("exposure_scale must be positive")
        if _finite(self.analog_gain, "analog_gain") <= 0.0:
            raise ValueError("analog_gain must be positive")
        black_level = _finite(self.black_level, "black_level")
        if black_level < 0.0 or black_level >= 1.0:
            raise ValueError("black_level must be in [0, 1)")
        shot = _finite(
            self.shot_noise_electrons_per_unit,
            "shot_noise_electrons_per_unit",
        )
        if shot < 0.0:
            raise ValueError("shot_noise_electrons_per_unit must be non-negative")
        if _finite(self.read_noise_electrons_rms, "read_noise_electrons_rms") < 0.0:
            raise ValueError("read_noise_electrons_rms must be non-negative")
        if isinstance(self.quantization_bits, bool) or not 2 <= int(self.quantization_bits) <= 16:
            raise ValueError("quantization_bits must be in [2, 16]")


@dataclass(frozen=True, slots=True)
class CameraTimingConfig:
    """Frame transport prior used by the ROS-boundary camera runtime."""

    device_clock_offset_s: float = 0.0
    device_clock_drift_ppm: float = 0.0
    processing_latency_mean_s: float = 0.0
    processing_latency_jitter_s: float = 0.0
    processing_latency_max_s: float | None = None
    transport_latency_mean_s: float = 0.0
    transport_latency_jitter_s: float = 0.0
    transport_latency_max_s: float | None = None
    dropout_probability: float = 0.0
    queue_capacity: int = 2
    overflow_policy: str = "drop_oldest"

    def __post_init__(self) -> None:
        _finite(self.device_clock_offset_s, "device_clock_offset_s")
        if _finite(self.device_clock_drift_ppm, "device_clock_drift_ppm") <= -1_000_000.0:
            raise ValueError("device_clock_drift_ppm must keep the clock increasing")
        for name, mean, jitter, maximum in (
            (
                "processing_latency",
                self.processing_latency_mean_s,
                self.processing_latency_jitter_s,
                self.processing_latency_max_s,
            ),
            (
                "transport_latency",
                self.transport_latency_mean_s,
                self.transport_latency_jitter_s,
                self.transport_latency_max_s,
            ),
        ):
            mean = _finite(mean, f"{name}_mean_s")
            jitter = _finite(jitter, f"{name}_jitter_s")
            if mean < 0.0 or jitter < 0.0:
                raise ValueError(f"{name} values must be non-negative")
            if maximum is not None and _finite(maximum, f"{name}_max_s") < mean:
                raise ValueError(f"{name}_max_s must be no smaller than its mean")
        dropout = _finite(self.dropout_probability, "dropout_probability")
        if dropout < 0.0 or dropout > 1.0:
            raise ValueError("dropout_probability must be in [0, 1]")
        if isinstance(self.queue_capacity, bool) or int(self.queue_capacity) < 1:
            raise ValueError("queue_capacity must be a positive integer")
        if str(self.overflow_policy) not in {"drop_oldest", "drop_newest"}:
            raise ValueError("overflow_policy must be drop_oldest or drop_newest")


@dataclass(frozen=True, slots=True)
class UnderwaterCameraProfile:
    """Complete optical, electronic, timing, and calibration prior."""

    schema: str
    profile: str
    calibration_status: str
    seed: int
    calibration: CameraCalibration
    optics: UnderwaterOpticsConfig = field(default_factory=UnderwaterOpticsConfig)
    electronics: CameraElectronicsConfig = field(default_factory=CameraElectronicsConfig)
    timing: CameraTimingConfig = field(default_factory=CameraTimingConfig)

    def __post_init__(self) -> None:
        if self.schema != "uuv_mujoco.sensor_model.underwater_camera.v1":
            raise ValueError(f"unsupported underwater camera schema: {self.schema!r}")
        if not str(self.profile).strip():
            raise ValueError("camera profile name must not be empty")
        if not str(self.calibration_status).strip():
            raise ValueError("camera calibration status must be explicit")
        if isinstance(self.seed, bool) or not isinstance(self.seed, int):
            raise TypeError("camera seed must be an integer")


@dataclass(frozen=True, slots=True)
class CameraModelDiagnostics:
    """Immutable summary of the transform applied to one frame."""

    sequence: int
    calibration_status: str
    optical_path_length_m: float
    distortion_applied: bool
    noise_applied: bool


class UnderwaterCameraSensorModel:
    """Apply a seeded underwater/lens/electronics model to ideal RGB images."""

    _CAMERA_SEED_SALTS = {
        "stereo_left": 0x49A10E21,
        "stereo_right": 0x49A10E22,
    }

    def __init__(
        self,
        profile: UnderwaterCameraProfile,
        *,
        calibration: CameraCalibration | None = None,
        camera_name: str = "stereo_left",
        enabled: bool = True,
        seed: int | None = None,
    ) -> None:
        if not isinstance(profile, UnderwaterCameraProfile):
            raise TypeError("profile must be an UnderwaterCameraProfile")
        self.profile = profile
        self.calibration = calibration or profile.calibration
        self.camera_name = str(camera_name)
        self.enabled = bool(enabled)
        selected_seed = profile.seed if seed is None else seed
        if isinstance(selected_seed, bool) or not isinstance(selected_seed, int):
            raise TypeError("camera model seed must be an integer")
        self.seed = int(selected_seed)
        self._seed_salt = self._CAMERA_SEED_SALTS.get(self.camera_name, 0x49A10EFF)
        self._distortion_map_cache: dict[tuple[int, int], tuple[np.ndarray, np.ndarray, np.ndarray]] = {}
        self._vignette_cache: dict[tuple[int, int], np.ndarray] = {}

    def process(
        self,
        rgb: np.ndarray,
        *,
        sequence: int,
    ) -> tuple[np.ndarray, CameraModelDiagnostics]:
        """Return one modeled ``rgb8`` frame and its deterministic diagnostics."""

        image = _validate_rgb8(rgb)
        if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
            raise ValueError("sequence must be a non-negative integer")
        if not self.enabled:
            return image.copy(), CameraModelDiagnostics(
                sequence=sequence,
                calibration_status=self.calibration.calibration_status,
                optical_path_length_m=0.0,
                distortion_applied=False,
                noise_applied=False,
            )

        working = image.astype(np.float32) * (1.0 / 255.0)
        calibration = self.calibration.scaled_to(image.shape[1], image.shape[0])
        if calibration.has_distortion:
            working = self._apply_distortion(working, calibration)

        optics = self.profile.optics
        path_m = float(optics.optical_path_length_m)
        attenuation = np.asarray(optics.attenuation_coefficients_rgb_per_m, dtype=np.float32)
        transmission = np.exp(-attenuation * path_m).astype(np.float32)
        working *= transmission.reshape(1, 1, 3)

        backscatter_coeff = np.asarray(
            optics.backscatter_coefficients_rgb_per_m,
            dtype=np.float32,
        )
        backscatter = 1.0 - np.exp(-backscatter_coeff * path_m).astype(np.float32)
        veiling = np.asarray(optics.veiling_light_rgb, dtype=np.float32)
        working += (
            float(optics.backscatter_strength)
            * backscatter.reshape(1, 1, 3)
            * veiling.reshape(1, 1, 3)
        )

        if optics.blur_radius_px > 0:
            working = _box_blur(working, int(optics.blur_radius_px))
        if optics.vignetting_strength > 0.0:
            working *= self._vignette(image.shape[0], image.shape[1])[:, :, None]

        electronics = self.profile.electronics
        photo_signal = np.maximum(working * float(electronics.exposure_scale), 0.0)
        rng = np.random.default_rng(
            np.random.SeedSequence(
                [
                    int(self.seed) & 0xFFFFFFFF,
                    int(self._seed_salt) & 0xFFFFFFFF,
                    int(sequence) & 0xFFFFFFFF,
                    (int(sequence) >> 32) & 0xFFFFFFFF,
                ]
            )
        )
        electrons_per_unit = float(electronics.shot_noise_electrons_per_unit)
        noise_applied = electrons_per_unit > 0.0 or electronics.read_noise_electrons_rms > 0.0
        signal = photo_signal * float(electronics.analog_gain)
        if noise_applied:
            noise_variance = np.zeros_like(photo_signal, dtype=np.float32)
            gain = float(electronics.analog_gain)
            if electrons_per_unit > 0.0:
                noise_variance += photo_signal / electrons_per_unit * (gain * gain)
            denominator = electrons_per_unit if electrons_per_unit > 0.0 else 65535.0
            read_std_output = (
                float(electronics.read_noise_electrons_rms)
                / denominator
                * gain
            )
            noise_variance += read_std_output * read_std_output
            signal += rng.standard_normal(signal.shape, dtype=np.float32) * np.sqrt(
                noise_variance,
                dtype=np.float32,
            )
        signal += float(electronics.black_level)
        signal = np.clip(signal, 0.0, 1.0)

        quantization_levels = float((1 << int(electronics.quantization_bits)) - 1)
        signal = np.rint(signal * quantization_levels) / quantization_levels
        output = np.ascontiguousarray(np.rint(signal * 255.0).astype(np.uint8))
        return output, CameraModelDiagnostics(
            sequence=sequence,
            calibration_status=self.calibration.calibration_status,
            optical_path_length_m=path_m,
            distortion_applied=calibration.has_distortion,
            noise_applied=noise_applied,
        )

    def close(self) -> None:
        """Release cached full-frame distortion and vignetting arrays."""

        self._distortion_map_cache.clear()
        self._vignette_cache.clear()

    def _vignette(self, height: int, width: int) -> np.ndarray:
        key = (int(height), int(width))
        cached = self._vignette_cache.get(key)
        if cached is not None:
            return cached
        y, x = np.mgrid[0:height, 0:width].astype(np.float32)
        cx = 0.5 * max(width - 1, 1)
        cy = 0.5 * max(height - 1, 1)
        radius_sq = ((x - cx) / max(cx, 1.0)) ** 2 + ((y - cy) / max(cy, 1.0)) ** 2
        mask = np.clip(
            1.0 - float(self.profile.optics.vignetting_strength) * radius_sq,
            0.0,
            1.0,
        ).astype(np.float32)
        self._vignette_cache[key] = mask
        return mask

    def _apply_distortion(
        self,
        image: np.ndarray,
        calibration: CameraCalibration,
    ) -> np.ndarray:
        height, width = image.shape[:2]
        key = (height, width)
        maps = self._distortion_map_cache.get(key)
        if maps is None:
            maps = _build_ideal_source_map(calibration)
            self._distortion_map_cache[key] = maps
        map_x, map_y, valid = maps
        return _bilinear_sample(image, map_x, map_y, valid)


def load_underwater_camera_profile(
    path: str | Path = DEFAULT_CAMERA_PROFILE_PATH,
) -> UnderwaterCameraProfile:
    """Load and validate one JSON underwater-camera sensor profile."""

    selected = Path(path).expanduser().resolve()
    with selected.open("r", encoding="utf-8") as stream:
        payload = json.load(stream)
    if not isinstance(payload, Mapping):
        raise TypeError("camera profile root must be a JSON object")

    calibration_status = str(payload.get("calibration_status", "")).strip()
    calibration = camera_calibration_from_mapping(
        _mapping(payload, "intrinsics"),
        calibration_status=calibration_status,
        source=str(selected),
    )
    optics_raw = _mapping(payload, "underwater_optics")
    electronics_raw = _mapping(payload, "electronics")
    timing_raw = _mapping(payload, "timing")
    processing_raw = _mapping(timing_raw, "processing_latency")
    transport_raw = _mapping(timing_raw, "transport_latency")
    queue_raw = _mapping(timing_raw, "queue")

    return UnderwaterCameraProfile(
        schema=str(payload.get("schema", "")),
        profile=str(payload.get("profile", "")),
        calibration_status=calibration_status,
        seed=int(payload.get("seed", 0)),
        calibration=calibration,
        optics=UnderwaterOpticsConfig(
            optical_path_length_m=float(optics_raw.get("optical_path_length_m", 0.0)),
            attenuation_coefficients_rgb_per_m=tuple(
                optics_raw.get("attenuation_coefficients_rgb_per_m", (0.0, 0.0, 0.0))
            ),
            backscatter_coefficients_rgb_per_m=tuple(
                optics_raw.get("backscatter_coefficients_rgb_per_m", (0.0, 0.0, 0.0))
            ),
            veiling_light_rgb=tuple(optics_raw.get("veiling_light_rgb", (0.0, 0.0, 0.0))),
            backscatter_strength=float(optics_raw.get("backscatter_strength", 0.0)),
            blur_radius_px=int(optics_raw.get("blur_radius_px", 0)),
            vignetting_strength=float(optics_raw.get("vignetting_strength", 0.0)),
        ),
        electronics=CameraElectronicsConfig(
            exposure_scale=float(electronics_raw.get("exposure_scale", 1.0)),
            analog_gain=float(electronics_raw.get("analog_gain", 1.0)),
            black_level=float(electronics_raw.get("black_level", 0.0)),
            shot_noise_electrons_per_unit=float(
                electronics_raw.get("shot_noise_electrons_per_unit", 0.0)
            ),
            read_noise_electrons_rms=float(
                electronics_raw.get("read_noise_electrons_rms", 0.0)
            ),
            quantization_bits=int(electronics_raw.get("quantization_bits", 8)),
        ),
        timing=CameraTimingConfig(
            device_clock_offset_s=float(timing_raw.get("device_clock_offset_s", 0.0)),
            device_clock_drift_ppm=float(timing_raw.get("device_clock_drift_ppm", 0.0)),
            processing_latency_mean_s=float(processing_raw.get("mean_s", 0.0)),
            processing_latency_jitter_s=float(processing_raw.get("jitter_std_s", 0.0)),
            processing_latency_max_s=_optional_float(processing_raw.get("max_s")),
            transport_latency_mean_s=float(transport_raw.get("mean_s", 0.0)),
            transport_latency_jitter_s=float(transport_raw.get("jitter_std_s", 0.0)),
            transport_latency_max_s=_optional_float(transport_raw.get("max_s")),
            dropout_probability=float(timing_raw.get("dropout_probability", 0.0)),
            queue_capacity=int(queue_raw.get("capacity", 2)),
            overflow_policy=str(queue_raw.get("overflow_policy", "drop_oldest")),
        ),
    )


def load_camera_calibration(
    path: str | Path,
    *,
    fallback_width: int | None = None,
    fallback_height: int | None = None,
) -> CameraCalibration:
    """Load a ROS CameraInfo YAML or equivalent JSON calibration file."""

    selected = Path(path).expanduser().resolve()
    text = selected.read_text(encoding="utf-8")
    if selected.suffix.lower() == ".json":
        payload = json.loads(text)
    else:
        payload = _load_yaml_or_minimal_camera_info(
            text,
            fallback_width=fallback_width,
            fallback_height=fallback_height,
        )
    if not isinstance(payload, Mapping):
        raise TypeError("camera calibration root must be an object")
    payload = _unwrap_ros_parameter_mapping(payload)
    return camera_calibration_from_mapping(
        payload,
        calibration_status="user_supplied_calibration",
        source=str(selected),
        fallback_width=fallback_width,
        fallback_height=fallback_height,
    )


def camera_calibration_from_mapping(
    payload: Mapping[str, Any],
    *,
    calibration_status: str,
    source: str = "",
    fallback_width: int | None = None,
    fallback_height: int | None = None,
) -> CameraCalibration:
    """Build a calibration from ROS CameraInfo or compact profile fields."""

    width = int(payload.get("image_width", payload.get("width", fallback_width or 0)))
    height = int(payload.get("image_height", payload.get("height", fallback_height or 0)))
    k = _matrix_data(payload, "camera_matrix", "k", expected=9)
    d = _matrix_data(payload, "distortion_coefficients", "d", minimum=5)
    r = _matrix_data(
        payload,
        "rectification_matrix",
        "r",
        expected=9,
        default=(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
    )
    p_default = (
        k[0],
        k[1],
        k[2],
        0.0,
        k[3],
        k[4],
        k[5],
        0.0,
        k[6],
        k[7],
        k[8],
        0.0,
    )
    p = _matrix_data(payload, "projection_matrix", "p", expected=12, default=p_default)
    return CameraCalibration(
        width=width,
        height=height,
        distortion_model=str(payload.get("distortion_model", "plumb_bob")),
        d=d,
        k=k,
        r=r,
        p=p,
        calibration_status=str(calibration_status),
        source=str(source),
    )


def _mapping(payload: Mapping[str, Any], key: str) -> Mapping[str, Any]:
    value = payload.get(key, {})
    if not isinstance(value, Mapping):
        raise TypeError(f"{key} must be an object")
    return value


def _matrix_data(
    payload: Mapping[str, Any],
    ros_key: str,
    compact_key: str,
    *,
    expected: int | None = None,
    minimum: int | None = None,
    default: Sequence[float] | None = None,
) -> tuple[float, ...]:
    value = payload.get(ros_key, payload.get(compact_key, default))
    if isinstance(value, Mapping):
        value = value.get("data", default)
    if value is None:
        raise ValueError(f"missing camera calibration field {ros_key}")
    return _float_tuple(value, ros_key, length=expected, minimum_length=minimum)


def _optional_float(value: Any) -> float | None:
    return None if value is None else float(value)


def _unwrap_ros_parameter_mapping(payload: Mapping[str, Any]) -> Mapping[str, Any]:
    direct = payload.get("ros__parameters")
    if isinstance(direct, Mapping):
        return direct
    for value in payload.values():
        if not isinstance(value, Mapping):
            continue
        parameters = value.get("ros__parameters")
        if isinstance(parameters, Mapping):
            return parameters
    return payload


def _load_yaml_or_minimal_camera_info(
    text: str,
    *,
    fallback_width: int | None = None,
    fallback_height: int | None = None,
) -> Mapping[str, Any]:
    try:
        import yaml  # type: ignore
    except Exception:
        return _parse_minimal_camera_info_yaml(
            text,
            fallback_width=fallback_width,
            fallback_height=fallback_height,
        )
    parsed = yaml.safe_load(text)
    if not isinstance(parsed, Mapping):
        raise TypeError("camera calibration YAML root must be an object")
    return parsed


def _parse_minimal_camera_info_yaml(
    text: str,
    *,
    fallback_width: int | None = None,
    fallback_height: int | None = None,
) -> Mapping[str, Any]:
    """Parse the standard camera_calibration YAML subset without PyYAML."""

    result: dict[str, Any] = {}
    fallback_dimensions = {
        "image_width": fallback_width,
        "image_height": fallback_height,
    }
    for key in ("image_width", "image_height"):
        match = re.search(rf"(?m)^\s*{re.escape(key)}\s*:\s*([^#\n]+)", text)
        if match is None:
            fallback = fallback_dimensions[key]
            if fallback is None or int(fallback) <= 0:
                raise ValueError(f"camera calibration YAML is missing {key}")
            result[key] = int(fallback)
        else:
            result[key] = int(match.group(1).strip())
    model_match = re.search(r"(?m)^\s*distortion_model\s*:\s*([^#\n]+)", text)
    result["distortion_model"] = (
        model_match.group(1).strip().strip("'\"") if model_match else "plumb_bob"
    )
    for key in (
        "camera_matrix",
        "distortion_coefficients",
        "rectification_matrix",
        "projection_matrix",
    ):
        inline = re.search(
            rf"(?m)^\s*{re.escape(key)}\s*:\s*(\[[^\]]*\])",
            text,
        )
        if inline is not None:
            result[key] = ast.literal_eval(inline.group(1))
            continue
        block = re.search(
            rf"(?ms)^\s*{re.escape(key)}\s*:\s*\n(?P<body>(?:^[ \t]+.*(?:\n|$))*)",
            text,
        )
        if block is None:
            if key in {"rectification_matrix", "projection_matrix"}:
                continue
            raise ValueError(f"camera calibration YAML is missing {key}")
        data_match = re.search(r"(?ms)^\s*data\s*:\s*(\[[^\]]*\])", block.group("body"))
        if data_match is None:
            raise ValueError(f"camera calibration YAML {key} is missing data")
        values = ast.literal_eval(data_match.group(1))
        result[key] = {"data": values}
    return result


def _validate_rgb8(rgb: np.ndarray) -> np.ndarray:
    if not isinstance(rgb, np.ndarray):
        raise TypeError("rgb must be a numpy array")
    if rgb.dtype != np.uint8:
        raise TypeError(f"rgb must have dtype uint8, got {rgb.dtype}")
    if rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError(f"rgb must have shape [height, width, 3], got {rgb.shape}")
    if rgb.shape[0] < 1 or rgb.shape[1] < 1:
        raise ValueError("rgb dimensions must be positive")
    return np.ascontiguousarray(rgb)


def _box_blur(image: np.ndarray, radius: int) -> np.ndarray:
    output = image
    for axis in (0, 1):
        pad_width = [(0, 0)] * output.ndim
        pad_width[axis] = (radius, radius)
        padded = np.pad(output, pad_width, mode="edge")
        cumulative = np.cumsum(padded, axis=axis, dtype=np.float32)
        zero_shape = list(cumulative.shape)
        zero_shape[axis] = 1
        cumulative = np.concatenate(
            [np.zeros(zero_shape, dtype=np.float32), cumulative],
            axis=axis,
        )
        window = 2 * radius + 1
        high = [slice(None)] * cumulative.ndim
        low = [slice(None)] * cumulative.ndim
        high[axis] = slice(window, None)
        low[axis] = slice(None, -window)
        output = (cumulative[tuple(high)] - cumulative[tuple(low)]) / float(window)
    return np.ascontiguousarray(output, dtype=np.float32)


def _build_ideal_source_map(
    calibration: CameraCalibration,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    height = calibration.height
    width = calibration.width
    fx, fy = float(calibration.k[0]), float(calibration.k[4])
    cx, cy = float(calibration.k[2]), float(calibration.k[5])
    y_pixels, x_pixels = np.mgrid[0:height, 0:width].astype(np.float32)
    x_distorted = (x_pixels - cx) / fx
    y_distorted = (y_pixels - cy) / fy
    x_ideal = x_distorted.copy()
    y_ideal = y_distorted.copy()

    coeff = list(calibration.d[:8]) + [0.0] * max(0, 8 - len(calibration.d))
    k1, k2, p1, p2, k3, k4, k5, k6 = coeff[:8]
    for _ in range(6):
        radius_sq = x_ideal * x_ideal + y_ideal * y_ideal
        radius_fourth = radius_sq * radius_sq
        radius_sixth = radius_fourth * radius_sq
        numerator = 1.0 + k1 * radius_sq + k2 * radius_fourth + k3 * radius_sixth
        denominator = 1.0 + k4 * radius_sq + k5 * radius_fourth + k6 * radius_sixth
        radial = numerator / np.maximum(denominator, 1.0e-8)
        x_estimate = (
            x_ideal * radial
            + 2.0 * p1 * x_ideal * y_ideal
            + p2 * (radius_sq + 2.0 * x_ideal * x_ideal)
        )
        y_estimate = (
            y_ideal * radial
            + p1 * (radius_sq + 2.0 * y_ideal * y_ideal)
            + 2.0 * p2 * x_ideal * y_ideal
        )
        x_ideal += x_distorted - x_estimate
        y_ideal += y_distorted - y_estimate

    map_x = (fx * x_ideal + cx).astype(np.float32)
    map_y = (fy * y_ideal + cy).astype(np.float32)
    valid = (
        (map_x >= 0.0)
        & (map_x <= width - 1.0)
        & (map_y >= 0.0)
        & (map_y <= height - 1.0)
    )
    return map_x, map_y, valid


def _bilinear_sample(
    image: np.ndarray,
    map_x: np.ndarray,
    map_y: np.ndarray,
    valid: np.ndarray,
) -> np.ndarray:
    height, width = image.shape[:2]
    x0 = np.floor(map_x).astype(np.int32)
    y0 = np.floor(map_y).astype(np.int32)
    x0 = np.clip(x0, 0, width - 1)
    y0 = np.clip(y0, 0, height - 1)
    x1 = np.minimum(x0 + 1, width - 1)
    y1 = np.minimum(y0 + 1, height - 1)
    wx = (map_x - x0).astype(np.float32)[:, :, None]
    wy = (map_y - y0).astype(np.float32)[:, :, None]
    top = image[y0, x0] * (1.0 - wx) + image[y0, x1] * wx
    bottom = image[y1, x0] * (1.0 - wx) + image[y1, x1] * wx
    output = top * (1.0 - wy) + bottom * wy
    output[~valid] = 0.0
    return np.ascontiguousarray(output, dtype=np.float32)


__all__ = [
    "CameraCalibration",
    "CameraElectronicsConfig",
    "CameraModelDiagnostics",
    "CameraTimingConfig",
    "DEFAULT_CAMERA_PROFILE_PATH",
    "UnderwaterCameraProfile",
    "UnderwaterCameraSensorModel",
    "UnderwaterOpticsConfig",
    "camera_calibration_from_mapping",
    "load_camera_calibration",
    "load_underwater_camera_profile",
]
