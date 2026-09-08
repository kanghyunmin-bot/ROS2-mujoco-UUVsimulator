"""Synthetic pinger/hydrophone bridge for audio-common verification."""

from __future__ import annotations

from dataclasses import dataclass
import math
import os
import time
from typing import Any

import mujoco
import numpy as np

from .ros2_mujoco_model import site_id
from .ros2_publish_schedule_hydrophone import schedule_hydrophone_jobs
from .ros2_status_messages import build_json_string_msg
from .sitl_env import env_to_float, env_to_int


@dataclass(frozen=True)
class HydrophoneConfig:
    enabled: bool
    audio_enabled: bool
    pinger_site_name: str
    center_site_name: str
    left_site_name: str
    right_site_name: str
    frequency_hz: float
    sample_rate_hz: int
    channels: int
    publish_hz: float
    info_hz: float
    status_hz: float
    sound_speed_mps: float
    amplitude: float
    noise_amplitude: float
    snr_probe_noise_amplitude: float
    max_range_m: float
    interferers_enabled: bool
    interferer_count: int
    interferer_thruster_count: int
    interferer_frequency_span_hz: float
    interferer_amplitude: float
    interferer_seed: int
    pool_x_min_m: float
    pool_x_max_m: float
    pool_y_min_m: float
    pool_y_max_m: float
    pool_z_min_m: float
    pool_z_max_m: float


@dataclass(frozen=True)
class HydrophoneNoiseSource:
    name: str
    kind: str
    frequency_hz: float
    amplitude: float
    phase_rad: float
    site_name: str
    site_id: int
    actuator_name: str
    actuator_id: int
    position_m: tuple[float, float, float] | None


THRUSTER_NOISE_SITES: tuple[tuple[str, str], ...] = (
    ("thr_ver_lf", "ver_lf"),
    ("thr_ver_lr", "ver_lr"),
    ("thr_ver_rf", "ver_rf"),
    ("thr_ver_rr", "ver_rr"),
    ("thr_yaw_lf", "yaw_lf"),
    ("thr_yaw_lr", "yaw_lr"),
    ("thr_yaw_rf", "yaw_rf"),
    ("thr_yaw_rr", "yaw_rr"),
)


# Match the viewer's direction staleness window.  The controller-owned
# /pinger_homing/direction_body topic is the canonical red-arrow input; legacy
# body topics and the raw SNR /homing/direction topic are fallbacks only.  A
# fallback must not win a callback race while a canonical sample is still
# drawable.
CANONICAL_HOMING_DIRECTION_FRESHNESS_S = 1.5


def _env_str(name: str, default: str) -> str:
    value = os.getenv(name)
    return default if value is None or value == "" else value


def _site_world_pos(data, sid: int) -> np.ndarray | None:
    if sid < 0:
        return None
    try:
        pos = np.asarray(data.site_xpos[sid], dtype=np.float64).copy()
    except Exception:
        return None
    if not np.all(np.isfinite(pos)):
        return None
    return pos


def _lookup_site(model, name: str) -> int:
    try:
        return int(site_id(model, name))
    except Exception:
        return -1


def _lookup_actuator(model, name: str) -> int:
    try:
        return int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name))
    except Exception:
        return -1


def _resolved_sample_rate_hz(frequency_hz: float) -> int:
    requested = int(env_to_int("ROS2_UUV_HYDROPHONE_SAMPLE_RATE_HZ", 0))
    auto_rate = int(math.ceil(max(96000.0, 2.5 * float(frequency_hz))))
    sample_rate = requested if requested > 0 else auto_rate
    if sample_rate < 2.05 * frequency_hz:
        sample_rate = int(math.ceil(2.5 * frequency_hz))
    return int(np.clip(sample_rate, 8000, 1000000))


def _env_range(name_min: str, name_max: str, default_min: float, default_max: float) -> tuple[float, float]:
    lo = float(env_to_float(name_min, default_min))
    hi = float(env_to_float(name_max, default_max))
    if not (math.isfinite(lo) and math.isfinite(hi)) or hi <= lo:
        return float(default_min), float(default_max)
    return lo, hi


def configure_hydrophone_runtime(bridge: Any) -> None:
    frequency_hz = float(np.clip(env_to_float("ROS2_UUV_HYDROPHONE_FREQ_HZ", 21164.0), 100.0, 48000.0))
    pool_x = _env_range("ROS2_UUV_HYDROPHONE_NOISE_POOL_X_MIN_M", "ROS2_UUV_HYDROPHONE_NOISE_POOL_X_MAX_M", -14.0, 14.0)
    pool_y = _env_range("ROS2_UUV_HYDROPHONE_NOISE_POOL_Y_MIN_M", "ROS2_UUV_HYDROPHONE_NOISE_POOL_Y_MAX_M", -9.0, 9.0)
    pool_z = _env_range("ROS2_UUV_HYDROPHONE_NOISE_POOL_Z_MIN_M", "ROS2_UUV_HYDROPHONE_NOISE_POOL_Z_MAX_M", -8.5, -0.4)
    cfg = HydrophoneConfig(
        enabled=bool(env_to_int("ROS2_UUV_HYDROPHONE_ENABLE", 1)),
        audio_enabled=bool(env_to_int("ROS2_UUV_HYDROPHONE_AUDIO_ENABLE", 1)),
        pinger_site_name=_env_str(
            "ROS2_UUV_HYDROPHONE_PINGER_SITE",
            "course_buoy_pinger_white_1_acoustic_site",
        ),
        center_site_name=_env_str("ROS2_UUV_HYDROPHONE_CENTER_SITE", "hydrophone_center_site"),
        left_site_name=_env_str("ROS2_UUV_HYDROPHONE_LEFT_SITE", "hydrophone_left_site"),
        right_site_name=_env_str("ROS2_UUV_HYDROPHONE_RIGHT_SITE", "hydrophone_right_site"),
        frequency_hz=frequency_hz,
        sample_rate_hz=_resolved_sample_rate_hz(frequency_hz),
        channels=2,
        publish_hz=float(np.clip(env_to_float("ROS2_UUV_HYDROPHONE_AUDIO_HZ", 20.0), 1.0, 100.0)),
        info_hz=float(np.clip(env_to_float("ROS2_UUV_HYDROPHONE_INFO_HZ", 1.0), 0.1, 10.0)),
        status_hz=float(np.clip(env_to_float("ROS2_UUV_HYDROPHONE_STATUS_HZ", 10.0), 0.1, 100.0)),
        sound_speed_mps=float(np.clip(env_to_float("ROS2_UUV_HYDROPHONE_SOUND_SPEED_MPS", 1500.0), 1000.0, 1700.0)),
        amplitude=float(np.clip(env_to_float("ROS2_UUV_HYDROPHONE_AMPLITUDE", 0.65), 0.0, 0.95)),
        noise_amplitude=float(np.clip(env_to_float("ROS2_UUV_HYDROPHONE_NOISE_AMPLITUDE", 0.0015), 0.0, 0.10)),
        snr_probe_noise_amplitude=float(
            np.clip(
                env_to_float("ROS2_UUV_HYDROPHONE_SNR_PROBE_NOISE_AMPLITUDE", 0.006),
                0.0,
                0.05,
            )
        ),
        max_range_m=float(np.clip(env_to_float("ROS2_UUV_HYDROPHONE_MAX_RANGE_M", 80.0), 0.1, 500.0)),
        interferers_enabled=bool(env_to_int("ROS2_UUV_HYDROPHONE_INTERFERERS_ENABLE", 1)),
        interferer_count=int(np.clip(env_to_int("ROS2_UUV_HYDROPHONE_INTERFERER_COUNT", 10), 0, 64)),
        interferer_thruster_count=int(
            np.clip(env_to_int("ROS2_UUV_HYDROPHONE_INTERFERER_THRUSTER_COUNT", 8), 0, len(THRUSTER_NOISE_SITES))
        ),
        interferer_frequency_span_hz=float(
            np.clip(env_to_float("ROS2_UUV_HYDROPHONE_INTERFERER_FREQ_SPAN_HZ", 470.0), 0.0, 100000.0)
        ),
        interferer_amplitude=float(
            np.clip(env_to_float("ROS2_UUV_HYDROPHONE_INTERFERER_AMPLITUDE", 0.035), 0.0, 0.50)
        ),
        interferer_seed=int(env_to_int("ROS2_UUV_HYDROPHONE_INTERFERER_SEED", 2608)),
        pool_x_min_m=pool_x[0],
        pool_x_max_m=pool_x[1],
        pool_y_min_m=pool_y[0],
        pool_y_max_m=pool_y[1],
        pool_z_min_m=pool_z[0],
        pool_z_max_m=pool_z[1],
    )
    bridge._hydrophone_config = cfg
    bridge._hydrophone_pinger_site_id = _lookup_site(bridge.model, cfg.pinger_site_name)
    bridge._hydrophone_center_site_id = _lookup_site(bridge.model, cfg.center_site_name)
    bridge._hydrophone_left_site_id = _lookup_site(bridge.model, cfg.left_site_name)
    bridge._hydrophone_right_site_id = _lookup_site(bridge.model, cfg.right_site_name)
    bridge._hydrophone_audio_sample_index = 0
    bridge._hydrophone_audio_last_wall = None
    bridge._hydrophone_audio_last_sim_time = None
    bridge._hydrophone_audio_last_target_ranges = None
    bridge._hydrophone_audio_last_source_ranges = {}
    bridge._hydrophone_rng = np.random.default_rng(2607)
    bridge._hydrophone_noise_sources = _build_noise_sources(bridge, cfg)
    bridge._hydrophone_last_estimated_direction_body = np.zeros(3, dtype=np.float64)
    bridge._hydrophone_last_estimated_direction_active = False
    bridge._hydrophone_last_estimated_direction_wall = float("-inf")
    bridge._hydrophone_last_estimated_direction_source = "none"
    bridge._hydrophone_last_canonical_direction_wall = float("-inf")
    if cfg.enabled:
        print(
            "[hydrophone] "
            f"pinger={cfg.pinger_site_name}({bridge._hydrophone_pinger_site_id}) "
            f"array={cfg.center_site_name}/{cfg.left_site_name}/{cfg.right_site_name} "
            f"freq={cfg.frequency_hz:.1f}Hz sample_rate={cfg.sample_rate_hz}Hz "
            f"audio={cfg.audio_enabled} interferers={len(bridge._hydrophone_noise_sources)}",
            flush=True,
        )


def _build_noise_sources(bridge: Any, cfg: HydrophoneConfig) -> list[HydrophoneNoiseSource]:
    if not cfg.interferers_enabled or cfg.interferer_count <= 0 or cfg.interferer_amplitude <= 0.0:
        return []
    rng = np.random.default_rng(cfg.interferer_seed)
    sources: list[HydrophoneNoiseSource] = []
    thruster_count = min(cfg.interferer_thruster_count, cfg.interferer_count, len(THRUSTER_NOISE_SITES))
    for index, (site_name, actuator_name) in enumerate(THRUSTER_NOISE_SITES[:thruster_count]):
        sources.append(
            HydrophoneNoiseSource(
                name=f"thruster_noise_{site_name}",
                kind="thruster",
                frequency_hz=_near_frequency(rng, cfg),
                amplitude=float(cfg.interferer_amplitude * rng.uniform(0.65, 1.25)),
                phase_rad=float(rng.uniform(0.0, 2.0 * math.pi)),
                site_name=site_name,
                site_id=_lookup_site(bridge.model, site_name),
                actuator_name=actuator_name,
                actuator_id=_lookup_actuator(bridge.model, actuator_name),
                position_m=None,
            )
        )
    for index in range(max(0, cfg.interferer_count - thruster_count)):
        position = (
            float(rng.uniform(cfg.pool_x_min_m, cfg.pool_x_max_m)),
            float(rng.uniform(cfg.pool_y_min_m, cfg.pool_y_max_m)),
            float(rng.uniform(cfg.pool_z_min_m, cfg.pool_z_max_m)),
        )
        sources.append(
            HydrophoneNoiseSource(
                name=f"pool_noise_{index + 1}",
                kind="pool",
                frequency_hz=_near_frequency(rng, cfg),
                amplitude=float(cfg.interferer_amplitude * rng.uniform(0.25, 0.90)),
                phase_rad=float(rng.uniform(0.0, 2.0 * math.pi)),
                site_name="",
                site_id=-1,
                actuator_name="",
                actuator_id=-1,
                position_m=position,
            )
        )
    return sources


def _near_frequency(rng: np.random.Generator, cfg: HydrophoneConfig) -> float:
    offset = float(rng.uniform(-cfg.interferer_frequency_span_hz, cfg.interferer_frequency_span_hz))
    # The unmodified receiver demodulates 4096 samples at 96 kHz. Interferers
    # inside roughly two Hann main lobes are indistinguishable from the pinger
    # and corrupt phase rather than exercising adjacent-channel rejection.
    guard_hz = float(
        np.clip(
            env_to_float("ROS2_UUV_HYDROPHONE_INTERFERER_GUARD_HZ", 120.0),
            0.0,
            cfg.interferer_frequency_span_hz,
        )
    )
    if guard_hz > 0.0 and abs(offset) < guard_hz:
        sign = -1.0 if offset < 0.0 else 1.0
        remaining = max(cfg.interferer_frequency_span_hz - guard_hz, 0.0)
        offset = sign * (guard_hz + remaining * abs(offset) / max(guard_hz, 1.0e-9))
    nyquist_guard = 0.48 * float(cfg.sample_rate_hz)
    return float(np.clip(cfg.frequency_hz + offset, 100.0, max(100.0, nyquist_guard)))


def create_hydrophone_publishers(bridge: Any, *, q10) -> None:
    node = bridge.node
    cfg = bridge._hydrophone_config
    bridge.pub_hydrophone_status = node.create_publisher(
        bridge.String,
        "/mujoco/hydrophone/status",
        q10,
    )
    bridge.pub_hydrophone_direction = node.create_publisher(
        bridge.Vector3Stamped,
        "/mujoco/hydrophone/direction",
        q10,
    )
    # /homing/direction is owned by the unmodified audio phase estimator. The
    # simulator exposes exact direction only on the explicit MuJoCo diagnostic
    # topic so mission control cannot bypass the acoustic signal path.
    bridge.pub_homing_direction = None
    bridge.pub_hydrophone_audio = (
        node.create_publisher(bridge.AudioData, "/audio", q10)
        if cfg.enabled and cfg.audio_enabled and bridge.AudioData is not None
        else None
    )
    bridge.pub_hydrophone_audio_info = (
        node.create_publisher(bridge.AudioInfo, "/audio_info", q10)
        if cfg.enabled and cfg.audio_enabled and bridge.AudioInfo is not None
        else None
    )


def create_hydrophone_subscriptions(bridge: Any, *, q10) -> None:
    # Dedicated real/sim C++ pinger package default. This is already a
    # normalized base_link vector and is the canonical red viewer arrow input.
    bridge.sub_pinger_homing_direction_body = bridge.node.create_subscription(
        bridge.Vector3Stamped,
        "/pinger_homing/direction_body",
        lambda msg: _on_homing_direction_estimated_body(
            bridge, msg, require_body_frame=True, source="canonical"
        ),
        q10,
    )
    bridge.sub_homing_direction_estimated_body = bridge.node.create_subscription(
        bridge.Vector3Stamped,
        "/mission/hydrophone/direction_body",
        lambda msg: _on_homing_direction_estimated_body(
            bridge, msg, source="mission_fallback"
        ),
        q10,
    )
    bridge.sub_legacy_homing_direction_estimated_body = bridge.node.create_subscription(
        bridge.Vector3Stamped,
        "/homing/direction_estimated_body",
        lambda msg: _on_homing_direction_estimated_body(
            bridge, msg, source="legacy_fallback"
        ),
        q10,
    )
    bridge.sub_snr_homing_direction_body = bridge.node.create_subscription(
        bridge.Vector3Stamped,
        "/homing/direction",
        lambda msg: _on_homing_direction_estimated_body(
            bridge, msg, require_body_frame=True, source="raw_snr_fallback"
        ),
        q10,
    )


def _canonical_homing_direction_is_fresh(bridge: Any, now_wall: float) -> bool:
    canonical_wall = float(
        getattr(bridge, "_hydrophone_last_canonical_direction_wall", float("-inf"))
    )
    age_s = now_wall - canonical_wall
    return (
        math.isfinite(age_s)
        and 0.0 <= age_s <= CANONICAL_HOMING_DIRECTION_FRESHNESS_S
    )


def _on_homing_direction_estimated_body(
    bridge: Any,
    msg: Any,
    *,
    require_body_frame: bool = False,
    source: str = "fallback",
) -> None:
    now_wall = time.monotonic()
    if source != "canonical" and _canonical_homing_direction_is_fresh(
        bridge, now_wall
    ):
        return
    if require_body_frame and str(msg.header.frame_id).strip().lower() not in {
        "base_link",
        "body",
    }:
        # The phase estimator publishes its direction in the odometry/world
        # frame. Only consume the shared topic here when the selected SNR mode
        # explicitly publishes a body-frame direction.
        return
    direction = np.array(
        [msg.vector.x, msg.vector.y, msg.vector.z],
        dtype=np.float64,
    )
    norm = float(np.linalg.norm(direction))
    if direction.shape != (3,) or not np.all(np.isfinite(direction)) or norm <= 1.0e-9:
        bridge._hydrophone_last_estimated_direction_active = False
        bridge._hydrophone_last_estimated_direction_source = source
        return
    bridge._hydrophone_last_estimated_direction_body = direction / norm
    bridge._hydrophone_last_estimated_direction_active = True
    bridge._hydrophone_last_estimated_direction_wall = now_wall
    bridge._hydrophone_last_estimated_direction_source = source
    if source == "canonical":
        bridge._hydrophone_last_canonical_direction_wall = now_wall


def _hydrophone_measurement(bridge: Any, data, state) -> dict[str, Any]:
    cfg = bridge._hydrophone_config
    pinger_pos = _site_world_pos(data, bridge._hydrophone_pinger_site_id)
    center_pos = _site_world_pos(data, bridge._hydrophone_center_site_id)
    left_pos = _site_world_pos(data, bridge._hydrophone_left_site_id)
    right_pos = _site_world_pos(data, bridge._hydrophone_right_site_id)
    if left_pos is None:
        left_pos = center_pos
    if right_pos is None:
        right_pos = center_pos
    valid = pinger_pos is not None and center_pos is not None
    if not valid:
        return {
            "valid": False,
            "active": False,
            "reason": "missing_site",
            "range_m": None,
            "bearing_rad": None,
            "elevation_rad": None,
            "direction_body": [0.0, 0.0, 0.0],
            "ranges_m": [None, None],
        }

    vector_world = pinger_pos - center_pos
    range_m = float(np.linalg.norm(vector_world))
    if range_m > 1.0e-9:
        direction_world = vector_world / range_m
    else:
        direction_world = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    body_vec = np.asarray(state.rot_world_body, dtype=np.float64).T @ vector_world
    body_range = float(np.linalg.norm(body_vec))
    body_dir = (body_vec / body_range) if body_range > 1.0e-9 else np.array([1.0, 0.0, 0.0], dtype=np.float64)
    bearing_rad = float(math.atan2(body_vec[1], body_vec[0]))
    elevation_rad = float(math.atan2(body_vec[2], max(math.hypot(body_vec[0], body_vec[1]), 1.0e-9)))
    left_range_m = float(np.linalg.norm(pinger_pos - left_pos)) if left_pos is not None else range_m
    right_range_m = float(np.linalg.norm(pinger_pos - right_pos)) if right_pos is not None else range_m
    return {
        "valid": True,
        "active": bool(range_m <= cfg.max_range_m),
        "reason": "ok",
        "range_m": range_m,
        "bearing_rad": bearing_rad,
        "elevation_rad": elevation_rad,
        "direction_body": [float(body_dir[0]), float(body_dir[1]), float(body_dir[2])],
        "direction_world": [float(direction_world[0]), float(direction_world[1]), float(direction_world[2])],
        "ranges_m": [left_range_m, right_range_m],
    }


def _noise_source_world_pos(bridge: Any, data, source: HydrophoneNoiseSource) -> np.ndarray | None:
    if source.site_id >= 0:
        return _site_world_pos(data, source.site_id)
    if source.position_m is None:
        return None
    pos = np.asarray(source.position_m, dtype=np.float64)
    return pos if np.all(np.isfinite(pos)) else None


def _source_ranges_to_hydrophones(
    source_pos: np.ndarray,
    center_pos: np.ndarray | None,
    left_pos: np.ndarray | None,
    right_pos: np.ndarray | None,
) -> tuple[float, list[float]]:
    if center_pos is None:
        center_range = 1.0
    else:
        center_range = float(np.linalg.norm(source_pos - center_pos))
    if left_pos is None:
        left_pos = center_pos
    if right_pos is None:
        right_pos = center_pos
    left_range = float(np.linalg.norm(source_pos - left_pos)) if left_pos is not None else center_range
    right_range = float(np.linalg.norm(source_pos - right_pos)) if right_pos is not None else center_range
    return center_range, [left_range, right_range]


def _source_activity_scale(bridge: Any, data, source: HydrophoneNoiseSource) -> float:
    if source.kind != "thruster" or source.actuator_id < 0:
        return 1.0
    try:
        ctrl = float(data.ctrl[source.actuator_id])
        limits = np.asarray(bridge.model.actuator_ctrlrange[source.actuator_id], dtype=np.float64)
        denom = float(max(abs(limits[0]), abs(limits[1]), 1.0))
    except Exception:
        return 0.45
    activity = float(np.clip(abs(ctrl) / denom, 0.0, 1.0))
    return 0.35 + 0.65 * activity


def _noise_status(bridge: Any, data) -> list[dict[str, Any]]:
    center_pos = _site_world_pos(data, bridge._hydrophone_center_site_id)
    sources = getattr(bridge, "_hydrophone_noise_sources", [])
    out: list[dict[str, Any]] = []
    for source in sources:
        pos = _noise_source_world_pos(bridge, data, source)
        if pos is None:
            continue
        range_m = float(np.linalg.norm(pos - center_pos)) if center_pos is not None else None
        out.append(
            {
                "name": source.name,
                "kind": source.kind,
                "frequency_hz": source.frequency_hz,
                "amplitude": source.amplitude,
                "site": source.site_name or None,
                "actuator": source.actuator_name or None,
                "range_m": range_m,
                "xyz": [float(pos[0]), float(pos[1]), float(pos[2])],
            }
        )
    return out


def build_hydrophone_status_msg(bridge, data, _stamp, state):
    cfg = bridge._hydrophone_config
    measurement = _hydrophone_measurement(bridge, data, state)
    interferers = _noise_status(bridge, data)
    payload = {
        "enabled": cfg.enabled,
        "audio_enabled": bool(bridge.pub_hydrophone_audio is not None),
        "frequency_hz": cfg.frequency_hz,
        "sample_rate_hz": cfg.sample_rate_hz,
        "publish_hz": cfg.publish_hz,
        "snr_probe_noise_amplitude": cfg.snr_probe_noise_amplitude,
        "pinger_site": cfg.pinger_site_name,
        "center_site": cfg.center_site_name,
        "interferers_enabled": cfg.interferers_enabled,
        "interferer_count": len(interferers),
        "interferers": interferers,
        **measurement,
    }
    return build_json_string_msg(bridge.String, payload)


def build_hydrophone_direction_msg(bridge, data, stamp, state):
    measurement = _hydrophone_measurement(bridge, data, state)
    msg = bridge.Vector3Stamped()
    msg.header.stamp = stamp
    msg.header.frame_id = "dvl"
    direction = measurement["direction_body"]
    msg.vector.x = float(direction[0])
    msg.vector.y = float(direction[1])
    msg.vector.z = float(direction[2])
    return msg


def build_hydrophone_audio_info_msg(bridge, _data, _stamp, _state):
    cfg = bridge._hydrophone_config
    msg = bridge.AudioInfo()
    msg.channels = int(cfg.channels)
    msg.sample_rate = int(cfg.sample_rate_hz)
    msg.sample_format = "S32LE"
    msg.bitrate = int(cfg.sample_rate_hz * cfg.channels * 32)
    msg.coding_format = "wave"
    return msg


def build_hydrophone_audio_msg(bridge, data, _stamp, state):
    cfg = bridge._hydrophone_config
    measurement = _hydrophone_measurement(bridge, data, state)
    frames = _next_audio_frame_count(bridge, cfg, sim_time_s=float(data.time))
    sample_index = int(bridge._hydrophone_audio_sample_index)
    # Keep the large absolute sample index out of float32.  A double-precision
    # phase offset is applied in _add_tone, while the bulk trigonometry stays
    # float32 so 96 kHz PCM generation cannot stall the physics loop.
    samples = np.arange(frames, dtype=np.float32)
    signal = np.zeros((frames, cfg.channels), dtype=np.float32)
    if measurement["active"]:
        left_range_m, right_range_m = measurement["ranges_m"]
        ranges = [float(left_range_m), float(right_range_m)]
        previous_ranges = getattr(bridge, "_hydrophone_audio_last_target_ranges", None)
        if previous_ranges is None or len(previous_ranges) != len(ranges):
            previous_ranges = ranges
        range_tracks = [
            np.linspace(float(start), float(end), frames, endpoint=True, dtype=np.float32)
            for start, end in zip(previous_ranges, ranges)
        ]
        attenuation = cfg.amplitude / math.sqrt(max(float(measurement["range_m"]), 1.0))
        amplitude = float(np.clip(attenuation, 0.02, cfg.amplitude))
        _add_tone(
            signal,
            samples,
            frequency_hz=cfg.frequency_hz,
            sample_rate_hz=cfg.sample_rate_hz,
            sound_speed_mps=cfg.sound_speed_mps,
            amplitude=amplitude,
            phase_rad=0.0,
            ranges_m=range_tracks,
            sample_offset=sample_index,
        )
        bridge._hydrophone_audio_last_target_ranges = ranges
    else:
        bridge._hydrophone_audio_last_target_ranges = None
    _add_interferer_audio(
        bridge, data, signal, samples, sample_offset=sample_index
    )
    _add_snr_probe_noise(
        signal,
        samples,
        cfg=cfg,
        sample_offset=sample_index,
    )
    if cfg.noise_amplitude > 0.0:
        signal += bridge._hydrophone_rng.normal(
            0.0, cfg.noise_amplitude, size=signal.shape
        ).astype(np.float32)
    pcm = (np.clip(signal, -0.98, 0.98) * 2147483647.0).astype("<i4", copy=False)
    msg = bridge.AudioData()
    msg.data = pcm.tobytes(order="C")
    bridge._hydrophone_audio_sample_index += frames
    return msg


def _add_snr_probe_noise(
    signal: np.ndarray,
    samples: np.ndarray,
    *,
    cfg: HydrophoneConfig,
    sample_offset: int = 0,
) -> None:
    """Provide a stable side-band floor for the upstream SNR estimator.

    The estimator measures noise at six fixed offsets around the pinger. Pure
    frame-wise white noise gives each short IQ window a Rayleigh-distributed
    denominator, which can dominate the small range-related amplitude change.
    These low-amplitude stationary components model a repeatable broadband
    receiver floor while preserving all target amplitude and phase physics.
    """

    amplitude = float(cfg.snr_probe_noise_amplitude)
    if amplitude <= 0.0:
        return
    for index, offset_hz in enumerate((-700.0, -450.0, -250.0, 250.0, 450.0, 700.0)):
        frequency_hz = float(cfg.frequency_hz + offset_hz)
        if frequency_hz <= 1.0 or frequency_hz >= 0.49 * float(cfg.sample_rate_hz):
            continue
        _add_tone(
            signal,
            samples,
            frequency_hz=frequency_hz,
            sample_rate_hz=cfg.sample_rate_hz,
            sound_speed_mps=cfg.sound_speed_mps,
            amplitude=amplitude,
            phase_rad=0.71 * float(index + 1),
            ranges_m=[0.0] * signal.shape[1],
            sample_offset=sample_offset,
        )


def _next_audio_frame_count(
    bridge: Any,
    cfg: HydrophoneConfig,
    *,
    sim_time_s: float | None = None,
) -> int:
    """Keep PCM duration on the same clock used by the phase estimator."""

    nominal_s = 1.0 / max(float(cfg.publish_hz), 1.0e-6)
    if sim_time_s is not None and math.isfinite(float(sim_time_s)):
        now_sim = float(sim_time_s)
        previous_sim = getattr(bridge, "_hydrophone_audio_last_sim_time", None)
        elapsed_s = nominal_s if previous_sim is None else now_sim - float(previous_sim)
        bridge._hydrophone_audio_last_sim_time = now_sim
        if elapsed_s <= 0.0:
            elapsed_s = nominal_s
    else:
        # Retain a wall-time fallback for isolated tests and non-MuJoCo users.
        now_wall = time.monotonic()
        previous_wall = getattr(bridge, "_hydrophone_audio_last_wall", None)
        elapsed_s = nominal_s if previous_wall is None else now_wall - float(previous_wall)
        bridge._hydrophone_audio_last_wall = now_wall
    # Audio publication is subscriber-driven.  During the ten-second FFT
    # selection the phase estimator is intentionally absent, so the builder
    # may not run at all.  A physical volatile DDS stream does not replay that
    # missing interval when a new receiver connects; its first packet is one
    # ordinary capture buffer.  Generating a 0.5 s catch-up packet here made
    # the external O(N*FFT) phase estimator several seconds late and paired
    # old delta-range with current odometry.  Preserve short scheduler jitter,
    # but treat a gap longer than 2.5 nominal periods as a fresh receiver.
    if elapsed_s > 2.5 * nominal_s:
        elapsed_s = nominal_s
    elapsed_s = float(
        np.clip(
            elapsed_s,
            1.0 / float(cfg.sample_rate_hz),
            max(2.5 * nominal_s, 1.0 / float(cfg.sample_rate_hz)),
        )
    )
    return max(1, int(round(float(cfg.sample_rate_hz) * elapsed_s)))


def _add_interferer_audio(
    bridge: Any,
    data,
    signal: np.ndarray,
    samples: np.ndarray,
    *,
    sample_offset: int = 0,
) -> None:
    sources = getattr(bridge, "_hydrophone_noise_sources", [])
    if not sources:
        return
    cfg = bridge._hydrophone_config
    center_pos = _site_world_pos(data, bridge._hydrophone_center_site_id)
    left_pos = _site_world_pos(data, bridge._hydrophone_left_site_id)
    right_pos = _site_world_pos(data, bridge._hydrophone_right_site_id)
    previous_by_name = getattr(bridge, "_hydrophone_audio_last_source_ranges", {})
    current_by_name: dict[str, list[float]] = {}
    for source in sources:
        pos = _noise_source_world_pos(bridge, data, source)
        if pos is None:
            continue
        center_range, ranges = _source_ranges_to_hydrophones(pos, center_pos, left_pos, right_pos)
        current_by_name[source.name] = ranges
        previous_ranges = previous_by_name.get(source.name, ranges)
        range_tracks = [
            np.linspace(float(start), float(end), signal.shape[0], endpoint=True, dtype=np.float32)
            for start, end in zip(previous_ranges, ranges)
        ]
        activity = _source_activity_scale(bridge, data, source)
        amplitude = float(np.clip(source.amplitude * activity / math.sqrt(max(center_range, 1.0)), 0.0, 0.95))
        if amplitude <= 0.0:
            continue
        _add_tone(
            signal,
            samples,
            frequency_hz=source.frequency_hz,
            sample_rate_hz=cfg.sample_rate_hz,
            sound_speed_mps=cfg.sound_speed_mps,
            amplitude=amplitude,
            phase_rad=source.phase_rad,
            ranges_m=range_tracks,
            sample_offset=sample_offset,
        )
    bridge._hydrophone_audio_last_source_ranges = current_by_name


def _add_tone(
    signal: np.ndarray,
    samples: np.ndarray,
    *,
    frequency_hz: float,
    sample_rate_hz: int,
    sound_speed_mps: float,
    amplitude: float,
    phase_rad: float,
    ranges_m: list[float | np.ndarray],
    sample_offset: int = 0,
) -> None:
    omega = 2.0 * math.pi * float(frequency_hz) / float(sample_rate_hz)
    phase_offset = math.remainder(
        omega * int(sample_offset) + float(phase_rad), 2.0 * math.pi
    )
    phase = (
        np.float32(omega) * np.asarray(samples, dtype=np.float32)
        + np.float32(phase_offset)
    )
    channels = min(signal.shape[1], len(ranges_m))
    for channel in range(channels):
        channel_range = np.asarray(ranges_m[channel], dtype=np.float32)
        distance_scale = np.float32(
            2.0 * math.pi * float(frequency_hz) / float(sound_speed_mps)
        )
        signal[:, channel] += np.float32(amplitude) * np.sin(
            phase - distance_scale * channel_range
        )


def build_hydrophone_publish_builders(bridge, data, stamp, state) -> dict[str, object]:
    return {
        "hydrophone_status": lambda: build_hydrophone_status_msg(bridge, data, stamp, state),
        "hydrophone_direction": lambda: build_hydrophone_direction_msg(bridge, data, stamp, state),
        "hydrophone_audio_info": lambda: build_hydrophone_audio_info_msg(bridge, data, stamp, state),
        "hydrophone_audio": lambda: build_hydrophone_audio_msg(bridge, data, stamp, state),
    }


__all__ = [
    "HydrophoneConfig",
    "HydrophoneNoiseSource",
    "build_hydrophone_publish_builders",
    "configure_hydrophone_runtime",
    "create_hydrophone_publishers",
    "create_hydrophone_subscriptions",
    "schedule_hydrophone_jobs",
]
