#!/usr/bin/env python3
"""Pure relative-pose math for the filtered-odometry oracle check.

The functions in this module never compare absolute positions.  Each phase is
rebased independently with ``T_phase_start^-1 * T_sample`` before the filtered
estimate is compared with the MuJoCo oracle.
"""

from __future__ import annotations

import bisect
import math
from dataclasses import asdict, dataclass
from typing import Iterable, Sequence


Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]


@dataclass(frozen=True)
class PoseSample:
    """One stamped pose; absolute coordinates must not be serialized in reports."""

    stamp_s: float
    elapsed_s: float
    position: Vector3
    orientation_xyzw: Quaternion
    frame_id: str
    child_frame_id: str


@dataclass(frozen=True)
class MatchedPair:
    oracle: PoseSample
    estimate: PoseSample
    stamp_delta_s: float
    elapsed_s: float


@dataclass(frozen=True)
class RelativeMotion:
    translation: Vector3
    orientation_xyzw: Quaternion


@dataclass(frozen=True)
class ErrorThresholds:
    """Optional upper limits.  ``None`` records a metric without gating it."""

    xy_rms_m: float | None = None
    xy_max_m: float | None = None
    z_rms_m: float | None = None
    z_max_m: float | None = None
    yaw_rms_deg: float | None = None
    yaw_max_deg: float | None = None


@dataclass(frozen=True)
class PhaseMotionThresholds:
    """Oracle-only checks that prove a phase was actually stationary or moving."""

    stationary_translation_max_m: float = 0.10
    stationary_yaw_max_deg: float = 5.0
    moving_translation_min_m: float = 0.20
    moving_yaw_min_deg: float = 5.0


def _finite(values: Iterable[float]) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def normalize_quaternion(quaternion: Quaternion) -> Quaternion:
    if not _finite(quaternion):
        raise ValueError("quaternion contains a non-finite component")
    norm = math.sqrt(sum(float(value) * float(value) for value in quaternion))
    if norm <= 1.0e-12:
        raise ValueError("quaternion norm is zero")
    return tuple(float(value) / norm for value in quaternion)  # type: ignore[return-value]


def quaternion_conjugate(quaternion: Quaternion) -> Quaternion:
    x, y, z, w = normalize_quaternion(quaternion)
    return (-x, -y, -z, w)


def quaternion_multiply(left: Quaternion, right: Quaternion) -> Quaternion:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return normalize_quaternion(
        (
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        )
    )


def rotate_vector(quaternion: Quaternion, vector: Vector3) -> Vector3:
    """Rotate a vector by an xyzw quaternion without constructing a matrix."""

    qx, qy, qz, qw = normalize_quaternion(quaternion)
    vx, vy, vz = vector
    # v' = v + 2*w*(q_xyz x v) + 2*(q_xyz x (q_xyz x v)).
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


def yaw_radians(quaternion: Quaternion) -> float:
    x, y, z, w = normalize_quaternion(quaternion)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_radians(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def relative_motion(reference: PoseSample, sample: PoseSample) -> RelativeMotion:
    """Return ``T_reference^-1 * T_sample`` in the phase-start body frame."""

    if not _finite(reference.position + sample.position):
        raise ValueError("pose position contains a non-finite component")
    reference_inverse = quaternion_conjugate(reference.orientation_xyzw)
    delta_world = tuple(
        float(sample.position[index]) - float(reference.position[index])
        for index in range(3)
    )
    translation = rotate_vector(reference_inverse, delta_world)  # type: ignore[arg-type]
    orientation = quaternion_multiply(reference_inverse, sample.orientation_xyzw)
    return RelativeMotion(translation=translation, orientation_xyzw=orientation)


def match_samples_by_stamp(
    oracle_samples: Sequence[PoseSample],
    estimate_samples: Sequence[PoseSample],
    max_stamp_delta_s: float,
) -> list[MatchedPair]:
    """Greedily make unique nearest-stamp pairs in chronological order."""

    if max_stamp_delta_s < 0.0:
        raise ValueError("max_stamp_delta_s must be non-negative")
    oracle = sorted(oracle_samples, key=lambda sample: sample.stamp_s)
    estimate = sorted(estimate_samples, key=lambda sample: sample.stamp_s)
    estimate_stamps = [sample.stamp_s for sample in estimate]
    pairs: list[MatchedPair] = []
    next_estimate_index = 0
    for oracle_sample in oracle:
        insert = bisect.bisect_left(
            estimate_stamps, oracle_sample.stamp_s, lo=next_estimate_index
        )
        candidate_indices = {
            index
            for index in (insert - 1, insert)
            if next_estimate_index <= index < len(estimate)
        }
        if not candidate_indices:
            continue
        best_index = min(
            candidate_indices,
            key=lambda index: abs(estimate[index].stamp_s - oracle_sample.stamp_s),
        )
        estimate_sample = estimate[best_index]
        delta_s = estimate_sample.stamp_s - oracle_sample.stamp_s
        if abs(delta_s) > max_stamp_delta_s:
            continue
        pairs.append(
            MatchedPair(
                oracle=oracle_sample,
                estimate=estimate_sample,
                stamp_delta_s=delta_s,
                elapsed_s=max(oracle_sample.elapsed_s, estimate_sample.elapsed_s),
            )
        )
        next_estimate_index = best_index + 1
    return pairs


def select_elapsed_window(
    pairs: Sequence[MatchedPair], start_s: float, end_s: float
) -> list[MatchedPair]:
    if end_s <= start_s:
        raise ValueError("phase end must be after phase start")
    return [pair for pair in pairs if start_s <= pair.elapsed_s < end_s]


def _error_metrics(values: Sequence[float]) -> dict[str, float | None]:
    if not values:
        return {"rms": None, "max_abs": None, "mean_abs": None, "final_abs": None}
    absolute = [abs(float(value)) for value in values]
    return {
        "rms": math.sqrt(sum(float(value) ** 2 for value in values) / len(values)),
        "max_abs": max(absolute),
        "mean_abs": sum(absolute) / len(absolute),
        "final_abs": absolute[-1],
    }


def _upper_limit_check(actual: float | None, limit: float | None) -> dict[str, object]:
    enabled = limit is not None
    ok = actual is not None and (not enabled or actual <= float(limit))
    return {"actual": actual, "limit": limit, "enabled": enabled, "ok": ok}


def _motion_check(
    phase_kind: str,
    max_translation_m: float,
    max_yaw_deg: float,
    thresholds: PhaseMotionThresholds,
) -> dict[str, object]:
    if phase_kind == "stationary":
        translation_ok = max_translation_m <= thresholds.stationary_translation_max_m
        yaw_ok = max_yaw_deg <= thresholds.stationary_yaw_max_deg
        return {
            "kind": phase_kind,
            "translation": {
                "actual_max_m": max_translation_m,
                "limit_max_m": thresholds.stationary_translation_max_m,
                "ok": translation_ok,
            },
            "yaw": {
                "actual_max_deg": max_yaw_deg,
                "limit_max_deg": thresholds.stationary_yaw_max_deg,
                "ok": yaw_ok,
            },
            "rule": "translation AND yaw must remain below their limits",
            "ok": translation_ok and yaw_ok,
        }
    if phase_kind != "moving":
        raise ValueError(f"unknown phase_kind: {phase_kind}")
    translation_ok = max_translation_m >= thresholds.moving_translation_min_m
    yaw_ok = max_yaw_deg >= thresholds.moving_yaw_min_deg
    return {
        "kind": phase_kind,
        "translation": {
            "actual_max_m": max_translation_m,
            "limit_min_m": thresholds.moving_translation_min_m,
            "ok": translation_ok,
        },
        "yaw": {
            "actual_max_deg": max_yaw_deg,
            "limit_min_deg": thresholds.moving_yaw_min_deg,
            "ok": yaw_ok,
        },
        "rule": "translation OR yaw must exceed its minimum",
        "ok": translation_ok or yaw_ok,
    }


def analyze_phase(
    name: str,
    phase_kind: str,
    pairs: Sequence[MatchedPair],
    *,
    window_start_s: float,
    window_end_s: float,
    min_pairs: int,
    thresholds: ErrorThresholds,
    motion_thresholds: PhaseMotionThresholds,
) -> dict[str, object]:
    """Analyze one phase using only phase-start-relative SE(3) motion."""

    selected = select_elapsed_window(pairs, window_start_s, window_end_s)
    base: dict[str, object] = {
        "name": name,
        "kind": phase_kind,
        "window_elapsed_s": {"start": window_start_s, "end": window_end_s},
        "pair_count": len(selected),
        "min_pair_count": min_pairs,
        "comparison": "T_phase_start^-1 * T_sample (absolute positions excluded)",
        "thresholds": asdict(thresholds),
    }
    if not selected:
        base.update(
            {
                "reference": None,
                "oracle_relative_motion": None,
                "errors": None,
                "checks": {
                    "sample_count": {"actual": 0, "minimum": min_pairs, "ok": False}
                },
                "failures": ["no synchronized pose pairs in phase window"],
                "ok": False,
            }
        )
        return base

    reference = selected[0]
    xy_errors: list[float] = []
    z_errors: list[float] = []
    yaw_errors_deg: list[float] = []
    oracle_translations: list[float] = []
    oracle_xy: list[float] = []
    oracle_z: list[float] = []
    oracle_yaw_deg: list[float] = []
    stamp_deltas: list[float] = []
    for pair in selected:
        oracle_motion = relative_motion(reference.oracle, pair.oracle)
        estimate_motion = relative_motion(reference.estimate, pair.estimate)
        error = tuple(
            estimate_motion.translation[index] - oracle_motion.translation[index]
            for index in range(3)
        )
        xy_errors.append(math.hypot(error[0], error[1]))
        z_errors.append(error[2])
        orientation_error = quaternion_multiply(
            quaternion_conjugate(oracle_motion.orientation_xyzw),
            estimate_motion.orientation_xyzw,
        )
        yaw_errors_deg.append(math.degrees(wrap_radians(yaw_radians(orientation_error))))
        oracle_translations.append(math.sqrt(sum(value * value for value in oracle_motion.translation)))
        oracle_xy.append(math.hypot(oracle_motion.translation[0], oracle_motion.translation[1]))
        oracle_z.append(oracle_motion.translation[2])
        oracle_yaw_deg.append(
            math.degrees(wrap_radians(yaw_radians(oracle_motion.orientation_xyzw)))
        )
        stamp_deltas.append(pair.stamp_delta_s)

    xy_metrics = _error_metrics(xy_errors)
    z_metrics = _error_metrics(z_errors)
    yaw_metrics = _error_metrics(yaw_errors_deg)
    sync_metrics = _error_metrics(stamp_deltas)
    max_translation_m = max(oracle_translations)
    max_yaw_deg = max(abs(value) for value in oracle_yaw_deg)
    motion_check = _motion_check(
        phase_kind, max_translation_m, max_yaw_deg, motion_thresholds
    )
    checks = {
        "sample_count": {
            "actual": len(selected),
            "minimum": min_pairs,
            "ok": len(selected) >= min_pairs,
        },
        "oracle_phase_motion": motion_check,
        "xy_rms_m": _upper_limit_check(xy_metrics["rms"], thresholds.xy_rms_m),
        "xy_max_m": _upper_limit_check(xy_metrics["max_abs"], thresholds.xy_max_m),
        "z_rms_m": _upper_limit_check(z_metrics["rms"], thresholds.z_rms_m),
        "z_max_m": _upper_limit_check(z_metrics["max_abs"], thresholds.z_max_m),
        "yaw_rms_deg": _upper_limit_check(
            yaw_metrics["rms"], thresholds.yaw_rms_deg
        ),
        "yaw_max_deg": _upper_limit_check(
            yaw_metrics["max_abs"], thresholds.yaw_max_deg
        ),
    }
    failures = [label for label, check in checks.items() if not bool(check["ok"])]
    base.update(
        {
            # Timestamps identify the relative reference without leaking or
            # comparing either topic's absolute position.
            "reference": {
                "oracle_stamp_s": reference.oracle.stamp_s,
                "estimate_stamp_s": reference.estimate.stamp_s,
                "elapsed_s": reference.elapsed_s,
            },
            "duration_s": selected[-1].elapsed_s - reference.elapsed_s,
            "stamp_sync_error_s": sync_metrics,
            "oracle_relative_motion": {
                "translation_m": {
                    "max_3d": max_translation_m,
                    "max_xy": max(oracle_xy),
                    "max_abs_z": max(abs(value) for value in oracle_z),
                    "final_3d": oracle_translations[-1],
                },
                "yaw_deg": {
                    "max_abs": max_yaw_deg,
                    "final": oracle_yaw_deg[-1],
                },
            },
            "errors": {
                "relative_xy_m": xy_metrics,
                "relative_z_m": z_metrics,
                "relative_yaw_deg": yaw_metrics,
            },
            "checks": checks,
            "failures": failures,
            "ok": not failures,
        }
    )
    return base


__all__ = [
    "ErrorThresholds",
    "MatchedPair",
    "PhaseMotionThresholds",
    "PoseSample",
    "RelativeMotion",
    "analyze_phase",
    "match_samples_by_stamp",
    "normalize_quaternion",
    "quaternion_multiply",
    "relative_motion",
    "rotate_vector",
    "select_elapsed_window",
    "wrap_radians",
    "yaw_radians",
]
