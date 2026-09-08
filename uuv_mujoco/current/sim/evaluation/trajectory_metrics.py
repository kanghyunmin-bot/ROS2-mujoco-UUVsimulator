"""Deterministic trajectory synchronization, alignment, ATE, and RPE metrics.

Ground truth enters only this evaluation layer. The module has no ROS imports
and cannot publish or feed an estimator/controller.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np


_EPS = 1.0e-12


@dataclass(frozen=True)
class Trajectory:
    """Timestamped poses.

    Attributes:
        timestamps_s: Strictly increasing reference-clock timestamps [s],
            shape ``[sample_count]``.
        positions_m: Cartesian positions [m], shape ``[sample_count, 3]``.
        quaternions_xyzw: Unit quaternions in ``x, y, z, w`` order, shape
            ``[sample_count, 4]``.
    """

    timestamps_s: np.ndarray
    positions_m: np.ndarray
    quaternions_xyzw: np.ndarray

    def __post_init__(self) -> None:
        timestamps = np.asarray(self.timestamps_s, dtype=np.float64).copy()
        positions = np.asarray(self.positions_m, dtype=np.float64).copy()
        quaternions = np.asarray(self.quaternions_xyzw, dtype=np.float64).copy()
        if timestamps.ndim != 1:
            raise ValueError("timestamps_s must have shape [sample_count]")
        if positions.shape != (timestamps.size, 3):
            raise ValueError("positions_m must have shape [sample_count, 3]")
        if quaternions.shape != (timestamps.size, 4):
            raise ValueError("quaternions_xyzw must have shape [sample_count, 4]")
        if timestamps.size < 2:
            raise ValueError("trajectory requires at least two samples")
        if not (
            np.all(np.isfinite(timestamps))
            and np.all(np.isfinite(positions))
            and np.all(np.isfinite(quaternions))
        ):
            raise ValueError("trajectory contains non-finite values")
        if np.any(np.diff(timestamps) <= 0.0):
            raise ValueError("trajectory timestamps must be strictly increasing")
        norms = np.linalg.norm(quaternions, axis=1)
        if np.any(norms <= _EPS):
            raise ValueError("trajectory contains a zero-norm quaternion")
        quaternions /= norms[:, None]
        timestamps.setflags(write=False)
        positions.setflags(write=False)
        quaternions.setflags(write=False)
        object.__setattr__(self, "timestamps_s", timestamps)
        object.__setattr__(self, "positions_m", positions)
        object.__setattr__(self, "quaternions_xyzw", quaternions)


@dataclass(frozen=True)
class _Alignment:
    rotation: np.ndarray
    translation_m: np.ndarray
    scale: float


def load_tum_trajectory(path: str | Path) -> Trajectory:
    """Load a TUM trajectory with ``t tx ty tz qx qy qz qw`` columns."""

    rows: list[list[float]] = []
    for line_number, raw_line in enumerate(
        Path(path).read_text(encoding="utf-8").splitlines(),
        start=1,
    ):
        line = raw_line.split("#", maxsplit=1)[0].strip()
        if not line:
            continue
        tokens = line.replace(",", " ").split()
        if tokens[0].lower() in {"timestamp", "time", "t"}:
            continue
        if len(tokens) != 8:
            raise ValueError(f"{path}:{line_number}: expected 8 columns")
        try:
            rows.append([float(token) for token in tokens])
        except ValueError as exc:
            raise ValueError(f"{path}:{line_number}: non-numeric trajectory row") from exc
    if len(rows) < 2:
        raise ValueError(f"{path}: trajectory requires at least two data rows")
    values = np.asarray(rows, dtype=np.float64)
    return Trajectory(values[:, 0], values[:, 1:4], values[:, 4:8])


def evaluate_trajectory(
    estimate: Trajectory,
    ground_truth: Trajectory,
    *,
    alignment: str = "se3",
    estimate_time_offset_s: float = 0.0,
    max_interpolation_gap_s: float = 0.1,
    rpe_delta_s: float = 1.0,
    rpe_tolerance_s: float = 0.05,
) -> dict[str, object]:
    """Synchronize an estimate to truth and calculate ATE/RPE.

    Args:
        estimate: Estimated trajectory. Ground truth must never have been an
            estimator input.
        ground_truth: Evaluation-only reference trajectory.
        alignment: ``"none"``, rigid ``"se3"``, or scale-aware ``"sim3"``.
        estimate_time_offset_s: Offset added to estimate timestamps [s]. Use a
            separately identified device-to-reference clock correction.
        max_interpolation_gap_s: Largest truth bracket accepted for one
            interpolated correspondence [s].
        rpe_delta_s: Requested RPE pair separation [s].
        rpe_tolerance_s: Maximum separation error accepted for an RPE pair [s].

    Returns:
        JSON-serializable metrics and synchronization diagnostics.
    """

    if alignment not in {"none", "se3", "sim3"}:
        raise ValueError("alignment must be one of: none, se3, sim3")
    for name, value, allow_zero in (
        ("max_interpolation_gap_s", max_interpolation_gap_s, False),
        ("rpe_delta_s", rpe_delta_s, False),
        ("rpe_tolerance_s", rpe_tolerance_s, True),
    ):
        if not np.isfinite(value) or value < 0.0 or (not allow_zero and value == 0.0):
            raise ValueError(f"{name} must be finite and {'non-negative' if allow_zero else 'positive'}")
    if not np.isfinite(estimate_time_offset_s):
        raise ValueError("estimate_time_offset_s must be finite")

    shifted_times = estimate.timestamps_s + float(estimate_time_offset_s)
    query_indices, truth_positions, truth_quaternions = _interpolate_truth(
        ground_truth,
        shifted_times,
        max_gap_s=float(max_interpolation_gap_s),
    )
    if query_indices.size < 3:
        raise ValueError("fewer than three synchronized trajectory samples")
    estimate_positions = estimate.positions_m[query_indices]
    estimate_quaternions = estimate.quaternions_xyzw[query_indices]
    synchronized_times = shifted_times[query_indices]

    transform = _solve_alignment(
        estimate_positions,
        truth_positions,
        alignment=alignment,
    )
    aligned_positions = (
        transform.scale * (transform.rotation @ estimate_positions.T).T
        + transform.translation_m
    )
    aligned_rotations = np.stack(
        [transform.rotation @ _quat_to_rotation(quaternion) for quaternion in estimate_quaternions]
    )
    truth_rotations = np.stack(
        [_quat_to_rotation(quaternion) for quaternion in truth_quaternions]
    )

    ate_translation = np.linalg.norm(aligned_positions - truth_positions, axis=1)
    ate_rotation = np.asarray(
        [
            _rotation_angle_rad(reference.T @ measured)
            for reference, measured in zip(truth_rotations, aligned_rotations, strict=True)
        ],
        dtype=np.float64,
    )
    rpe_translation, rpe_rotation, rpe_pair_dt = _relative_pose_errors(
        synchronized_times,
        aligned_positions,
        aligned_rotations,
        truth_positions,
        truth_rotations,
        delta_s=float(rpe_delta_s),
        tolerance_s=float(rpe_tolerance_s),
    )

    return {
        "schema": "uuv_mujoco.slam_trajectory_metrics.v1",
        "alignment": alignment,
        "alignment_scale": float(transform.scale),
        "alignment_rotation": transform.rotation.tolist(),
        "alignment_translation_m": transform.translation_m.tolist(),
        "estimate_time_offset_s": float(estimate_time_offset_s),
        "estimate_sample_count": int(estimate.timestamps_s.size),
        "ground_truth_sample_count": int(ground_truth.timestamps_s.size),
        "matched_sample_count": int(query_indices.size),
        "match_ratio": float(query_indices.size / estimate.timestamps_s.size),
        "matched_time_start_s": float(synchronized_times[0]),
        "matched_time_end_s": float(synchronized_times[-1]),
        "max_interpolation_gap_s": float(max_interpolation_gap_s),
        "ate_translation_m": _error_summary(ate_translation),
        "ate_rotation_deg": _error_summary(np.degrees(ate_rotation)),
        "rpe_delta_requested_s": float(rpe_delta_s),
        "rpe_pair_count": int(rpe_translation.size),
        "rpe_pair_dt_s": _error_summary(rpe_pair_dt),
        "rpe_translation_m": _error_summary(rpe_translation),
        "rpe_rotation_deg": _error_summary(np.degrees(rpe_rotation)),
    }


def _interpolate_truth(
    truth: Trajectory,
    query_times_s: np.ndarray,
    *,
    max_gap_s: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    accepted_indices: list[int] = []
    positions: list[np.ndarray] = []
    quaternions: list[np.ndarray] = []
    times = truth.timestamps_s
    for query_index, query_time in enumerate(query_times_s):
        if query_time < times[0] or query_time > times[-1]:
            continue
        right = int(np.searchsorted(times, query_time, side="left"))
        if right < times.size and abs(times[right] - query_time) <= _EPS:
            accepted_indices.append(query_index)
            positions.append(truth.positions_m[right])
            quaternions.append(truth.quaternions_xyzw[right])
            continue
        if right == 0 or right >= times.size:
            continue
        left = right - 1
        bracket_s = float(times[right] - times[left])
        if bracket_s > max_gap_s + _EPS:
            continue
        fraction = float((query_time - times[left]) / bracket_s)
        accepted_indices.append(query_index)
        positions.append(
            (1.0 - fraction) * truth.positions_m[left]
            + fraction * truth.positions_m[right]
        )
        quaternions.append(
            _quaternion_slerp(
                truth.quaternions_xyzw[left],
                truth.quaternions_xyzw[right],
                fraction,
            )
        )
    return (
        np.asarray(accepted_indices, dtype=np.int64),
        np.asarray(positions, dtype=np.float64),
        np.asarray(quaternions, dtype=np.float64),
    )


def _solve_alignment(
    estimate_positions: np.ndarray,
    truth_positions: np.ndarray,
    *,
    alignment: str,
) -> _Alignment:
    if alignment == "none":
        return _Alignment(np.eye(3), np.zeros(3), 1.0)
    estimate_center = np.mean(estimate_positions, axis=0)
    truth_center = np.mean(truth_positions, axis=0)
    estimate_zero = estimate_positions - estimate_center
    truth_zero = truth_positions - truth_center
    covariance = truth_zero.T @ estimate_zero / estimate_positions.shape[0]
    left, singular_values, right_transpose = np.linalg.svd(covariance)
    signs = np.ones(3, dtype=np.float64)
    if np.linalg.det(left @ right_transpose) < 0.0:
        signs[-1] = -1.0
    rotation = left @ np.diag(signs) @ right_transpose
    scale = 1.0
    if alignment == "sim3":
        estimate_variance = float(np.mean(np.sum(estimate_zero**2, axis=1)))
        if estimate_variance <= _EPS:
            raise ValueError("cannot solve Sim(3) alignment for a stationary estimate")
        scale = float(np.dot(singular_values, signs) / estimate_variance)
        if not np.isfinite(scale) or scale <= 0.0:
            raise ValueError("alignment produced a non-positive scale")
    translation = truth_center - scale * rotation @ estimate_center
    return _Alignment(rotation, translation, scale)


def _relative_pose_errors(
    times_s: np.ndarray,
    estimate_positions: np.ndarray,
    estimate_rotations: np.ndarray,
    truth_positions: np.ndarray,
    truth_rotations: np.ndarray,
    *,
    delta_s: float,
    tolerance_s: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    translation_errors: list[float] = []
    rotation_errors: list[float] = []
    pair_dts: list[float] = []
    for start in range(times_s.size - 1):
        target = float(times_s[start] + delta_s)
        insertion = int(np.searchsorted(times_s, target, side="left"))
        candidates = [index for index in (insertion - 1, insertion) if index > start and index < times_s.size]
        if not candidates:
            continue
        end = min(candidates, key=lambda index: abs(float(times_s[index] - target)))
        pair_dt = float(times_s[end] - times_s[start])
        if abs(pair_dt - delta_s) > tolerance_s + _EPS:
            continue

        truth_relative_rotation = truth_rotations[start].T @ truth_rotations[end]
        estimate_relative_rotation = estimate_rotations[start].T @ estimate_rotations[end]
        truth_relative_translation = truth_rotations[start].T @ (
            truth_positions[end] - truth_positions[start]
        )
        estimate_relative_translation = estimate_rotations[start].T @ (
            estimate_positions[end] - estimate_positions[start]
        )
        error_rotation = truth_relative_rotation.T @ estimate_relative_rotation
        error_translation = truth_relative_rotation.T @ (
            estimate_relative_translation - truth_relative_translation
        )
        translation_errors.append(float(np.linalg.norm(error_translation)))
        rotation_errors.append(_rotation_angle_rad(error_rotation))
        pair_dts.append(pair_dt)
    return (
        np.asarray(translation_errors, dtype=np.float64),
        np.asarray(rotation_errors, dtype=np.float64),
        np.asarray(pair_dts, dtype=np.float64),
    )


def _error_summary(values: np.ndarray) -> dict[str, float | int | None]:
    values = np.asarray(values, dtype=np.float64)
    if values.size == 0:
        return {
            "count": 0,
            "rmse": None,
            "mean": None,
            "median": None,
            "p95": None,
            "max": None,
        }
    return {
        "count": int(values.size),
        "rmse": float(np.sqrt(np.mean(values**2))),
        "mean": float(np.mean(values)),
        "median": float(np.median(values)),
        "p95": float(np.percentile(values, 95.0)),
        "max": float(np.max(values)),
    }


def _quaternion_slerp(first: np.ndarray, second: np.ndarray, fraction: float) -> np.ndarray:
    first = np.asarray(first, dtype=np.float64)
    second = np.asarray(second, dtype=np.float64)
    dot = float(np.dot(first, second))
    if dot < 0.0:
        second = -second
        dot = -dot
    dot = float(np.clip(dot, -1.0, 1.0))
    if dot > 0.9995:
        result = first + fraction * (second - first)
        return result / np.linalg.norm(result)
    angle = float(np.arccos(dot))
    sin_angle = float(np.sin(angle))
    return (
        np.sin((1.0 - fraction) * angle) / sin_angle * first
        + np.sin(fraction * angle) / sin_angle * second
    )


def _quat_to_rotation(quaternion_xyzw: Iterable[float]) -> np.ndarray:
    x, y, z, w = np.asarray(tuple(quaternion_xyzw), dtype=np.float64)
    norm = float(np.sqrt(x * x + y * y + z * z + w * w))
    if norm <= _EPS:
        raise ValueError("zero-norm quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _rotation_angle_rad(rotation: np.ndarray) -> float:
    cosine = float(np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0))
    return float(np.arccos(cosine))
