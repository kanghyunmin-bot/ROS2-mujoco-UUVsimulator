"""Range-difference source localization for a moving single hydrophone."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class SourceEstimate:
    source_world: np.ndarray
    initial_range_m: float
    bias_range_rate_mps: float
    rms_residual_m: float
    condition_number: float
    sample_count: int


@dataclass(frozen=True)
class DepthSafetyResult:
    """Result of applying the positive-down vehicle depth safety contract."""

    requested_heave: float
    command_heave: float
    vehicle_depth_m: float | None
    limit_active: bool
    recovery_active: bool


def derive_vehicle_depth_limit(
    tank_max_depth_m: float,
    *,
    vehicle_bottom_extent_m: float = 0.17,
    floor_clearance_m: float = 0.10,
) -> float:
    """Derive a positive-down base-link limit from the only pool-depth input."""

    tank_depth = float(tank_max_depth_m)
    if not math.isfinite(tank_depth) or tank_depth <= 0.0:
        return 0.0
    reserved = max(float(vehicle_bottom_extent_m), 0.0) + max(float(floor_clearance_m), 0.0)
    return max(0.0, tank_depth - reserved)


def select_auto_probe_heave(
    vehicle_depth_m: float | None,
    *,
    max_vehicle_depth_m: float,
    shallow_vehicle_depth_m: float = 0.45,
    magnitude: float = 0.10,
) -> float:
    """Choose the vertical probe direction with more safe travel available."""

    if vehicle_depth_m is None or not math.isfinite(float(vehicle_depth_m)):
        return 0.0
    depth = max(0.0, float(vehicle_depth_m))
    maximum = float(max_vehicle_depth_m)
    if not math.isfinite(maximum) or maximum <= 0.0:
        return 0.0
    upward_room = max(0.0, depth - max(float(shallow_vehicle_depth_m), 0.0))
    downward_room = max(0.0, maximum - depth)
    command = -abs(float(magnitude)) if downward_room >= upward_room else abs(float(magnitude))
    return float(np.clip(command, -0.4, 0.4))


def limit_heave_by_vehicle_depth(
    requested_heave: float,
    *,
    vehicle_z_world: float | None,
    max_vehicle_depth_m: float,
    soft_margin_m: float,
    recovery_heave: float,
) -> DepthSafetyResult:
    """Limit normalized heave using a positive-down base-link depth.

    Negative heave is the controller's downward command.  An enabled limit
    progressively attenuates that command inside ``soft_margin_m`` and applies
    a small upward recovery command at or below the hard depth limit.  If the
    pose is unavailable, downward motion is fail-closed while upward commands
    remain available.
    """

    requested = float(np.clip(requested_heave, -1.0, 1.0))
    maximum = float(max_vehicle_depth_m)
    margin = max(float(soft_margin_m), 0.0)
    recovery = float(np.clip(recovery_heave, 0.0, 1.0))
    if not math.isfinite(maximum) or maximum <= 0.0:
        return DepthSafetyResult(requested, requested, None, False, False)

    if vehicle_z_world is None or not math.isfinite(float(vehicle_z_world)):
        command = max(requested, 0.0)
        return DepthSafetyResult(requested, command, None, command != requested, False)

    depth = max(0.0, -float(vehicle_z_world))
    if depth >= maximum:
        command = max(requested, recovery)
        return DepthSafetyResult(requested, command, depth, True, recovery > 0.0)

    remaining = maximum - depth
    if requested < 0.0 and margin > 0.0 and remaining < margin:
        scale = float(np.clip(remaining / margin, 0.0, 1.0))
        command = requested * scale
        return DepthSafetyResult(requested, command, depth, command != requested, False)

    return DepthSafetyResult(requested, requested, depth, False, False)


def update_range_success_timer(
    distance_m: float | None,
    *,
    threshold_m: float,
    hold_s: float,
    now_s: float,
    started_s: float | None,
) -> tuple[float | None, bool]:
    """Advance a continuous sensor-range success timer."""

    threshold = float(threshold_m)
    if (
        threshold <= 0.0
        or distance_m is None
        or not math.isfinite(float(distance_m))
        or float(distance_m) > threshold
    ):
        return None, False
    now = float(now_s)
    if started_s is None or not math.isfinite(float(started_s)):
        return now, False
    return float(started_s), now - float(started_s) >= max(float(hold_s), 0.0)


def filter_unit_vector(
    previous: np.ndarray | None,
    current: np.ndarray,
    *,
    alpha: float,
) -> np.ndarray:
    """Low-pass a direction without introducing angle wrap discontinuities."""

    value = np.asarray(current, dtype=np.float64).reshape(3)
    norm = float(np.linalg.norm(value))
    if not np.all(np.isfinite(value)) or norm <= 1.0e-9:
        raise ValueError("current direction must be finite and non-zero")
    value = value / norm
    if previous is None:
        return value
    prior = np.asarray(previous, dtype=np.float64).reshape(3)
    prior_norm = float(np.linalg.norm(prior))
    if not np.all(np.isfinite(prior)) or prior_norm <= 1.0e-9:
        return value
    blend = float(np.clip(alpha, 0.0, 1.0))
    filtered = (1.0 - blend) * (prior / prior_norm) + blend * value
    filtered_norm = float(np.linalg.norm(filtered))
    return value if filtered_norm <= 1.0e-9 else filtered / filtered_norm


def stabilized_yaw_command(
    bearing_rad: float,
    yaw_rate_rad_s: float,
    previous_command: float,
    *,
    dt_s: float,
    gain: float,
    rate_damping: float,
    deadband_rad: float,
    command_limit: float,
    slew_rate_per_s: float,
) -> float:
    """Return a damped, deadbanded and slew-limited normalized yaw command."""

    bearing = float(bearing_rad)
    yaw_rate = float(yaw_rate_rad_s)
    previous = float(previous_command)
    if not all(math.isfinite(value) for value in (bearing, yaw_rate, previous)):
        return 0.0
    if abs(bearing) <= max(float(deadband_rad), 0.0):
        target = 0.0
    else:
        target = float(gain) * bearing - float(rate_damping) * yaw_rate
    limit = max(float(command_limit), 0.0)
    target = float(np.clip(target, -limit, limit))
    max_step = max(float(slew_rate_per_s), 0.0) * max(float(dt_s), 0.0)
    if max_step <= 0.0:
        return target
    return float(np.clip(target, previous - max_step, previous + max_step))


def estimate_source_from_range_gradient(
    positions_world: np.ndarray,
    absolute_ranges_m: np.ndarray,
    *,
    source_z_world: float,
) -> SourceEstimate | None:
    """Estimate bearing from the spatial gradient of single-sensor range."""

    positions = np.asarray(positions_world, dtype=np.float64)
    ranges = np.asarray(absolute_ranges_m, dtype=np.float64).reshape(-1)
    valid = (
        np.all(np.isfinite(positions), axis=1)
        & np.isfinite(ranges)
        & (ranges > 1.0)
        & (ranges < 80.0)
    )
    positions = positions[valid]
    ranges = ranges[valid]
    if len(positions) < 20:
        return None
    centered_xy = positions[:, :2] - np.mean(positions[:, :2], axis=0)
    centered_ranges = ranges - float(np.mean(ranges))
    singular = np.linalg.svd(centered_xy, compute_uv=False)
    if len(singular) < 2 or singular[-1] < 0.05:
        return None
    try:
        gradient, *_ = np.linalg.lstsq(centered_xy, centered_ranges, rcond=1.0e-8)
    except np.linalg.LinAlgError:
        return None
    gradient_norm = float(np.linalg.norm(gradient))
    if not np.all(np.isfinite(gradient)) or gradient_norm < 0.05:
        return None
    horizontal_direction = -gradient / gradient_norm
    current = positions[-1]
    current_range = float(np.median(ranges[-min(9, len(ranges)) :]))
    dz = float(source_z_world) - float(current[2])
    horizontal_range = math.sqrt(max(current_range**2 - dz**2, 0.25))
    source = np.array(
        [
            current[0] + horizontal_direction[0] * horizontal_range,
            current[1] + horizontal_direction[1] * horizontal_range,
            float(source_z_world),
        ],
        dtype=np.float64,
    )
    predicted = np.linalg.norm(source[None, :] - positions, axis=1)
    residual = predicted - ranges
    condition = float("inf") if singular[-1] <= 1.0e-9 else float(singular[0] / singular[-1])
    return SourceEstimate(
        source_world=source,
        initial_range_m=float(ranges[0]),
        bias_range_rate_mps=0.0,
        rms_residual_m=float(np.sqrt(np.mean(residual**2))),
        condition_number=condition,
        sample_count=len(positions),
    )


def estimate_source_xy_from_absolute_ranges(
    positions_world: np.ndarray,
    absolute_ranges_m: np.ndarray,
    *,
    source_z_world: float,
    initial_source_world: np.ndarray | None = None,
    huber_scale_m: float = 0.20,
) -> SourceEstimate | None:
    """Locate a stationary source on a known-depth plane from one moving sensor."""

    positions = np.asarray(positions_world, dtype=np.float64)
    ranges = np.asarray(absolute_ranges_m, dtype=np.float64).reshape(-1)
    if positions.ndim != 2 or positions.shape[1] != 3 or len(positions) != len(ranges):
        raise ValueError("positions_world must have shape (N, 3) matching ranges")
    valid = (
        np.all(np.isfinite(positions), axis=1)
        & np.isfinite(ranges)
        & (ranges > 0.1)
        & (ranges < 80.0)
    )
    positions = positions[valid]
    ranges = ranges[valid]
    if len(positions) < 20:
        return None

    origin = positions[0]
    displacement = positions - origin
    z_local = float(source_z_world) - float(origin[2])
    horizontal_span = displacement[:, :2]
    singular = np.linalg.svd(horizontal_span, compute_uv=False)
    if len(singular) < 2 or singular[-1] < 0.05:
        return None

    delta = displacement[1:]
    matrix = -2.0 * delta[:, :2]
    rhs = (
        ranges[1:] ** 2
        - ranges[0] ** 2
        - np.sum(delta**2, axis=1)
        + 2.0 * z_local * delta[:, 2]
    )
    try:
        xy, *_ = np.linalg.lstsq(matrix, rhs, rcond=1.0e-8)
    except np.linalg.LinAlgError:
        return None
    if not np.all(np.isfinite(xy)):
        return None
    if initial_source_world is not None:
        seeded = np.asarray(initial_source_world, dtype=np.float64).reshape(3) - origin
        if np.all(np.isfinite(seeded)) and np.linalg.norm(seeded[:2]) > 0.1:
            xy = seeded[:2].copy()

    damping = 1.0e-3
    scale = max(float(huber_scale_m), 0.03)
    for _ in range(25):
        source_local = np.array([xy[0], xy[1], z_local], dtype=np.float64)
        relative = source_local[None, :] - displacement
        predicted = np.linalg.norm(relative, axis=1)
        safe = np.maximum(predicted, 1.0e-8)
        residual = predicted - ranges
        weights = np.ones_like(residual)
        large = np.abs(residual) > scale
        weights[large] = scale / np.maximum(np.abs(residual[large]), 1.0e-12)
        jacobian = relative[:, :2] / safe[:, None]
        weighted_jacobian = jacobian * np.sqrt(weights)[:, None]
        weighted_residual = residual * np.sqrt(weights)
        normal = weighted_jacobian.T @ weighted_jacobian
        gradient = weighted_jacobian.T @ weighted_residual
        try:
            step = np.linalg.solve(normal + damping * np.eye(2), -gradient)
        except np.linalg.LinAlgError:
            return None
        if not np.all(np.isfinite(step)):
            return None
        xy += step
        if float(np.linalg.norm(step)) < 1.0e-5:
            break

    source_local = np.array([xy[0], xy[1], z_local], dtype=np.float64)
    relative = source_local[None, :] - displacement
    predicted = np.linalg.norm(relative, axis=1)
    residual = predicted - ranges
    jacobian = relative[:, :2] / np.maximum(predicted, 1.0e-8)[:, None]
    singular = np.linalg.svd(jacobian, compute_uv=False)
    condition = float("inf") if singular[-1] <= 1.0e-9 else float(singular[0] / singular[-1])
    return SourceEstimate(
        source_world=origin + source_local,
        initial_range_m=float(ranges[0]),
        bias_range_rate_mps=0.0,
        rms_residual_m=float(np.sqrt(np.mean(residual**2))),
        condition_number=condition,
        sample_count=len(positions),
    )


def estimate_source_from_range_differences(
    positions_world: np.ndarray,
    cumulative_range_changes_m: np.ndarray,
    times_s: np.ndarray,
    *,
    initial_source_world: np.ndarray | None = None,
    absolute_ranges_m: np.ndarray | None = None,
    min_source_z_world: float | None = None,
    max_source_z_world: float | None = None,
    fixed_source_z_world: float | None = None,
    huber_scale_m: float = 0.005,
    absolute_range_scale_m: float = 0.45,
) -> SourceEstimate | None:
    """Estimate a stationary source without using an absolute range measurement.

    The measurement model is
      q_i = ||source - position_i|| - r_0 + bias * (t_i - t_0).
    Vehicle motion therefore acts as a synthetic array. Non-coplanar motion is
    required to estimate all three source coordinates.
    """

    positions = np.asarray(positions_world, dtype=np.float64)
    changes = np.asarray(cumulative_range_changes_m, dtype=np.float64).reshape(-1)
    times = np.asarray(times_s, dtype=np.float64).reshape(-1)
    if positions.ndim != 2 or positions.shape[1] != 3:
        raise ValueError("positions_world must have shape (N, 3)")
    if len(positions) != len(changes) or len(positions) != len(times) or len(positions) < 12:
        return None
    if not np.all(np.isfinite(positions)) or not np.all(np.isfinite(changes)) or not np.all(np.isfinite(times)):
        return None

    origin = positions[0].copy()
    displacement = positions - origin
    q = changes - changes[0]
    t = times - times[0]
    if float(np.max(np.linalg.norm(displacement, axis=1))) < 0.08:
        return None

    absolute_ranges = None
    if absolute_ranges_m is not None:
        candidate = np.asarray(absolute_ranges_m, dtype=np.float64).reshape(-1)
        if len(candidate) == len(positions):
            absolute_ranges = candidate
    params = _initial_parameters(
        displacement,
        q,
        origin,
        initial_source_world,
        absolute_ranges,
    )
    min_source_z_local = None
    if min_source_z_world is not None and np.isfinite(min_source_z_world):
        min_source_z_local = float(min_source_z_world) - float(origin[2])
    max_source_z_local = None
    if max_source_z_world is not None and np.isfinite(max_source_z_world):
        max_source_z_local = float(max_source_z_world) - float(origin[2])
    if (
        min_source_z_local is not None
        and max_source_z_local is not None
        and min_source_z_local > max_source_z_local
    ):
        raise ValueError("min_source_z_world must not exceed max_source_z_world")
    if min_source_z_local is not None:
        params[2] = max(float(params[2]), min_source_z_local)
    if max_source_z_local is not None:
        params[2] = min(float(params[2]), max_source_z_local)
    fixed_source_z_local = None
    if fixed_source_z_world is not None and np.isfinite(fixed_source_z_world):
        fixed_source_z_local = float(fixed_source_z_world) - float(origin[2])
        params[2] = fixed_source_z_local
    active_indices = np.array([0, 1, 3, 4] if fixed_source_z_local is not None else [0, 1, 2, 3, 4])
    damping = 1.0e-3
    for _ in range(30):
        residual, jacobian = _residual_and_jacobian(params, displacement, q, t)
        scale = max(float(huber_scale_m), 1.0e-4)
        weights = np.ones_like(residual)
        large = np.abs(residual) > scale
        weights[large] = scale / np.maximum(np.abs(residual[large]), 1.0e-12)
        weighted_jacobian = jacobian * np.sqrt(weights)[:, None]
        weighted_residual = residual * np.sqrt(weights)
        normal = weighted_jacobian.T @ weighted_jacobian
        gradient = weighted_jacobian.T @ weighted_residual
        if absolute_ranges is not None:
            valid = np.isfinite(absolute_ranges) & (absolute_ranges > 0.1) & (absolute_ranges < 80.0)
            if np.count_nonzero(valid) >= 8:
                abs_scale = max(float(absolute_range_scale_m), 0.05)
                amplitude_range = float(np.median(absolute_ranges[valid] - q[valid]))
                prior_weight = 1.0 / (abs_scale * abs_scale)
                normal[3, 3] += prior_weight
                gradient[3] += (params[3] - amplitude_range) * prior_weight
                relative = params[:3][None, :] - displacement[valid]
                predicted = np.linalg.norm(relative, axis=1)
                safe = np.maximum(predicted, 1.0e-8)
                absolute_residual = predicted - absolute_ranges[valid]
                absolute_jacobian = np.zeros((len(predicted), 5), dtype=np.float64)
                absolute_jacobian[:, :3] = relative / safe[:, None]
                robust = np.ones_like(absolute_residual)
                large_abs = np.abs(absolute_residual) > 2.0 * abs_scale
                robust[large_abs] = (2.0 * abs_scale) / np.maximum(
                    np.abs(absolute_residual[large_abs]), 1.0e-12
                )
                absolute_weights = robust / (abs_scale * abs_scale)
                normal += absolute_jacobian.T @ (absolute_jacobian * absolute_weights[:, None])
                gradient += absolute_jacobian.T @ (absolute_residual * absolute_weights)
        # The simulator uses a fixed coherent carrier and sample clock. Keep a
        # small drift state for realism, but do not let it absorb source range.
        bias_sigma_mps = 0.04
        normal[4, 4] += 1.0 / (bias_sigma_mps * bias_sigma_mps)
        gradient[4] += params[4] / (bias_sigma_mps * bias_sigma_mps)
        try:
            active_normal = normal[np.ix_(active_indices, active_indices)]
            active_gradient = gradient[active_indices]
            active_step = np.linalg.solve(
                active_normal + damping * np.eye(len(active_indices)), -active_gradient
            )
        except np.linalg.LinAlgError:
            return None
        step = np.zeros(5, dtype=np.float64)
        step[active_indices] = active_step
        if not np.all(np.isfinite(step)):
            return None
        trial = params + step
        if min_source_z_local is not None:
            trial[2] = max(float(trial[2]), min_source_z_local)
        if max_source_z_local is not None:
            trial[2] = min(float(trial[2]), max_source_z_local)
        if fixed_source_z_local is not None:
            trial[2] = fixed_source_z_local
        trial[3] = float(np.clip(trial[3], 0.10, 80.0))
        trial[4] = float(np.clip(trial[4], -0.50, 0.50))
        current_cost = _objective_cost(
            params,
            displacement,
            q,
            t,
            absolute_ranges,
            huber_scale_m=scale,
            absolute_range_scale_m=absolute_range_scale_m,
        )
        trial_cost = _objective_cost(
            trial,
            displacement,
            q,
            t,
            absolute_ranges,
            huber_scale_m=scale,
            absolute_range_scale_m=absolute_range_scale_m,
        )
        if trial_cost <= current_cost:
            params = trial
            damping = max(1.0e-7, damping * 0.4)
            if float(np.linalg.norm(step)) < 1.0e-5:
                break
        else:
            damping = min(1.0e3, damping * 8.0)

    residual, jacobian = _residual_and_jacobian(params, displacement, q, t)
    information_jacobian = jacobian
    if absolute_ranges is not None:
        valid = np.isfinite(absolute_ranges) & (absolute_ranges > 0.1) & (absolute_ranges < 80.0)
        if np.count_nonzero(valid) >= 8:
            prior_jacobian = np.zeros((1, 5), dtype=np.float64)
            prior_jacobian[0, 3] = 1.0 / max(float(absolute_range_scale_m), 0.05)
            relative = params[:3][None, :] - displacement[valid]
            safe = np.maximum(np.linalg.norm(relative, axis=1), 1.0e-8)
            absolute_jacobian = np.zeros((np.count_nonzero(valid), 5), dtype=np.float64)
            absolute_jacobian[:, :3] = relative / safe[:, None]
            information_jacobian = np.vstack(
                (
                    jacobian,
                    prior_jacobian,
                    absolute_jacobian / max(float(absolute_range_scale_m), 0.05),
                )
            )
    singular_values = np.linalg.svd(information_jacobian[:, active_indices], compute_uv=False)
    condition = float("inf")
    if len(singular_values) and singular_values[-1] > 1.0e-9:
        condition = float(singular_values[0] / singular_values[-1])
    source_world = origin + params[:3]
    if not np.all(np.isfinite(source_world)):
        return None
    return SourceEstimate(
        source_world=source_world,
        initial_range_m=float(params[3]),
        bias_range_rate_mps=float(params[4]),
        rms_residual_m=float(np.sqrt(np.mean(residual**2))),
        condition_number=condition,
        sample_count=len(positions),
    )


def world_vector_to_body_flu(vector_world: np.ndarray, quaternion_xyzw: np.ndarray) -> np.ndarray:
    vector = np.asarray(vector_world, dtype=np.float64).reshape(3)
    qx, qy, qz, qw = np.asarray(quaternion_xyzw, dtype=np.float64).reshape(4)
    norm = float(np.linalg.norm([qx, qy, qz, qw]))
    if norm < 1.0e-9:
        raise ValueError("zero-length quaternion")
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    rotation = np.array(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ],
        dtype=np.float64,
    )
    return rotation.T @ vector


def _initial_parameters(
    displacement: np.ndarray,
    q: np.ndarray,
    origin: np.ndarray,
    initial_source_world: np.ndarray | None,
    absolute_ranges: np.ndarray | None,
) -> np.ndarray:
    rows = np.column_stack((-2.0 * displacement[1:], -2.0 * q[1:]))
    rhs = q[1:] ** 2 - np.sum(displacement[1:] ** 2, axis=1)
    try:
        linear, *_ = np.linalg.lstsq(rows, rhs, rcond=1.0e-8)
    except np.linalg.LinAlgError:
        linear = np.zeros(4, dtype=np.float64)
    source_local = linear[:3]
    initial_range = float(abs(linear[3]))
    amplitude_range = None
    if absolute_ranges is not None:
        valid_ranges = absolute_ranges[
            np.isfinite(absolute_ranges) & (absolute_ranges > 0.1) & (absolute_ranges < 80.0)
        ]
        if len(valid_ranges):
            valid_mask = np.isfinite(absolute_ranges) & (absolute_ranges > 0.1) & (absolute_ranges < 80.0)
            amplitude_range = float(np.median(absolute_ranges[valid_mask] - q[valid_mask]))
    if initial_source_world is not None:
        seeded = np.asarray(initial_source_world, dtype=np.float64).reshape(3) - origin
        if np.all(np.isfinite(seeded)) and np.linalg.norm(seeded) > 0.1:
            if amplitude_range is not None:
                source_local = seeded / np.linalg.norm(seeded) * amplitude_range
            elif not np.all(np.isfinite(source_local)) or np.linalg.norm(source_local) < 0.5:
                source_local = seeded
            elif float(np.dot(source_local, seeded)) < 0.0:
                source_local = seeded
    source_norm = float(np.linalg.norm(source_local))
    if not np.isfinite(source_norm) or source_norm < 0.5 or source_norm > 80.0:
        source_local = np.array([12.0, 0.0, -6.0], dtype=np.float64)
        source_norm = float(np.linalg.norm(source_local))
    if initial_range < 0.1 or initial_range > 80.0:
        initial_range = source_norm
    if amplitude_range is not None:
        initial_range = amplitude_range
    return np.array([*source_local, initial_range, 0.0], dtype=np.float64)


def _residual_and_jacobian(
    params: np.ndarray,
    displacement: np.ndarray,
    q: np.ndarray,
    t: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    source_local = params[:3]
    initial_range = float(params[3])
    bias = float(params[4])
    relative = source_local[None, :] - displacement
    ranges = np.linalg.norm(relative, axis=1)
    safe_ranges = np.maximum(ranges, 1.0e-8)
    residual = ranges - initial_range + bias * t - q
    jacobian = np.empty((len(displacement), 5), dtype=np.float64)
    jacobian[:, :3] = relative / safe_ranges[:, None]
    jacobian[:, 3] = -1.0
    jacobian[:, 4] = t
    return residual, jacobian


def _objective_cost(
    params: np.ndarray,
    displacement: np.ndarray,
    q: np.ndarray,
    t: np.ndarray,
    absolute_ranges: np.ndarray | None,
    *,
    huber_scale_m: float,
    absolute_range_scale_m: float,
) -> float:
    phase_residual, _ = _residual_and_jacobian(params, displacement, q, t)
    scale = max(float(huber_scale_m), 1.0e-4)
    magnitude = np.abs(phase_residual)
    cost = float(
        np.sum(
            np.where(
                magnitude <= scale,
                0.5 * phase_residual**2,
                scale * (magnitude - 0.5 * scale),
            )
        )
    )
    if absolute_ranges is not None:
        valid = np.isfinite(absolute_ranges) & (absolute_ranges > 0.1) & (absolute_ranges < 80.0)
        if np.count_nonzero(valid) >= 8:
            abs_scale = max(float(absolute_range_scale_m), 0.05)
            relative = params[:3][None, :] - displacement[valid]
            absolute_residual = np.linalg.norm(relative, axis=1) - absolute_ranges[valid]
            threshold = 2.0 * abs_scale
            absolute_magnitude = np.abs(absolute_residual)
            cost += float(
                np.sum(
                    np.where(
                        absolute_magnitude <= threshold,
                        0.5 * absolute_residual**2,
                        threshold * (absolute_magnitude - 0.5 * threshold),
                    )
                )
                / (abs_scale * abs_scale)
            )
            amplitude_initial_range = float(np.median(absolute_ranges[valid] - q[valid]))
            cost += 0.5 * ((float(params[3]) - amplitude_initial_range) / abs_scale) ** 2
    cost += 0.5 * (float(params[4]) / 0.04) ** 2
    return cost


__all__ = [
    "SourceEstimate",
    "estimate_source_from_range_gradient",
    "estimate_source_xy_from_absolute_ranges",
    "estimate_source_from_range_differences",
    "world_vector_to_body_flu",
]
