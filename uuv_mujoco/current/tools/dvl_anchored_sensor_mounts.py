"""Rebase recorded sensor mounts while keeping the physical DVL pose fixed.

The physical DVL frame and a historically published DVL frame can use different
axis conventions. Their basis transform must be supplied explicitly. Treating
a historical FLU-to-FRD label change as a physical mounting change would turn
the entire robot upside down.
"""

from __future__ import annotations

from collections.abc import Mapping

import numpy as np


def _rigid_transform(value: np.ndarray, name: str) -> np.ndarray:
    matrix = np.asarray(value, dtype=np.float64)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError(f"{name} must be a finite 4x4 transform")
    rotation = matrix[:3, :3]
    if (not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-10, rtol=0)
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-10, rtol=0)
            or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-10, rtol=0)):
        raise ValueError(f"{name} must be a proper rigid transform")
    return matrix.copy()


def sensor_mounts_from_dvl_anchor(
    *,
    model_from_dvl_anchor: np.ndarray,
    anchor_from_recorded_dvl: np.ndarray,
    recorded_base_from_dvl: np.ndarray,
    recorded_base_from_sensors: Mapping[str, np.ndarray],
) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    """Resolve recorded sensor poses in a model with a fixed DVL anchor.

    Args:
        model_from_dvl_anchor: Physical DVL pose in the model body frame;
            homogeneous translation [m]. It is not inferred from a trajectory.
        anchor_from_recorded_dvl: Explicit rotation from the historically
            declared DVL axes to physical anchor axes. Translation [m] must be
            zero because both frames refer to the same fixed DVL origin.
        recorded_base_from_dvl: Published DVL pose in the recorded base frame;
            homogeneous translation [m].
        recorded_base_from_sensors: Composed recorded sensor poses, keyed by
            frame name; homogeneous translations [m].

    Returns:
        Model-from-recorded-base transform and model-from-sensor transforms,
        all with translations [m]. The former must also be used when comparing
        base-frame poses or lever arms across these coordinate origins.

    Raises:
        ValueError: A transform is invalid or the basis transform moves the
            supposedly fixed physical DVL origin.
    """
    model_anchor = _rigid_transform(model_from_dvl_anchor, "model_from_dvl_anchor")
    basis = _rigid_transform(anchor_from_recorded_dvl, "anchor_from_recorded_dvl")
    recorded_dvl = _rigid_transform(recorded_base_from_dvl, "recorded_base_from_dvl")
    if not np.allclose(basis[:3, 3], 0, atol=1e-12, rtol=0):
        raise ValueError("DVL axis conversion must not move the physical origin")
    model_base = model_anchor @ basis @ np.linalg.inv(recorded_dvl)
    sensors = {
        name: model_base @ _rigid_transform(value, name)
        for name, value in recorded_base_from_sensors.items()
    }
    # Published DVL axes may differ, but its physical origin must stay fixed.
    if not np.allclose((model_base @ recorded_dvl)[:3, 3], model_anchor[:3, 3], atol=1e-10, rtol=0):
        raise ValueError("DVL origin changed during sensor registration")
    return model_base, sensors


def initial_state_from_recorded_base(
    *,
    world_from_recorded_base: np.ndarray,
    model_from_recorded_base: np.ndarray,
    linear_velocity_recorded_body_mps: np.ndarray,
    angular_velocity_recorded_body_radps: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Convert one initial rigid-body state after rebasing its coordinate origin.

    Args:
        world_from_recorded_base: Initial recorded base pose, translation [m].
        model_from_recorded_base: Registration from recorded base to model body,
            translation [m], derived from the fixed DVL mounting constraint.
        linear_velocity_recorded_body_mps: Velocity of the recorded base origin,
            expressed in its body axes [m/s]. Zero is an explicit initialization
            assumption when no trustworthy linear-velocity observation exists.
        angular_velocity_recorded_body_radps: Angular velocity expressed in the
            recorded body axes [rad/s].

    Returns:
        Initial model pose with translation [m], model-origin linear velocity
        in model body axes [m/s], and model-body angular velocity [rad/s].
        Transforming velocity includes the angular-rate cross lever-arm term;
        preserving zero velocity at both different origins is generally wrong.
    """
    world_base = _rigid_transform(world_from_recorded_base, "world_from_recorded_base")
    model_base = _rigid_transform(model_from_recorded_base, "model_from_recorded_base")
    linear = np.asarray(linear_velocity_recorded_body_mps, dtype=np.float64)
    angular = np.asarray(angular_velocity_recorded_body_radps, dtype=np.float64)
    if linear.shape != (3,) or angular.shape != (3,) or not np.isfinite([linear, angular]).all():
        raise ValueError("Initial body velocities must be finite three-vectors")
    angular_model = model_base[:3, :3] @ angular
    linear_model = model_base[:3, :3] @ linear - np.cross(angular_model, model_base[:3, 3])
    return world_base @ np.linalg.inv(model_base), linear_model, angular_model
