"""Keep the DVL physical origin fixed across historical frame conventions."""

import numpy as np
import pytest

from dvl_anchored_sensor_mounts import initial_state_from_recorded_base, sensor_mounts_from_dvl_anchor


def pose(xyz, rotation=None):
    value = np.eye(4)
    value[:3, 3] = xyz
    if rotation is not None:
        value[:3, :3] = rotation
    return value


@pytest.mark.parametrize("native_frd", [False, True])
def test_axis_label_change_does_not_rotate_the_body(native_frd):
    x180 = np.diag([1.0, -1.0, -1.0])
    anchor = pose([-0.00488, 0.0, -0.15861], x180)
    dvl = pose([-0.00488, 0.0, -0.03910], x180 if native_frd else np.eye(3))
    basis = pose([0, 0, 0], np.eye(3) if native_frd else x180)
    imu = pose([0.13135, 0.0, 0.08541])
    model_base, sensors = sensor_mounts_from_dvl_anchor(
        model_from_dvl_anchor=anchor, anchor_from_recorded_dvl=basis,
        recorded_base_from_dvl=dvl, recorded_base_from_sensors={"imu": imu, "dvl": dvl},
    )
    np.testing.assert_allclose(model_base[:3, :3], np.eye(3), atol=1e-12)
    np.testing.assert_allclose(sensors["dvl"][:3, 3], anchor[:3, 3], atol=1e-12)
    np.testing.assert_allclose(sensors["imu"][:3, 3], [0.13135, 0.0, -0.03410], atol=1e-12)
    np.testing.assert_allclose(np.linalg.inv(sensors["dvl"]) @ sensors["imu"], np.linalg.inv(dvl) @ imu, atol=1e-12)


def test_recorded_base_origin_and_sensor_levers_remain_explicit():
    anchor = pose([-0.00488, 0.0, -0.15861])
    dvl = pose([-0.03196, 0.0, -0.097])
    pressure = pose([-0.17364, -0.03034, 0.0536])
    model_base, sensors = sensor_mounts_from_dvl_anchor(
        model_from_dvl_anchor=anchor, anchor_from_recorded_dvl=np.eye(4),
        recorded_base_from_dvl=dvl, recorded_base_from_sensors={"pressure": pressure},
    )
    np.testing.assert_allclose(model_base[:3, 3], [0.02708, 0, -0.06161], atol=1e-12)
    np.testing.assert_allclose(sensors["pressure"][:3, 3], [-0.14656, -0.03034, -0.00801], atol=1e-12)
    np.testing.assert_allclose(np.linalg.inv(model_base) @ sensors["pressure"], pressure, atol=1e-12)


@pytest.mark.parametrize("invalid", [pose([0.01, 0, 0]), np.full((4, 4), np.nan), np.zeros((4, 4))])
def test_invalid_or_translated_basis_is_rejected(invalid):
    with pytest.raises(ValueError):
        sensor_mounts_from_dvl_anchor(
            model_from_dvl_anchor=np.eye(4), anchor_from_recorded_dvl=invalid,
            recorded_base_from_dvl=np.eye(4), recorded_base_from_sensors={},
        )


def test_rebased_initialization_preserves_sensor_world_pose_and_velocity():
    rng = np.random.default_rng(20261106)
    for _ in range(100):
        rotations = []
        for _ in range(2):
            q, _ = np.linalg.qr(rng.normal(size=(3, 3)))
            q[:, 0] *= np.linalg.det(q)
            rotations.append(q)
        world_base = pose(rng.normal(size=3), rotations[0])
        model_base = pose(rng.normal(size=3) * 0.1, rotations[1])
        linear, angular, sensor_position = rng.normal(size=(3, 3))
        world_model, model_velocity, model_angular = initial_state_from_recorded_base(
            world_from_recorded_base=world_base, model_from_recorded_base=model_base,
            linear_velocity_recorded_body_mps=linear,
            angular_velocity_recorded_body_radps=angular,
        )
        original_sensor = world_base @ np.r_[sensor_position, 1.0]
        model_sensor = model_base @ np.r_[sensor_position, 1.0]
        np.testing.assert_allclose(world_model @ model_sensor, original_sensor, atol=1e-12)
        original_velocity = world_base[:3, :3] @ (linear + np.cross(angular, sensor_position))
        rebased_velocity = world_model[:3, :3] @ (model_velocity + np.cross(model_angular, model_sensor[:3]))
        np.testing.assert_allclose(rebased_velocity, original_velocity, atol=1e-12)


def test_zero_velocity_assumption_stays_at_the_recorded_origin():
    model_base = pose([0.02708, 0.0, -0.06161])
    _, linear, angular = initial_state_from_recorded_base(
        world_from_recorded_base=np.eye(4), model_from_recorded_base=model_base,
        linear_velocity_recorded_body_mps=np.zeros(3),
        angular_velocity_recorded_body_radps=np.array([0.0, 0.5, 0.0]),
    )
    assert np.linalg.norm(linear) > 0.01
    np.testing.assert_allclose(linear + np.cross(angular, model_base[:3, 3]), 0.0, atol=1e-12)
