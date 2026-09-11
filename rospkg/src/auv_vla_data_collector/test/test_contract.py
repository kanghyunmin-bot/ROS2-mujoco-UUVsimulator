import numpy as np

from kmu26_auv_vla_data_collector.contract import (
    ACTION_NAMES,
    STATE_NAMES,
    build_state,
    update_normalized_rc_command,
)


def test_rc_override_is_converted_in_body_command_order():
    channels = [1500] * 18
    channels[4] = 1800
    channels[5] = 1200
    channels[2] = 1650
    channels[3] = 1350

    command, updated = update_normalized_rc_command(channels, np.zeros(4))

    np.testing.assert_allclose(command, [1.0, -1.0, 0.5, -0.5])
    np.testing.assert_array_equal(updated, np.ones(4))


def test_nochange_and_release_retain_previous_command():
    previous = np.asarray([0.1, 0.2, 0.3, 0.4], dtype=np.float32)
    channels = [1500] * 18
    channels[4] = 65535
    channels[5] = 0

    command, updated = update_normalized_rc_command(channels, previous)

    np.testing.assert_allclose(command, [0.1, 0.2, 0.0, 0.0])
    np.testing.assert_array_equal(updated, [0.0, 0.0, 1.0, 1.0])


def test_state_contract_has_expected_size_and_normalized_quaternion():
    state = build_state(
        previous_command=np.zeros(4),
        dvl_velocity=np.zeros(3),
        angular_velocity=np.zeros(3),
        linear_acceleration=np.zeros(3),
        attitude_wxyz=[2.0, 0.0, 0.0, 0.0],
        depth_m=1.2,
        altitude_m=0.8,
        validity=np.ones(4),
    )

    assert state.shape == (len(STATE_NAMES),)
    assert len(ACTION_NAMES) == 4
    np.testing.assert_allclose(state[13:17], [1.0, 0.0, 0.0, 0.0])
