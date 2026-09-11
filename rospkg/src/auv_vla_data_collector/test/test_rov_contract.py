import numpy as np
import pytest

from kmu26_auv_vla_data_collector.contract import (
    RcCommandTracker,
    body_velocity,
    sample_is_fresh,
    validate_sample_times,
)


def test_delayed_frame_is_not_fresh_when_received_now():
    assert not sample_is_fresh(1.0, 10.0, 10.0, 0.25)
    assert not sample_is_fresh(11.0, 10.0, 10.0, 0.25)
    assert sample_is_fresh(0.0, 0.0, 0.0, 0.25)


def test_physical_dvl_axes_are_converted_once():
    np.testing.assert_equal(
        body_velocity([1, 2, 3], "dvl_link", "dvl_link", "FRD"), [1, -2, -3]
    )
    np.testing.assert_equal(
        body_velocity([1, 2, 3], "base_link", "base_link", "FLU"), [1, 2, 3]
    )
    with pytest.raises(ValueError):
        body_velocity([1, 2, 3], "unknown", "dvl_link", "FRD")


def test_release_invalidates_previous_command_and_requires_explicit_reacquisition():
    tracker = RcCommandTracker()
    tracker.update([1800] * 18, 1.0)
    assert tracker.fresh(1.1, 0.5)
    release = [65535] * 18
    release[4] = 0
    tracker.update(release, 1.1)
    assert not tracker.fresh(1.1, 0.5)
    assert tracker.command[0] == 0.0
    tracker.update([65535] * 18, 1.2)
    assert not tracker.fresh(1.2, 0.5)
    tracker.update([1500] * 18, 1.3)
    assert tracker.fresh(1.3, 0.5)


def test_nochange_does_not_extend_command_lifetime():
    tracker = RcCommandTracker()
    tracker.update([1800] * 18, 1.0)
    tracker.update([65535] * 18, 2.0)
    assert not tracker.fresh(2.0, 0.5)
    assert not tracker.fresh(0.0, 0.5)


@pytest.mark.parametrize(
    "times", [[0, 0.1, 0.5], [1, 1.1, 0], [0, 0], [0, float("nan")]]
)
def test_export_refuses_discontinuous_time(times):
    with pytest.raises(ValueError):
        validate_sample_times(times, 10)


def test_normal_timer_jitter_is_accepted():
    validate_sample_times([1, 1.11, 1.2, 1.3], 10)


def test_pwm_outside_declared_span_is_not_clipped_into_valid_label():
    tracker = RcCommandTracker(span=300)
    tracker.update([1900] * 18, 1.0)
    assert not tracker.fresh(1.1, 0.5)
