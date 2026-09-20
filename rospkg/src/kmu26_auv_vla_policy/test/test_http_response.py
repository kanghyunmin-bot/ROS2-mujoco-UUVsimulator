"""The real CAP-free U0 HTTP endpoint serializes (action, None) as a list."""
import numpy as np
import pytest
from kmu26_auv_vla_policy.kmu26_contract import motion_chunk


def test_cap_free_http_tuple_envelope():
    expected = np.full((16, 4), 0.2)
    np.testing.assert_array_equal(motion_chunk([{'action.motion': expected}, None]), expected)
    np.testing.assert_array_equal(motion_chunk({'action.motion': expected[None]}), expected)


@pytest.mark.parametrize('response', [[], [None, None], [{'action.motion': []}, {}], [1], 'invalid'])
def test_reject_unknown_or_cap_enabled_envelope(response):
    with pytest.raises(ValueError):
        motion_chunk(response)
