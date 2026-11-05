"""Randomization moves the entire buoy assembly, not just its free float."""
import json
import numpy as np
import pytest
from test_demo_reset import fixture


def test_spawn_is_repeatable_and_preserves_attachment(tmp_path, monkeypatch):
    reset = fixture()
    request = tmp_path/'spawn-request.json'
    monkeypatch.setenv('UUV_DEMO_SPAWN_REQUEST', str(request))
    spec = dict(seed=42, x_m=[-1.5, 1.5], y_m=[-1., 1.])
    request.write_text(json.dumps(spec))
    original = reset.step.data.qpos.copy()
    original_constraint_error = np.max(np.abs(reset.step.data.efc_pos), initial=0.)
    reset.restore()
    first = reset.step.data.qpos.copy()
    assert not np.array_equal(first, original)
    buoy = next(b for b in reset.buoy_runtime.buoys if b.name == 'course_buoy_a_yellow_1')
    delta = first[buoy.free_qposadr:buoy.free_qposadr+2]-original[buoy.free_qposadr:buoy.free_qposadr+2]
    anchor = reset.step.model.body('course_buoy_a_yellow_1_magnet_base').id
    np.testing.assert_allclose(reset.step.model.body_pos[anchor,:2]-reset.model_arrays['body_pos'][anchor,:2], delta)
    assert reset.step.data.eq_active[buoy.eq_id]
    assert first[buoy.free_qposadr+2] == original[buoy.free_qposadr+2]
    assert np.isfinite(reset.step.data.efc_pos).all()
    assert np.max(np.abs(reset.step.data.efc_pos), initial=0.) <= original_constraint_error+1e-6
    assert np.linalg.norm(first[buoy.free_qposadr:buoy.free_qposadr+2]-original[:2]) >= 1.25
    applied = json.loads((tmp_path/'spawn-result.json').read_text())
    assert applied['seed'] == 42 and applied['target'] == buoy.name
    np.testing.assert_allclose(applied['xy_m'],first[buoy.free_qposadr:buoy.free_qposadr+2])
    reset.restore()
    np.testing.assert_array_equal(first, reset.step.data.qpos)
    for _ in range(50):
        reset.buoy_runtime.apply(reset.step.model.opt.timestep)
        reset.step.mujoco.mj_step(reset.step.model, reset.step.data)
    assert np.isfinite(reset.step.data.qpos).all() and not buoy.detached
    spec['seed'] = 43
    request.write_text(json.dumps(spec)); reset.restore()
    assert not np.array_equal(first, reset.step.data.qpos)
    monkeypatch.delenv('UUV_DEMO_SPAWN_REQUEST'); reset.restore()
    np.testing.assert_array_equal(original, reset.step.data.qpos)


def test_invalid_request_does_not_move_scene(tmp_path, monkeypatch):
    reset = fixture(); before = reset.step.data.qpos.copy()
    request = tmp_path/'spawn-request.json'
    request.write_text(json.dumps(dict(seed=42, x_m=[-100, 1], y_m=[-1, 1])))
    monkeypatch.setenv('UUV_DEMO_SPAWN_REQUEST', str(request))
    with pytest.raises(ValueError): reset.restore()
    np.testing.assert_array_equal(before, reset.step.data.qpos)
