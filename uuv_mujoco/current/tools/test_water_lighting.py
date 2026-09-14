"""Rendering contracts: deterministic motion, occlusion and bounded light loss."""

from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from bridge.water_lighting import PoolWaterLighting, wave_field


def render(depth, *, position=(0, 0, 1), time_s=0):
    rgb = np.full((9, 9, 3), 120, dtype=np.uint8)
    return PoolWaterLighting(9, 9, 45).apply(
        rgb,
        np.full((9, 9), depth, dtype=np.float32),
        position=np.array(position),
        rotation=np.eye(3),
        time_s=time_s,
        surface_z=0,
        center_xy=np.zeros(2),
        half_size_xy=np.array([5, 2.5]),
    )


def test_foreground_above_water_is_unchanged():
    np.testing.assert_array_equal(render(0.5), 120)


def test_waves_are_bounded_and_replayable():
    x, y = np.meshgrid(np.linspace(-5, 5, 40), np.linspace(-2.5, 2.5, 20))
    a = wave_field(x, y, 1)
    for left, right in zip(a, wave_field(x, y, 1)):
        np.testing.assert_array_equal(left, right)
    assert np.max(np.abs(a[0])) <= 0.01601
    assert not np.array_equal(a[0], wave_field(x, y, 2)[0])
    np.testing.assert_array_equal(render(3, time_s=1), render(3, time_s=1))
    assert not np.array_equal(render(3, time_s=1), render(3, time_s=2))


def test_deep_objects_receive_less_light_and_ambient_remains():
    shallow = render(0.5, position=(0, 0, -0.1))
    deep = render(4, position=(0, 0, -0.1))
    assert np.mean(deep) < np.mean(shallow)
    assert np.min(deep) >= 83


def test_no_surface_effect_outside_pool():
    np.testing.assert_array_equal(render(3, position=(20, 20, 1)), 120)


def test_native_upload_is_bounded_and_does_not_block_physics():
    from contextlib import nullcontext
    from threading import Event
    import time
    from sim.runtime.water_surface_visual import WaterSurfaceVisual

    entered, release, finished = Event(), Event(), Event()
    calls = []

    class Viewer:
        def lock(self):
            return nullcontext()

        def is_running(self):
            return True

        def update_hfield(self, field_id):
            calls.append(field_id)
            entered.set()
            release.wait(1)
            finished.set()

    visual = WaterSurfaceVisual.__new__(WaterSurfaceVisual)
    visual.field_id = 0
    visual._upload_pending = False
    visual.update = lambda time_s: None
    viewer = Viewer()
    try:
        start = time.monotonic()
        visual.update_viewer(viewer, 1)
        assert time.monotonic() - start < 0.2
        assert entered.wait(0.5)
        visual.update_viewer(viewer, 2)
        assert calls == [0]
    finally:
        release.set()
        assert finished.wait(1)
