#!/usr/bin/env python3
"""Offline regression for bounded MuJoCo camera snapshot reuse."""

from __future__ import annotations

import threading
from pathlib import Path
import sys
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from bridge import ros2_stereo_image as camera  # noqa: E402


def main() -> int:
    allocated: list[object] = []
    copied: list[tuple[object, object]] = []

    original_data = camera.mujoco.MjData
    original_copy = camera.mujoco.mj_copyData
    original_start = camera._start_async_camera_worker

    def make_data(_model: object) -> object:
        value = object()
        allocated.append(value)
        return value

    def copy_data(target: object, _model: object, source: object) -> None:
        copied.append((target, source))

    bridge = SimpleNamespace(
        model=object(),
        _stereo_image_async_lock=threading.Lock(),
        _stereo_image_async_event=threading.Event(),
        _stereo_image_async_pending={},
        _stereo_image_async_free={},
        _stereo_image_async_latest={},
        _stereo_image_warned=set(),
        node=None,
    )
    try:
        camera.mujoco.MjData = make_data
        camera.mujoco.mj_copyData = copy_data
        camera._start_async_camera_worker = lambda _bridge: None

        source_a = object()
        source_b = object()
        assert camera._submit_async_camera_render(bridge, "stereo_left", source_a) is None
        assert len(allocated) == 1
        assert len(copied) == 1

        # A full pending slot drops another request before allocating/copying.
        assert camera._submit_async_camera_render(bridge, "stereo_left", source_b) is None
        assert len(allocated) == 1
        assert len(copied) == 1

        snapshot = bridge._stereo_image_async_pending.pop("stereo_left")
        camera._recycle_async_camera_snapshot(bridge, "stereo_left", snapshot)
        assert bridge._stereo_image_async_free["stereo_left"] == [snapshot]

        camera._submit_async_camera_render(bridge, "stereo_left", source_b)
        assert len(allocated) == 1, "recycled MjData must be reused"
        assert copied[-1] == (snapshot, source_b)

        # Pool size is bounded even if cleanup returns redundant buffers.
        bridge._stereo_image_async_pending.pop("stereo_left")
        camera._recycle_async_camera_snapshot(bridge, "stereo_left", object())
        camera._recycle_async_camera_snapshot(bridge, "stereo_left", object())
        camera._recycle_async_camera_snapshot(bridge, "stereo_left", object())
        assert len(bridge._stereo_image_async_free["stereo_left"]) == 2
    finally:
        camera.mujoco.MjData = original_data
        camera.mujoco.mj_copyData = original_copy
        camera._start_async_camera_worker = original_start

    print("async_camera_snapshot_reuse=PASS allocations=1 pool_max=2")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
