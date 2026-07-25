#!/usr/bin/env python3
"""Verify dependency invalidation and exact MJB model parity."""

from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile

import mujoco
import numpy as np


CURRENT_DIR = Path(__file__).resolve().parents[1]
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))

from sim.runtime.model_binary_cache import (  # noqa: E402
    load_model_with_binary_cache,
    model_cache_paths,
    scene_dependency_fingerprint,
)


class _CountingModelFactory:
    xml_loads = 0
    binary_loads = 0

    @classmethod
    def from_xml_path(cls, path: str):
        cls.xml_loads += 1
        return mujoco.MjModel.from_xml_path(path)

    @classmethod
    def from_binary_path(cls, path: str):
        cls.binary_loads += 1
        return mujoco.MjModel.from_binary_path(path)


class _CountingMujoco:
    __version__ = mujoco.__version__
    MjModel = _CountingModelFactory
    mj_saveModel = staticmethod(mujoco.mj_saveModel)


def _assert_model_parity(a, b) -> None:
    scalar_fields = ("nq", "nv", "nu", "nbody", "ngeom", "nsite", "neq", "nmesh")
    for field in scalar_fields:
        if int(getattr(a, field)) != int(getattr(b, field)):
            raise AssertionError(f"MJB parity mismatch for {field}")
    for field in (
        "body_mass",
        "body_inertia",
        "body_pos",
        "geom_type",
        "geom_size",
        "geom_pos",
        "geom_quat",
        "geom_contype",
        "geom_conaffinity",
        "eq_type",
        "eq_data",
    ):
        if not np.array_equal(np.asarray(getattr(a, field)), np.asarray(getattr(b, field))):
            raise AssertionError(f"MJB parity mismatch for {field}")


def main() -> int:
    _CountingModelFactory.xml_loads = 0
    _CountingModelFactory.binary_loads = 0
    with tempfile.TemporaryDirectory(prefix="model_cache_contract_", dir="/tmp") as temp_text:
        temp = Path(temp_text)
        assets = temp / "assets"
        assets.mkdir()
        include = temp / "body.xml"
        include.write_text(
            '<mujoco><worldbody><body name="cached_body"><geom type="sphere" size="0.1"/></body></worldbody></mujoco>\n',
            encoding="utf-8",
        )
        scene = temp / "scene.xml"
        scene.write_text(
            '<mujoco model="cache_test"><include file="body.xml"/></mujoco>\n',
            encoding="utf-8",
        )
        cache_dir = temp / "generated"
        first_fingerprint = scene_dependency_fingerprint(scene)
        first = load_model_with_binary_cache(
            mujoco_module=_CountingMujoco,
            scene=scene,
            enabled=True,
            cache_dir=cache_dir,
            log=lambda _message: None,
        )
        paths = model_cache_paths(scene, cache_dir=cache_dir)
        if not paths.binary.is_file() or not paths.metadata.is_file():
            raise AssertionError("first XML load did not create MJB cache")
        second = load_model_with_binary_cache(
            mujoco_module=_CountingMujoco,
            scene=scene,
            enabled=True,
            cache_dir=cache_dir,
            log=lambda _message: None,
        )
        if (_CountingModelFactory.xml_loads, _CountingModelFactory.binary_loads) != (1, 1):
            raise AssertionError(
                "valid cache did not use binary load exactly once: "
                f"xml={_CountingModelFactory.xml_loads} binary={_CountingModelFactory.binary_loads}"
            )
        _assert_model_parity(first, second)

        include.write_text(
            '<mujoco><worldbody><body name="cached_body"><geom type="sphere" size="0.2"/></body></worldbody></mujoco>\n',
            encoding="utf-8",
        )
        second_fingerprint = scene_dependency_fingerprint(scene)
        if second_fingerprint == first_fingerprint:
            raise AssertionError("included MJCF edit did not invalidate dependency fingerprint")
        third = load_model_with_binary_cache(
            mujoco_module=_CountingMujoco,
            scene=scene,
            enabled=True,
            cache_dir=cache_dir,
            log=lambda _message: None,
        )
        if _CountingModelFactory.xml_loads != 2:
            raise AssertionError("stale dependency cache was not recompiled")
        if not np.isclose(float(third.geom_size[0, 0]), 0.2):
            raise AssertionError("recompiled model did not include dependency edit")
        metadata = json.loads(paths.metadata.read_text(encoding="utf-8"))
        if metadata.get("fingerprint") != second_fingerprint:
            raise AssertionError("cache metadata did not advance to new dependency fingerprint")

    print("model_binary_cache=PASS xml_loads=2 binary_loads=1 parity=exact invalidation=include")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
