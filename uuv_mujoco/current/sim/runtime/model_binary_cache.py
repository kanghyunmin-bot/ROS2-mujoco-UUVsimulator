"""Dependency-checked MuJoCo binary model cache.

Large visual STL assets make MuJoCo's XML compiler build temporary convex-hull
and BVH workspaces.  Loading the compiled ``.mjb`` on later launches preserves
the exact model while avoiding that multi-gigabyte transient allocation.
"""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import re
from typing import Any, Callable
import xml.etree.ElementTree as ET


_CACHE_SCHEMA = 1
_CHUNK_BYTES = 1024 * 1024
_SAFE_NAME = re.compile(r"[^A-Za-z0-9_.-]+")


def load_model_with_binary_cache(
    *,
    mujoco_module: Any,
    scene: str | Path,
    enabled: bool,
    cache_dir: str | Path | None = None,
    log: Callable[[str], None] = print,
) -> Any:
    """Load *scene* from a valid MJB cache, otherwise compile and refresh it."""

    scene_path = Path(scene).expanduser().resolve()
    if not enabled:
        return mujoco_module.MjModel.from_xml_path(str(scene_path))

    try:
        fingerprint = scene_dependency_fingerprint(scene_path)
        paths = model_cache_paths(scene_path, cache_dir=cache_dir)
    except Exception as exc:
        log(f"[runtime] model cache fingerprint unavailable; compiling XML: {exc}")
        return mujoco_module.MjModel.from_xml_path(str(scene_path))

    version = str(getattr(mujoco_module, "__version__", "unknown"))
    if _cache_metadata_matches(paths.metadata, fingerprint=fingerprint, mujoco_version=version):
        try:
            model = mujoco_module.MjModel.from_binary_path(str(paths.binary))
        except Exception as exc:
            log(f"[runtime] cached MuJoCo model rejected; recompiling XML: {exc}")
        else:
            log(
                "[runtime] MuJoCo binary cache hit: "
                f"{paths.binary} fingerprint={fingerprint[:12]}"
            )
            return model

    model = mujoco_module.MjModel.from_xml_path(str(scene_path))
    try:
        _write_model_cache(
            mujoco_module=mujoco_module,
            model=model,
            scene=scene_path,
            paths=paths,
            fingerprint=fingerprint,
            mujoco_version=version,
        )
    except Exception as exc:
        # Cache creation is an optimization.  A read-only install or interrupted
        # write must never prevent the XML model from running.
        log(f"[runtime] MuJoCo binary cache write skipped: {exc}")
    else:
        log(
            "[runtime] MuJoCo binary cache refreshed: "
            f"{paths.binary} fingerprint={fingerprint[:12]}"
        )
    return model


class ModelCachePaths(tuple):
    """Tiny immutable path pair without another dataclass in the load path."""

    __slots__ = ()

    def __new__(cls, binary: Path, metadata: Path):
        return super().__new__(cls, (binary, metadata))

    @property
    def binary(self) -> Path:
        return self[0]

    @property
    def metadata(self) -> Path:
        return self[1]


def model_cache_paths(scene: Path, *, cache_dir: str | Path | None = None) -> ModelCachePaths:
    if cache_dir is None:
        root = Path(__file__).resolve().parents[2]
        directory = root / "generated" / "model_cache"
    else:
        directory = Path(cache_dir).expanduser().resolve()
    safe_stem = _SAFE_NAME.sub("_", scene.stem).strip("._") or "scene"
    return ModelCachePaths(
        directory / f"{safe_stem}.mjb",
        directory / f"{safe_stem}.mjb.json",
    )


def scene_dependency_fingerprint(scene: str | Path) -> str:
    """Hash the MJCF plus every recursively referenced file dependency."""

    scene_path = Path(scene).expanduser().resolve()
    dependencies = _scene_dependencies(scene_path)
    digest = hashlib.sha256()
    digest.update(b"uuv-mujoco-model-cache-v1\0")
    for dependency in dependencies:
        # Use a scene-relative label so a prebuilt distribution cache remains
        # valid when the whole install tree is moved to another machine.
        label = os.path.relpath(dependency, start=scene_path.parent)
        digest.update(label.encode("utf-8", errors="surrogateescape"))
        digest.update(b"\0")
        with dependency.open("rb") as handle:
            while True:
                chunk = handle.read(_CHUNK_BYTES)
                if not chunk:
                    break
                digest.update(chunk)
        digest.update(b"\0")
    return digest.hexdigest()


def _scene_dependencies(scene: Path) -> tuple[Path, ...]:
    found: set[Path] = set()
    visited_xml: set[Path] = set()

    def visit_xml(xml_path: Path, inherited_asset_dir: Path | None = None, inherited_mesh_dir: Path | None = None) -> None:
        xml_path = xml_path.resolve()
        if xml_path in visited_xml:
            return
        visited_xml.add(xml_path)
        found.add(xml_path)
        root = ET.parse(xml_path).getroot()
        compiler = root.find("compiler")
        asset_dir = inherited_asset_dir or xml_path.parent
        mesh_dir = inherited_mesh_dir or asset_dir
        if compiler is not None:
            asset_text = str(compiler.get("assetdir", "")).strip()
            mesh_text = str(compiler.get("meshdir", "")).strip()
            if asset_text:
                asset_dir = (xml_path.parent / asset_text).resolve()
            if mesh_text:
                mesh_dir = (xml_path.parent / mesh_text).resolve()
            elif asset_text:
                mesh_dir = asset_dir

        for element in root.iter():
            file_text = str(element.get("file", "")).strip()
            if not file_text:
                continue
            if element.tag == "include":
                include_path = (xml_path.parent / file_text).resolve()
                visit_xml(include_path, asset_dir, mesh_dir)
                continue
            base = mesh_dir if element.tag == "mesh" else asset_dir
            found.add((base / file_text).resolve())

    visit_xml(scene.resolve())
    missing = [path for path in found if not path.is_file()]
    if missing:
        raise FileNotFoundError("missing MJCF dependencies: " + ", ".join(str(path) for path in missing[:4]))
    return tuple(sorted(found, key=lambda path: str(path)))


def _cache_metadata_matches(metadata_path: Path, *, fingerprint: str, mujoco_version: str) -> bool:
    binary_path = metadata_path.with_suffix("")
    if not binary_path.is_file() or not metadata_path.is_file():
        return False
    try:
        payload = json.loads(metadata_path.read_text(encoding="utf-8"))
    except (OSError, ValueError, TypeError):
        return False
    return bool(
        isinstance(payload, dict)
        and int(payload.get("schema", -1)) == _CACHE_SCHEMA
        and str(payload.get("fingerprint", "")) == fingerprint
        and str(payload.get("mujoco_version", "")) == mujoco_version
    )


def _write_model_cache(
    *,
    mujoco_module: Any,
    model: Any,
    scene: Path,
    paths: ModelCachePaths,
    fingerprint: str,
    mujoco_version: str,
) -> None:
    paths.binary.parent.mkdir(parents=True, exist_ok=True)
    token = f"{os.getpid()}.{id(model):x}"
    binary_tmp = paths.binary.with_name(paths.binary.name + f".{token}.tmp")
    metadata_tmp = paths.metadata.with_name(paths.metadata.name + f".{token}.tmp")
    try:
        mujoco_module.mj_saveModel(model, str(binary_tmp), None)
        payload = {
            "schema": _CACHE_SCHEMA,
            "fingerprint": fingerprint,
            "mujoco_version": mujoco_version,
            "scene_name": scene.name,
        }
        metadata_tmp.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        os.replace(binary_tmp, paths.binary)
        os.replace(metadata_tmp, paths.metadata)
    finally:
        for temporary in (binary_tmp, metadata_tmp):
            try:
                temporary.unlink()
            except FileNotFoundError:
                pass


__all__ = [
    "ModelCachePaths",
    "load_model_with_binary_cache",
    "model_cache_paths",
    "scene_dependency_fingerprint",
]
