#!/usr/bin/env python3
"""Separate exact hand/lower-shell triangles for selective visual coloring.

Offline only: uses the existing CAD component cache and installed trimesh/scipy.
Run once on the unsplit detail meshes; never changes collision assets.
"""

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
import trimesh

CURRENT = Path(__file__).resolve().parents[1]
ASSETS = CURRENT / "assets/urdf_full/meshes_split/body_2026_09"
COMPONENTS = (92, 93, 137, 138, 181, 182)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--component_dir", type=Path, required=True)
    args = parser.parse_args()
    output = ASSETS / "body_hand_lower_green.obj"
    if output.exists():
        raise RuntimeError("Color mesh already exists; refusing to split twice")
    manifest = json.loads((ASSETS / "manifest.json").read_text())
    rotation = np.array(manifest["rotation"])
    translation = np.array(manifest["translation_m"])
    targets = []
    counts = {}
    for component in COMPONENTS:
        mesh = trimesh.load_mesh(
            args.component_dir / f"component-{component}.ply", process=False
        )
        vertices = mesh.vertices * manifest["source_scale"] @ rotation.T + translation
        targets.append(vertices[mesh.faces])
        counts[component] = len(mesh.faces)
    triangles = np.concatenate(targets)
    tree = cKDTree(triangles.mean(axis=1))
    found = np.zeros(len(triangles), dtype=np.int32)
    edits = []
    selected = []
    original_count = 0
    for index in range(20):
        path = ASSETS / f"body_visual_detail_{index}.obj"
        lines = path.read_text().splitlines()
        vertex_lines = [line for line in lines if line.startswith("v ")]
        face_lines = [line for line in lines if line.startswith("f ")]
        vertices = np.array(
            [line.split()[1:4] for line in vertex_lines], dtype=np.float64
        )
        faces = np.array(
            [
                [int(word.split("/")[0]) - 1 for word in line.split()[1:]]
                for line in face_lines
            ]
        )
        original_count += len(faces)
        distance, target = tree.query(vertices[faces].mean(axis=1))
        mask = distance < 1e-7
        if not np.any(mask):
            continue
        candidate = vertices[faces[mask]]
        expected = triangles[target[mask]]
        differences = np.linalg.norm(
            candidate[:, :, None, :] - expected[:, None, :, :], axis=-1
        )
        if np.max(differences.min(axis=2)) > 1e-7:
            raise RuntimeError("Centroid match does not preserve the source triangle")
        np.add.at(found, target[mask], 1)
        selected.append(candidate)
        # Keep original vertex precision/order and every unselected face unchanged.
        keep = iter(~mask)
        edits.append(
            (
                path,
                "\n".join(
                    line for line in lines if not line.startswith("f ") or next(keep)
                )
                + "\n",
            )
        )
    if not np.all(found == 1):
        raise RuntimeError(
            f"Expected each target triangle once: {np.unique(found, return_counts=True)}"
        )
    selected = np.concatenate(selected)
    vertices, inverse = np.unique(selected.reshape(-1, 3), axis=0, return_inverse=True)
    faces = inverse.reshape(-1, 3)
    lines = [
        "# Exact original triangles: two hands and four shells forming two lower capsules."
    ]
    lines.extend("v " + " ".join(f"{value:.9f}" for value in row) for row in vertices)
    lines.extend("f " + " ".join(str(value + 1) for value in row) for row in faces)
    output.write_text("\n".join(lines) + "\n")
    for path, text in edits:
        path.write_text(text)
    result = {
        "component_ids": COMPONENTS,
        "component_triangles": counts,
        "selected_triangles": len(faces),
        "total_triangles_unchanged": original_count,
        "mesh": output.name,
        "source": "exact existing OBJ triangles; no decimation",
    }
    (ASSETS / "selective_colors.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
