"""Build offline CAD collision assets; requires trimesh, NumPy and CoACD.

Input is the connected-component PLY cache from the body import, in CAD mm.
Visual meshes are never modified. Collision meshes use robot coordinates [m].
"""

import argparse
import json
from pathlib import Path

import numpy as np
import trimesh

# Concave exterior parts: two hands, four lower shells, and enclosure brackets.
DECOMPOSED_COMPONENTS = (92, 93, 137, 138, 181, 182, 350, 351, 352, 353, 421, 425)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--component_dir", type=Path, required=True)
    parser.add_argument("--decomposition_dir", type=Path, required=True)
    parser.add_argument("--output_dir", type=Path, required=True)
    parser.add_argument("--rebuild_decomposition", action="store_true")
    args = parser.parse_args()
    source = args.component_dir
    registration = json.loads((source / "registration.json").read_text())
    rows = json.loads((source / "components.json").read_text())
    visual = json.loads((args.output_dir.parent / "manifest.json").read_text())
    retained = set(visual["filter"]["kept"])
    args.output_dir.mkdir(parents=True, exist_ok=True)
    entries = []
    for row in rows:
        index = row["id"]
        if index not in retained:
            continue
        if index in DECOMPOSED_COMPONENTS:
            paths = sorted(args.decomposition_dir.glob(f"component_{index}_*.stl"))
            if not paths or args.rebuild_decomposition:
                import coacd

                coacd.set_log_level("error")
                mesh = trimesh.load_mesh(source / f"component-{index}.ply")
                mesh.vertices = (mesh.vertices * 0.001) @ np.array(
                    registration["rotation"]
                ).T
                mesh.vertices += registration["translation_m"]
                hand = index in (92, 93)
                parts = coacd.run_coacd(
                    coacd.Mesh(mesh.vertices, mesh.faces),
                    threshold=0.002 if hand else 0.005,
                    real_metric=True,
                    max_convex_hull=-1 if hand else 48,
                    preprocess_mode="auto",
                    preprocess_resolution=60,
                    resolution=2000,
                    mcts_iterations=80,
                    mcts_nodes=15,
                    merge=True,
                    decimate=True,
                    max_ch_vertex=64,
                    seed=1,
                )
                args.decomposition_dir.mkdir(parents=True, exist_ok=True)
                for old in paths:
                    old.unlink()
                paths = []
                for part, (vertices, faces) in enumerate(parts):
                    path = args.decomposition_dir / f"component_{index}_{part}.stl"
                    trimesh.Trimesh(vertices, faces, process=False).export(path)
                    paths.append(path)
            meshes = [trimesh.load_mesh(path, process=False) for path in paths]
        else:
            # Keep load-bearing features; tiny fasteners and CAD sheets remain visual.
            if max(row["size"]) < 20 or min(row["size"]) < 1 or row["faces"] < 4:
                continue
            mesh = trimesh.load_mesh(source / f"component-{index}.ply")
            mesh.vertices = (mesh.vertices * 0.001) @ np.array(
                registration["rotation"]
            ).T
            mesh.vertices += registration["translation_m"]
            if np.linalg.matrix_rank(mesh.vertices - mesh.vertices.mean(axis=0)) < 3:
                continue
            meshes = [mesh.convex_hull]
        for part, mesh in enumerate(meshes):
            name = f"cad_collision_{index}_{part}"
            mesh.export(args.output_dir / f"{name}.stl")
            entries.append(
                dict(
                    name=name,
                    component=index,
                    faces=len(mesh.faces),
                    hand=index in (92, 93),
                    bounds_m=mesh.bounds.tolist(),
                )
            )
    manifest = dict(
        source_sha256=visual["source_sha256"],
        method="Component convex hulls with CoACD for concave exterior parts",
        coacd=dict(
            version="1.0.14",
            hand_threshold_m=0.002,
            body_threshold_m=0.005,
            body_max_convex_hull=48,
            real_metric=True,
            preprocess_resolution=60,
            resolution=2000,
            mcts_iterations=80,
            mcts_nodes=15,
            max_ch_vertex=64,
            seed=1,
        ),
        decomposed_components=DECOMPOSED_COMPONENTS,
        meshes=entries,
    )
    (args.output_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n"
    )
    print(
        f"Built {len(entries)} collision meshes, {sum(e['faces'] for e in entries)} faces"
    )


if __name__ == "__main__":
    main()
