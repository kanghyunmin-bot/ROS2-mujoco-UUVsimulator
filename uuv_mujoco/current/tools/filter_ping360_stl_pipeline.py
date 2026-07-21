"""High-level Ping360 STL filtering pipeline."""

from __future__ import annotations

from pathlib import Path

from filter_ping360_stl_components import classify_components, component_stats
from filter_ping360_stl_io import Triangle, compute_bbox, offset_triangle, read_binary_stl, write_binary_stl


def filter_ping360_stl(
    input_path: Path,
    output_path: Path,
    *,
    keep_min_z_mm: float,
    cable_min_length_mm: float,
    cable_max_thickness_mm: float,
    quantize: float,
) -> dict:
    _, triangles = read_binary_stl(input_path)
    components, root_for_triangle = component_stats(triangles, quantize)
    keep_roots = classify_components(
        components,
        keep_min_z_mm=keep_min_z_mm,
        cable_min_length_mm=cable_min_length_mm,
        cable_max_thickness_mm=cable_max_thickness_mm,
    )

    kept_triangles: list[Triangle] = [
        triangle for triangle, root in zip(triangles, root_for_triangle) if root in keep_roots
    ]
    if not kept_triangles:
        raise ValueError("no triangles selected; adjust filtering thresholds")

    bbox_min, bbox_max = compute_bbox(kept_triangles)
    bbox_center = tuple((bbox_min[axis] + bbox_max[axis]) * 0.5 for axis in range(3))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    write_binary_stl(output_path, kept_triangles, bbox_center)

    shifted_triangles = [offset_triangle(triangle, bbox_center) for triangle in kept_triangles]
    out_bbox_min, out_bbox_max = compute_bbox(shifted_triangles)
    return {
        "source": str(input_path),
        "output": str(output_path),
        "source_triangle_count": len(triangles),
        "kept_triangle_count": len(kept_triangles),
        "component_count": len(components),
        "source_units": "millimeter",
        "output_units": "millimeter",
        "source_offset_removed_mm": list(bbox_center),
        "output_bbox_min_mm": out_bbox_min,
        "output_bbox_max_mm": out_bbox_max,
        "output_size_mm": [out_bbox_max[axis] - out_bbox_min[axis] for axis in range(3)],
        "components": components,
    }


__all__ = ["filter_ping360_stl"]
