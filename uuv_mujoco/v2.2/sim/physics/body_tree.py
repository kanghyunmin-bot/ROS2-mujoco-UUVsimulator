"""MuJoCo body-tree helpers used by runtime setup and audits."""

from __future__ import annotations


def build_body_children(model) -> list[list[int]]:
    """Return a child-body adjacency table for a MuJoCo model."""
    body_children: list[list[int]] = [[] for _ in range(model.nbody)]
    for body_idx in range(1, model.nbody):
        parent_idx = int(model.body_parentid[body_idx])
        if 0 <= parent_idx < model.nbody:
            body_children[parent_idx].append(int(body_idx))
    return body_children


def body_subtree_mass(model, root_body_id: int) -> float:
    """Return total mass of the given body and all descendants."""
    if not (0 <= int(root_body_id) < model.nbody):
        return 0.0
    body_children = build_body_children(model)
    total = 0.0
    stack = [int(root_body_id)]
    while stack:
        bid = stack.pop()
        total += float(model.body_mass[bid])
        stack.extend(body_children[bid])
    return total
