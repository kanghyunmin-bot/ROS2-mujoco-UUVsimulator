"""Union-find helper for STL connected-component labeling."""

from __future__ import annotations


class UnionFind:
    def __init__(self) -> None:
        self.parent: list[int] = []
        self.rank: list[int] = []

    def make(self) -> int:
        idx = len(self.parent)
        self.parent.append(idx)
        self.rank.append(0)
        return idx

    def find(self, idx: int) -> int:
        while self.parent[idx] != idx:
            self.parent[idx] = self.parent[self.parent[idx]]
            idx = self.parent[idx]
        return idx

    def union(self, a: int, b: int) -> None:
        ra = self.find(a)
        rb = self.find(b)
        if ra == rb:
            return
        if self.rank[ra] < self.rank[rb]:
            ra, rb = rb, ra
        self.parent[rb] = ra
        if self.rank[ra] == self.rank[rb]:
            self.rank[ra] += 1


__all__ = ["UnionFind"]
