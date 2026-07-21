"""Types and constants for the refactor inventory tool."""

from __future__ import annotations

import ast
from dataclasses import dataclass


DEFAULT_EXCLUDES = {
    ".git",
    "__pycache__",
    "logs",
    "generated",
    "experiments/runs",
    "UUV-HAN/outputs",
}

BRANCH_TYPES = (
    ast.If,
    ast.For,
    ast.While,
    ast.Try,
    ast.With,
    ast.BoolOp,
)


@dataclass(frozen=True)
class SymbolInfo:
    kind: str
    name: str
    line: int
    loc: int


@dataclass(frozen=True)
class FileInventory:
    path: str
    loc: int
    branches: int
    functions: int
    classes: int
    complexity_score: int
    data_only: bool
    largest_symbols: list[SymbolInfo]
    parse_error: str | None = None
