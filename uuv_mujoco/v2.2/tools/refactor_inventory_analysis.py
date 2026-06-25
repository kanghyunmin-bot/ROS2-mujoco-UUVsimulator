"""AST analysis helpers for the refactor inventory tool."""

from __future__ import annotations

import ast
from pathlib import Path

try:
    from .refactor_inventory_paths import iter_python_files, normalized_rel
    from .refactor_inventory_scoring import compute_complexity_score
    from .refactor_inventory_symbols import collect_symbol_metrics, largest_symbols
    from .refactor_inventory_types import FileInventory
except ImportError:
    from refactor_inventory_paths import iter_python_files, normalized_rel
    from refactor_inventory_scoring import compute_complexity_score
    from refactor_inventory_symbols import collect_symbol_metrics, largest_symbols
    from refactor_inventory_types import FileInventory


def syntax_error_inventory(path: str, loc: int, exc: SyntaxError) -> FileInventory:
    return FileInventory(
        path=path,
        loc=loc,
        branches=0,
        functions=0,
        classes=0,
        complexity_score=compute_complexity_score(
            loc=loc,
            branches=0,
            functions=0,
            classes=0,
            largest_symbol_loc=0,
            parse_error=True,
        ),
        data_only=False,
        largest_symbols=[],
        parse_error=f"{exc.msg} at line {exc.lineno}",
    )


def analyze_python_file(path: Path, root: Path) -> FileInventory:
    text = path.read_text(errors="ignore")
    lines = text.splitlines()
    rel = normalized_rel(path, root)
    try:
        tree = ast.parse(text)
    except SyntaxError as exc:
        return syntax_error_inventory(rel, len(lines), exc)

    branch_count, function_count, class_count, symbols = collect_symbol_metrics(tree)
    largest = largest_symbols(symbols)
    largest_symbol_loc = largest[0].loc if largest else 0
    data_only = branch_count == 0 and function_count == 0 and class_count == 0
    return FileInventory(
        path=rel,
        loc=len(lines),
        branches=branch_count,
        functions=function_count,
        classes=class_count,
        complexity_score=compute_complexity_score(
            loc=len(lines),
            branches=branch_count,
            functions=function_count,
            classes=class_count,
            largest_symbol_loc=largest_symbol_loc,
        ),
        data_only=data_only,
        largest_symbols=largest,
    )


__all__ = ["analyze_python_file", "iter_python_files", "syntax_error_inventory"]
