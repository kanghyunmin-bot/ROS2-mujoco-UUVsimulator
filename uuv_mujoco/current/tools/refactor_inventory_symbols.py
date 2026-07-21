"""AST symbol counting for the refactor inventory tool."""

from __future__ import annotations

import ast

try:
    from .refactor_inventory_types import BRANCH_TYPES, SymbolInfo
except ImportError:
    from refactor_inventory_types import BRANCH_TYPES, SymbolInfo


def symbol_loc(node: ast.AST) -> int:
    start = getattr(node, "lineno", 0)
    end = getattr(node, "end_lineno", start)
    return int(end) - int(start) + 1


def collect_symbol_metrics(tree: ast.AST) -> tuple[int, int, int, list[SymbolInfo]]:
    symbols: list[SymbolInfo] = []
    function_count = 0
    class_count = 0
    branch_count = 0
    for node in ast.walk(tree):
        if isinstance(node, BRANCH_TYPES):
            branch_count += 1
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            function_count += 1
            symbols.append(SymbolInfo("func", node.name, node.lineno, symbol_loc(node)))
        elif isinstance(node, ast.ClassDef):
            class_count += 1
            symbols.append(SymbolInfo("class", node.name, node.lineno, symbol_loc(node)))
    return branch_count, function_count, class_count, symbols


def largest_symbols(symbols: list[SymbolInfo], limit: int = 8) -> list[SymbolInfo]:
    return sorted(symbols, key=lambda item: item.loc, reverse=True)[:limit]


__all__ = ["collect_symbol_metrics", "largest_symbols", "symbol_loc"]
