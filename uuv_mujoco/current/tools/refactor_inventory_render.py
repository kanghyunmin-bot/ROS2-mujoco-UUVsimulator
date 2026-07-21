"""Output renderers for the refactor inventory tool."""

from __future__ import annotations

from refactor_inventory_types import FileInventory


def render_markdown(items: list[FileInventory], limit: int) -> str:
    rows = sorted(items, key=lambda item: (item.complexity_score, item.loc), reverse=True)[:limit]
    out = [
        "# Refactor Inventory",
        "",
        "| File | Score | LOC | Branches | Functions | Classes | Kind | Largest symbol |",
        "| --- | ---: | ---: | ---: | ---: | ---: | --- | --- |",
    ]
    for item in rows:
        largest = item.largest_symbols[0] if item.largest_symbols else None
        if largest:
            symbol = f"{largest.kind} `{largest.name}` @ {largest.line} ({largest.loc} loc)"
        else:
            symbol = item.parse_error or ""
        kind = "data" if item.data_only else "code"
        out.append(
            f"| `{item.path}` | {item.complexity_score} | {item.loc} | {item.branches} | "
            f"{item.functions} | {item.classes} | {kind} | {symbol} |"
        )
    out.append("")
    return "\n".join(out)
