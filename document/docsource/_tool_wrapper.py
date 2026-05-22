from __future__ import annotations

import sys
from pathlib import Path
from typing import Any


def run_tool(target_rel: str, namespace: dict[str, Any]) -> None:
    """Execute a relocated docsource tool while preserving legacy __file__ paths."""
    wrapper_path = Path(namespace["__file__"]).resolve()
    docsource_dir = wrapper_path.parent
    target_path = docsource_dir / target_rel
    if str(docsource_dir) not in sys.path:
        sys.path.insert(0, str(docsource_dir))
    namespace["__file__"] = str(wrapper_path)
    namespace["__package__"] = None
    namespace["__cached__"] = None
    code = compile(target_path.read_text(encoding="utf-8"), str(target_path), "exec")
    exec(code, namespace)
