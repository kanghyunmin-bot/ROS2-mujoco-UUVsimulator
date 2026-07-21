#!/usr/bin/env python3
"""Regression for bounded GUI logs and incremental readiness scanning."""

from __future__ import annotations

import os
from pathlib import Path
import sys
import tempfile


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from gui.process_log_files import IncrementalLogMatcher, open_process_log  # noqa: E402


def main() -> int:
    with tempfile.TemporaryDirectory(prefix="uuv_log_contract_") as temp:
        root = Path(temp)
        for idx in range(12):
            path = root / f"worker_20260718_1200{idx:02d}.log"
            path.write_text(f"old {idx}\n", encoding="utf-8")
            os.utime(path, ns=(idx + 1, idx + 1))

        new_path, stream = open_process_log(root, "worker", keep=5)
        stream.write("booting\n")
        stream.flush()
        assert len(list(root.glob("worker_*.log"))) == 5

        # A same-second allocation must never truncate the active file.
        second_path, second_stream = open_process_log(root, "worker", keep=5)
        second_stream.close()
        assert second_path != new_path
        assert len(list(root.glob("worker_*.log"))) == 5

        marker = "strict compatibility transport READY"
        matcher = IncrementalLogMatcher(new_path, (marker,))
        assert not matcher.poll()
        first_read = matcher.characters_read
        assert first_read == len("booting\n")
        assert not matcher.poll()
        assert matcher.characters_read == first_read, "unchanged log must not be reread"
        split = len(marker) // 2
        stream.write(marker[:split])
        stream.flush()
        assert not matcher.poll()
        stream.write(marker[split:] + "\n")
        stream.flush()
        assert matcher.poll(), "marker split across appends must still match"
        stream.close()

    print("process_log_lifecycle=PASS keep=5 incremental=true")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
