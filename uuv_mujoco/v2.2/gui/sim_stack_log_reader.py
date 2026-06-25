"""Tail a simulator stack log until its process exits."""

from __future__ import annotations

from collections.abc import Callable

from .runtime import subprocess, time


LogLineHandler = Callable[[str, str], str]


def follow_process_log(
    proc: subprocess.Popen[str],
    log_path,
    handle_line: LogLineHandler,
    *,
    poll_interval_s: float = 0.1,
) -> tuple[int, str]:
    last_line = ""
    try:
        with log_path.open("r", encoding="utf-8", errors="replace") as log_stream:
            while True:
                raw_line = log_stream.readline()
                if raw_line:
                    last_line = handle_line(raw_line, last_line)
                    continue
                if proc.poll() is not None:
                    break
                time.sleep(poll_interval_s)
            for raw_line in log_stream:
                line = raw_line.strip()
                if line:
                    last_line = line
        rc = proc.returncode if proc.returncode is not None else proc.wait()
    except Exception as exc:
        return -1, f"reader failed: {exc}"

    return rc, last_line


__all__ = ["LogLineHandler", "follow_process_log"]
