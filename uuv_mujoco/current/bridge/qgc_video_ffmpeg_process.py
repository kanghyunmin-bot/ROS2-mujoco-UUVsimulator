"""FFmpeg process lifecycle helpers."""

from __future__ import annotations

import subprocess


def open_ffmpeg_process(cmd: list[str]) -> subprocess.Popen:
    proc = subprocess.Popen(
        cmd,
        stdin=subprocess.PIPE,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    if proc.stdin is None:
        raise RuntimeError("failed to open ffmpeg stdin")
    return proc


def close_ffmpeg_process(proc: subprocess.Popen | None) -> None:
    if proc is None:
        return
    _close_stdin(proc)
    _terminate_process(proc)


def _close_stdin(proc: subprocess.Popen) -> None:
    try:
        if proc.stdin is not None:
            proc.stdin.close()
    except Exception:
        pass


def _terminate_process(proc: subprocess.Popen) -> None:
    try:
        proc.terminate()
        proc.wait(timeout=2.0)
    except Exception:
        try:
            proc.kill()
        except Exception:
            pass


__all__ = ["close_ffmpeg_process", "open_ffmpeg_process"]
