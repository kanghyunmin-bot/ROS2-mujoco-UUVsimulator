#!/usr/bin/env python3
"""Direct MuJoCo RGB frame -> QGroundControl RTP/H264 UDP streamer."""

from __future__ import annotations

import platform
import shutil
import subprocess
from functools import lru_cache
from typing import Optional


class QgcVideoStreamer:
    def __init__(
        self,
        host: str,
        port: int,
        width: int,
        height: int,
        fps: float,
        bitrate_kbps: int,
    ) -> None:
        self.host = str(host)
        self.port = int(port)
        self.width = int(width)
        self.height = int(height)
        self.fps = float(max(1.0, fps))
        self.bitrate_kbps = int(max(300, bitrate_kbps))
        self._proc: Optional[subprocess.Popen] = None

    @staticmethod
    def is_available() -> bool:
        return shutil.which("ffmpeg") is not None

    @staticmethod
    @lru_cache(maxsize=1)
    def _available_encoders() -> set[str]:
        if not QgcVideoStreamer.is_available():
            return set()
        result = subprocess.run(
            ["ffmpeg", "-hide_banner", "-encoders"],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            check=False,
        )
        encoders: set[str] = set()
        for line in result.stdout.splitlines():
            parts = line.split()
            if len(parts) >= 2 and parts[0].startswith("V"):
                encoders.add(parts[1])
        return encoders

    def _select_encoder(self) -> str:
        encoders = self._available_encoders()
        if platform.system() == "Darwin" and "h264_videotoolbox" in encoders:
            return "h264_videotoolbox"
        return "libx264"

    def _build_cmd(self) -> list[str]:
        gop = max(2, int(round(self.fps)))
        encoder = self._select_encoder()
        cmd = [
            "ffmpeg",
            "-loglevel",
            "warning",
            "-nostdin",
            "-f",
            "rawvideo",
            "-pix_fmt",
            "rgb24",
            "-video_size",
            f"{self.width}x{self.height}",
            "-framerate",
            str(int(round(self.fps))),
            "-i",
            "-",
            "-an",
        ]
        if encoder == "h264_videotoolbox":
            cmd.extend(
                [
                    "-c:v",
                    encoder,
                    "-realtime",
                    "true",
                    "-pix_fmt",
                    "yuv420p",
                    "-profile:v",
                    "baseline",
                    "-g",
                    str(gop),
                    "-bf",
                    "0",
                    "-b:v",
                    f"{self.bitrate_kbps}k",
                    "-maxrate",
                    f"{self.bitrate_kbps}k",
                    "-bufsize",
                    f"{self.bitrate_kbps * 2}k",
                ]
            )
        else:
            cmd.extend(
                [
                    "-c:v",
                    encoder,
                    "-preset",
                    "ultrafast",
                    "-tune",
                    "zerolatency",
                    "-pix_fmt",
                    "yuv420p",
                    "-profile:v",
                    "baseline",
                    "-g",
                    str(gop),
                    "-keyint_min",
                    str(gop),
                    "-bf",
                    "0",
                    "-b:v",
                    f"{self.bitrate_kbps}k",
                    "-maxrate",
                    f"{self.bitrate_kbps}k",
                    "-bufsize",
                    f"{self.bitrate_kbps * 2}k",
                ]
            )
        cmd.extend(
            [
                "-f",
                "rtp",
                f"rtp://{self.host}:{self.port}?pkt_size=1200",
            ]
        )
        return cmd

    def start(self) -> None:
        if self._proc is not None:
            return
        if not self.is_available():
            raise RuntimeError("ffmpeg not found")
        self._proc = subprocess.Popen(
            self._build_cmd(),
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        if self._proc.stdin is None:
            raise RuntimeError("failed to open ffmpeg stdin")

    def write(self, frame_rgb) -> None:
        if self._proc is None:
            self.start()
        if self._proc is None or self._proc.stdin is None:
            return
        if self._proc.poll() is not None:
            raise RuntimeError("ffmpeg exited")
        self._proc.stdin.write(frame_rgb.tobytes())

    def close(self) -> None:
        if self._proc is None:
            return
        try:
            if self._proc.stdin is not None:
                self._proc.stdin.close()
        except Exception:
            pass
        try:
            self._proc.terminate()
            self._proc.wait(timeout=2.0)
        except Exception:
            try:
                self._proc.kill()
            except Exception:
                pass
        self._proc = None
