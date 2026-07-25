"""FFmpeg command construction for QGC RTP/H264 video streaming."""

from __future__ import annotations

from bridge.qgc_video_ffmpeg_probe import select_h264_encoder


def build_qgc_video_ffmpeg_cmd(
    *,
    host: str,
    port: int,
    width: int,
    height: int,
    fps: float,
    bitrate_kbps: int,
) -> list[str]:
    gop = max(2, int(round(float(fps))))
    encoder = select_h264_encoder()
    cmd = _base_rawvideo_input(width=width, height=height, fps=fps)
    if encoder == "h264_videotoolbox":
        cmd.extend(_videotoolbox_h264_args(encoder, gop=gop, bitrate_kbps=bitrate_kbps))
    else:
        cmd.extend(_x264_low_latency_args(encoder, gop=gop, bitrate_kbps=bitrate_kbps))
    cmd.extend(["-f", "rtp", f"rtp://{host}:{int(port)}?pkt_size=1200"])
    return cmd


def _base_rawvideo_input(*, width: int, height: int, fps: float) -> list[str]:
    return [
        "ffmpeg",
        "-loglevel",
        "warning",
        "-nostdin",
        "-f",
        "rawvideo",
        "-pix_fmt",
        "rgb24",
        "-video_size",
        f"{int(width)}x{int(height)}",
        "-framerate",
        str(int(round(float(fps)))),
        "-i",
        "-",
        "-an",
    ]


def _common_h264_output_args(*, encoder: str, gop: int, bitrate_kbps: int) -> list[str]:
    return [
        "-c:v",
        encoder,
        "-pix_fmt",
        "yuv420p",
        "-profile:v",
        "baseline",
        "-g",
        str(gop),
        "-bf",
        "0",
        "-b:v",
        f"{int(bitrate_kbps)}k",
        "-maxrate",
        f"{int(bitrate_kbps)}k",
        "-bufsize",
        f"{int(bitrate_kbps) * 2}k",
    ]


def _videotoolbox_h264_args(encoder: str, *, gop: int, bitrate_kbps: int) -> list[str]:
    return [
        "-c:v",
        encoder,
        "-realtime",
        "true",
        *_common_h264_output_args(encoder=encoder, gop=gop, bitrate_kbps=bitrate_kbps)[2:],
    ]


def _x264_low_latency_args(encoder: str, *, gop: int, bitrate_kbps: int) -> list[str]:
    return [
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
        f"{int(bitrate_kbps)}k",
        "-maxrate",
        f"{int(bitrate_kbps)}k",
        "-bufsize",
        f"{int(bitrate_kbps) * 2}k",
    ]


__all__ = ["build_qgc_video_ffmpeg_cmd"]
