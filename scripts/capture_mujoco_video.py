#!/usr/bin/env python3
"""Record MuJoCo stereo topic to short MP4 for quick visual inspection."""

from __future__ import annotations

import argparse
import time
from pathlib import Path
from typing import Optional

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class _VideoRecorder(Node):
    def __init__(
        self,
        topic: str,
        fps: float,
        output: Path,
        width: int,
        height: int,
        duration: float,
        keep: bool,
    ) -> None:
        super().__init__("mujoco_video_recorder")
        self.topic = topic
        self.fps = float(max(0.5, fps))
        self.min_dt = 1.0 / self.fps
        self.output = output
        self.target_width = int(width)
        self.target_height = int(height)
        self.duration = float(max(0.5, duration))
        self.keep = bool(keep)
        self.bridge = CvBridge()
        self.writer: Optional[cv2.VideoWriter] = None
        self.frame_period = 1.0 / self.fps
        self.last_write = 0.0
        self.start_wall = time.monotonic()
        self.frames = 0
        self.done = False

        self.create_subscription(Image, self.topic, self._on_image, 10)
        self.get_logger().info(
            f"recording {self.topic} -> {self.output} (fps={self.fps:.1f}, "
            ff"duration={self.duration:.1f}s, keep={self.keep})"
        )

    def _open_writer(self, image) -> None:
        h, w = image.shape[:2]
        out_w = self.target_width if self.target_width > 0 else w
        out_h = self.target_height if self.target_height > 0 else h
        if w != out_w or h != out_h:
            self.output.parent.mkdir(parents=True, exist_ok=True)
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        self.writer = cv2.VideoWriter(
            str(self.output),
            fourcc,
            self.fps,
            (out_w, out_h),
            isColor=True,
        )
        if not self.writer.isOpened():
            raise RuntimeError(f"failed to open video writer: {self.output}")
        if w != out_w or h != out_h:
            self.get_logger().info(f"resize output from {w}x{h} -> {out_w}x{out_h}")

    def _on_image(self, msg: Image) -> None:
        now = time.monotonic()
        if now - self.start_wall >= self.duration:
            self.done = True
            return

        if now - self.last_write < self.min_dt:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if self.writer is None:
            self._open_writer(frame)

        if self.writer is None:
            return

        if self.target_width > 0 and self.target_height > 0:
            frame = cv2.resize(
                frame,
                (self.target_width, self.target_height),
                interpolation=cv2.INTER_LINEAR,
            )
        self.writer.write(frame)
        self.last_write = now
        self.frames += 1

    def shutdown(self) -> None:
        if self.writer is not None:
            self.writer.release()
            self.writer = None

    def should_stop(self) -> bool:
        if self.done:
            return True
        if time.monotonic() - self.start_wall >= self.duration:
            self.done = True
            return True
        return False


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Capture a short MJPEG MP4 for MuJoCo stereo preview.")
    parser.add_argument("--topic", default="/stereo/left/image_raw", help="Image topic")
    parser.add_argument("--duration", type=float, default=12.0, help="Capture time in seconds")
    parser.add_argument("--fps", type=float, default=15.0, help="Capture FPS")
    parser.add_argument("--width", type=int, default=640, help="Output width")
    parser.add_argument("--height", type=int, default=360, help="Output height")
    parser.add_argument("--output", default="", help="Output MP4 path (default: /tmp/mujoco_preview_*.mp4)")
    parser.add_argument("--keep", action="store_true", help="Keep mp4 after finish")
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    if args.duration <= 0:
        raise SystemExit("duration must be greater than 0")
    if args.fps <= 0:
        raise SystemExit("fps must be greater than 0")

    output = Path(args.output) if args.output else Path("/tmp") / f"mujoco_preview_{int(time.time())}.mp4"

    rclpy.init(args=None)
    node = _VideoRecorder(
        topic=args.topic,
        fps=args.fps,
        output=output,
        width=args.width,
        height=args.height,
        duration=args.duration,
        keep=args.keep,
    )

    try:
        while rclpy.ok() and not node.should_stop():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.done = True
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if output.exists() and output.stat().st_size > 0:
        print(f"[capture] saved: {output}")
        if node.frames > 0:
            print(f"[capture] frames: {node.frames}")
        else:
            print("[capture] no frames received (check /stereo/* topics and --ros2-images)")
    else:
        print("[capture] no output generated")

    if not args.keep and output.exists():
        output.unlink()
        print(f"[capture] auto deleted: {output}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
