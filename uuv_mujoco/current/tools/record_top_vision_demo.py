#!/usr/bin/env python3
"""Record a side-by-side MuJoCo top view and ROS vision preview."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2
import mujoco
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


class DemoRecorder(Node):
    def __init__(self, scene: Path) -> None:
        super().__init__("top_vision_demo_recorder")
        self.model = mujoco.MjModel.from_xml_path(str(scene))
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        self.renderer = mujoco.Renderer(self.model, height=360, width=640)
        self.camera = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(self.camera)
        self.camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.camera.lookat[:] = (0.0, 0.0, -0.55)
        self.camera.distance = 6.3
        self.camera.azimuth = 90.0
        self.camera.elevation = -89.0

        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        if body_id < 0 or self.model.body_jntnum[body_id] < 1:
            raise RuntimeError("base_link free joint not found")
        joint_id = int(self.model.body_jntadr[body_id])
        self.base_qpos_adr = int(self.model.jnt_qposadr[joint_id])
        self.vision_frame: np.ndarray | None = None
        self.pose_received = False

        self.create_subscription(
            PoseStamped,
            "/mujoco/ground_truth/pose",
            self._on_pose,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CompressedImage,
            "/vision/buoy/image_annotated/compressed",
            self._on_vision,
            qos_profile_sensor_data,
        )

    def _on_pose(self, msg: PoseStamped) -> None:
        q = self.base_qpos_adr
        pose = msg.pose
        self.data.qpos[q : q + 3] = (pose.position.x, pose.position.y, pose.position.z)
        self.data.qpos[q + 3 : q + 7] = (
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
        )
        mujoco.mj_forward(self.model, self.data)
        self.pose_received = True

    def _on_vision(self, msg: CompressedImage) -> None:
        encoded = np.frombuffer(msg.data, dtype=np.uint8)
        decoded = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if decoded is not None:
            self.vision_frame = cv2.resize(decoded, (640, 360), interpolation=cv2.INTER_AREA)

    def compose(self, elapsed: float) -> np.ndarray:
        self.renderer.update_scene(self.data, camera=self.camera)
        top_bgr = cv2.cvtColor(self.renderer.render(), cv2.COLOR_RGB2BGR)
        vision = self.vision_frame
        if vision is None:
            vision = np.zeros((360, 640, 3), dtype=np.uint8)
            cv2.putText(vision, "WAITING FOR VISION", (155, 185), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 80, 255), 2)
        canvas = np.zeros((408, 1280, 3), dtype=np.uint8)
        canvas[48:, :640] = top_bgr
        canvas[48:, 640:] = vision
        cv2.putText(canvas, "MuJoCo TOP VIEW", (190, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (255, 255, 255), 2)
        cv2.putText(canvas, "YOLO VISION", (865, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (255, 255, 255), 2)
        cv2.putText(canvas, f"{elapsed:05.1f}s", (1175, 31), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (160, 255, 160), 1)
        return canvas

    def close(self) -> None:
        self.renderer.close()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=18.0)
    parser.add_argument("--fps", type=float, default=10.0)
    args = parser.parse_args()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = DemoRecorder(args.scene.resolve())
    writer = cv2.VideoWriter(
        str(args.output),
        cv2.VideoWriter_fourcc(*"mp4v"),
        args.fps,
        (1280, 408),
    )
    if not writer.isOpened():
        raise RuntimeError(f"could not open video writer: {args.output}")
    start = time.monotonic()
    next_frame = start
    try:
        while rclpy.ok() and time.monotonic() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=0.01)
            now = time.monotonic()
            if now < next_frame:
                continue
            writer.write(node.compose(now - start))
            next_frame += 1.0 / args.fps
    finally:
        writer.release()
        node.close()
        node.destroy_node()
        rclpy.shutdown()
    print(f"recorded={args.output} duration={args.duration:.1f}s fps={args.fps:.1f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
