"""Convert staged KMU26 episodes into the LeRobot layout consumed by U0."""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np

from .contract import ACTION_NAMES, STATE_NAMES, validate_sample_times

VIDEO_KEYS = ("observation.images.ego", "observation.images.buoy_release")


def _jsonl(path: Path, rows: Iterable[dict]) -> None:
    with path.open("w", encoding="utf-8") as stream:
        for row in rows:
            stream.write(json.dumps(row, ensure_ascii=False) + "\n")


def _statistics(array: np.ndarray) -> dict[str, list[float]]:
    return {
        "mean": np.mean(array, axis=0).tolist(),
        "std": np.std(array, axis=0).tolist(),
        "min": np.min(array, axis=0).tolist(),
        "max": np.max(array, axis=0).tolist(),
        "q01": np.quantile(array, 0.01, axis=0).tolist(),
        "q99": np.quantile(array, 0.99, axis=0).tolist(),
    }


def _encode_video(frame_dir: Path, output_path: Path, fps: float) -> tuple[int, int]:
    first_frame = cv2.imread(str(frame_dir / "frame_000000.jpg"), cv2.IMREAD_COLOR)
    if first_frame is None:
        raise ValueError(f"No readable first frame in {frame_dir}")
    height, width = first_frame.shape[:2]
    output_path.parent.mkdir(parents=True, exist_ok=True)
    command = [
        "ffmpeg",
        "-hide_banner",
        "-loglevel",
        "error",
        "-framerate",
        str(fps),
        "-i",
        str(frame_dir / "frame_%06d.jpg"),
        "-an",
        "-c:v",
        "libx264",
        "-preset",
        "medium",
        "-crf",
        "18",
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        str(output_path),
    ]
    subprocess.run(command, check=True)
    return height, width


def _modality() -> dict:
    state_dimensions = (4, 3, 3, 3, 4, 1, 1, 4)
    state_keys = (
        "prev_command",
        "dvl_velocity",
        "angular_velocity",
        "linear_acceleration",
        "attitude",
        "depth",
        "altitude",
        "validity",
    )
    start = 0
    state = {}
    for key, dimension in zip(state_keys, state_dimensions):
        state[key] = {"start": start, "end": start + dimension}
        start += dimension
    return {
        "state": state,
        "action": {"motion": {"start": 0, "end": 4}},
        "video": {
            "ego": {"original_key": VIDEO_KEYS[0]},
            "buoy_release": {"original_key": VIDEO_KEYS[1]},
        },
        "annotation": {"human.action.task_description": {"original_key": "task_index"}},
    }


def export_dataset(
    staging_root: Path, output_root: Path, requested_fps: float | None
) -> None:
    episode_dirs = sorted(
        path
        for path in staging_root.glob("episode_*")
        if (path / "manifest.json").is_file()
    )
    if not episode_dirs:
        raise ValueError(f"No complete episode directories found in {staging_root}")
    if output_root.exists() and any(output_root.iterdir()):
        raise FileExistsError(f"Output directory is not empty: {output_root}")

    manifests = [
        json.loads((path / "manifest.json").read_text()) for path in episode_dirs
    ]
    contracts = []
    for manifest in manifests:
        for key, expected in (
            ("state_names", STATE_NAMES),
            ("action_names", ACTION_NAMES),
        ):
            if key in manifest and tuple(manifest[key]) != expected:
                raise ValueError(f"Incompatible {key}; remap explicitly before export")
        provenance = manifest.get("provenance", {})
        contracts.append(
            json.dumps(
                {
                    "conventions": manifest.get("state_conventions"),
                    "channels": provenance.get("action_channels"),
                    "neutral": provenance.get("neutral_pwm"),
                    "span": provenance.get("pwm_span"),
                },
                sort_keys=True,
            )
        )
    if len(set(contracts)) != 1:
        raise ValueError(
            "Incompatible acquisition contracts; export separately and reconcile before mixing"
        )
    recorded_fps = {float(manifest["fps"]) for manifest in manifests}
    if requested_fps is None:
        if len(recorded_fps) != 1:
            raise ValueError(
                f"Episodes contain multiple recording rates: {recorded_fps}"
            )
        fps = recorded_fps.pop()
    else:
        fps = float(requested_fps)
    if not np.isfinite(fps) or fps <= 0.0:
        raise ValueError("FPS must be positive")

    if recorded_fps and any(not np.isclose(rate, fps) for rate in recorded_fps):
        raise ValueError("--fps cannot retime demonstrations; resample them explicitly")
    for episode_dir, manifest in zip(episode_dirs, manifests):
        with np.load(episode_dir / "samples.npz") as samples:
            count = int(manifest["frames"])
            if count < 1 or not str(manifest["task"]).strip():
                raise ValueError(f"Empty episode or instruction in {episode_dir}")
            for key, shape in {
                "observation_state": (count, 23),
                "action": (count, 4),
                "rc_pwm": (count, 4),
                "rc_update_mask": (count, 4),
                "ros_timestamp": (count,),
                "source_age": (count, 7),
                "source_timestamp": (count, 7),
            }.items():
                if samples[key].shape != shape:
                    raise ValueError(f"Invalid {key} shape in {episode_dir}")
            if not np.all(np.isfinite(samples["observation_state"])):
                raise ValueError(f"Nonfinite state in {episode_dir}")
            for key in ("source_timestamp", "receipt_timestamp"):
                if key in samples and (
                    samples[key].shape != (count, 7)
                    or not np.isfinite(samples[key]).all()
                ):
                    raise ValueError(f"Invalid {key} in {episode_dir}")
            validate_sample_times(samples["ros_timestamp"], fps)
            if not np.all(np.isfinite(samples["action"])) or np.any(
                np.abs(samples["action"]) > 1
            ):
                raise ValueError(f"Invalid normalized action in {episode_dir}")

        for camera in ("ego", "buoy_release"):
            frame_dir = episode_dir / "frames" / camera
            expected = [f"frame_{i:06d}.jpg" for i in range(count)]
            if sorted(p.name for p in frame_dir.glob("frame_*.jpg")) != expected:
                raise ValueError(f"Frames must be contiguous in {frame_dir}")
            shape = None
            for name in expected:
                image = cv2.imread(str(frame_dir / name))
                if image is None or (shape is not None and image.shape != shape):
                    raise ValueError(f"Unreadable or resized image: {frame_dir / name}")
                shape = image.shape
            if shape[0] % 2 or shape[1] % 2:
                raise ValueError("H264 yuv420p requires even image dimensions")

    output_root.mkdir(parents=True, exist_ok=True)
    metadata_dir = output_root / "meta"
    data_dir = output_root / "data" / "chunk-000"
    metadata_dir.mkdir(parents=True)
    data_dir.mkdir(parents=True)

    tasks = sorted({manifest["task"] for manifest in manifests})
    task_indices = {task: index for index, task in enumerate(tasks)}
    all_states = []
    all_actions = []
    episode_rows = []
    global_index = 0
    resolutions: dict[str, tuple[int, int]] = {}

    # pandas/pyarrow belong to the GR00T training environment, not the ROS runtime.
    try:
        import pandas as pd
    except ImportError as error:
        raise RuntimeError(
            "Export requires pandas and pyarrow; run it in the kmu26_auv_vla training environment"
        ) from error

    for output_episode_index, (episode_dir, manifest) in enumerate(
        zip(episode_dirs, manifests)
    ):
        with np.load(episode_dir / "samples.npz") as samples:
            state = samples["observation_state"].astype(np.float32)
            action = samples["action"].astype(np.float32)
            rc_pwm = samples["rc_pwm"].astype(np.int32)
            rc_update_mask = samples["rc_update_mask"].astype(np.float32)
            ros_timestamp = samples["ros_timestamp"].astype(np.float64)
            source_age = samples["source_age"].astype(np.float32)
            source_timestamp = samples["source_timestamp"].astype(np.float64)

        frame_count = int(manifest["frames"])
        if state.shape != (frame_count, len(STATE_NAMES)):
            raise ValueError(f"Invalid state shape in {episode_dir}: {state.shape}")
        if ros_timestamp.shape != (frame_count,):
            raise ValueError(f"Invalid timestamp shape in {episode_dir}")
        if action.shape != (frame_count, len(ACTION_NAMES)):
            raise ValueError(f"Invalid action shape in {episode_dir}: {action.shape}")
        if rc_pwm.shape != (frame_count, len(ACTION_NAMES)):
            raise ValueError(
                f"Invalid RC telemetry shape in {episode_dir}: {rc_pwm.shape}"
            )
        if rc_update_mask.shape != (frame_count, len(ACTION_NAMES)):
            raise ValueError(
                f"Invalid RC update-mask shape in {episode_dir}: {rc_update_mask.shape}"
            )

        for camera_name, video_key in zip(("ego", "buoy_release"), VIDEO_KEYS):
            frame_dir = episode_dir / "frames" / camera_name
            frame_files = sorted(frame_dir.glob("frame_*.jpg"))
            if len(frame_files) != frame_count:
                raise ValueError(
                    f"{episode_dir} {camera_name} has {len(frame_files)} frames; "
                    f"expected {frame_count}"
                )
            video_path = (
                output_root
                / "videos"
                / "chunk-000"
                / video_key
                / f"episode_{output_episode_index:06d}.mp4"
            )
            camera_resolution = _encode_video(frame_dir, video_path, fps)
            if video_key not in resolutions:
                resolutions[video_key] = camera_resolution
            elif resolutions[video_key] != camera_resolution:
                raise ValueError(
                    f"{video_key} resolution changed between episodes: "
                    f"{resolutions[video_key]} and {camera_resolution}"
                )

        task_index = task_indices[manifest["task"]]
        frame_indices = np.arange(frame_count, dtype=np.int64)
        data = {
            "observation.state": [row for row in state],
            "action": [row for row in action],
            "timestamp": frame_indices.astype(np.float64) / fps,
            "frame_index": frame_indices,
            "episode_index": np.full(frame_count, output_episode_index, dtype=np.int64),
            "index": np.arange(
                global_index, global_index + frame_count, dtype=np.int64
            ),
            "task_index": np.full(frame_count, task_index, dtype=np.int64),
            "telemetry.rc_pwm": [row for row in rc_pwm],
            "telemetry.rc_update_mask": [row for row in rc_update_mask],
            "telemetry.ros_timestamp": ros_timestamp,
            "telemetry.source_age": [row for row in source_age],
            "telemetry.source_timestamp": [row for row in source_timestamp],
            "episode_success": np.full(frame_count, bool(manifest["success"])),
            "next.done": np.asarray(
                [False] * (frame_count - 1) + [True], dtype=np.bool_
            ),
            "next.reward": np.zeros(frame_count, dtype=np.float32),
        }
        with np.load(episode_dir / "samples.npz") as samples:
            for key in ("receipt_timestamp", "rc_axis_timestamp"):
                if key in samples:
                    data[f"telemetry.{key}"] = [row for row in samples[key]]
        audit_dir = metadata_dir / "acquisition" / f"episode_{output_episode_index:06d}"
        audit_dir.mkdir(parents=True)
        for name in ("manifest.json", "vehicle_state.jsonl"):
            if (episode_dir / name).is_file():
                shutil.copyfile(episode_dir / name, audit_dir / name)
        if (episode_dir / "collector_source").is_dir():
            shutil.copytree(
                episode_dir / "collector_source", audit_dir / "collector_source"
            )
        dataframe = pd.DataFrame(data)
        dataframe.to_parquet(
            data_dir / f"episode_{output_episode_index:06d}.parquet",
            index=False,
            engine="pyarrow",
        )

        all_states.append(state)
        all_actions.append(action)
        episode_rows.append(
            {
                "episode_index": output_episode_index,
                "tasks": [manifest["task"]],
                "length": frame_count,
                "success": bool(manifest["success"]),
                "termination_reason": manifest.get("termination_reason", "unknown"),
                "provenance": manifest.get("provenance", {"data_source": "unknown"}),
                "source_episode": episode_dir.name,
            }
        )
        global_index += frame_count

    features = {}
    for video_key in VIDEO_KEYS:
        height, width = resolutions[video_key]
        features[video_key] = {
            "dtype": "video",
            "shape": [height, width, 3],
            "names": ["height", "width", "channel"],
            "video_info": {
                "video.fps": fps,
                "video.codec": "h264",
                "video.pix_fmt": "yuv420p",
                "video.is_depth_map": False,
                "has_audio": False,
            },
        }
    features.update(
        {
            "observation.state": {
                "dtype": "float32",
                "shape": [len(STATE_NAMES)],
                "names": list(STATE_NAMES),
            },
            "action": {
                "dtype": "float32",
                "shape": [len(ACTION_NAMES)],
                "names": list(ACTION_NAMES),
            },
            "timestamp": {"dtype": "float64", "shape": [1]},
            "frame_index": {"dtype": "int64", "shape": [1]},
            "episode_index": {"dtype": "int64", "shape": [1]},
            "index": {"dtype": "int64", "shape": [1]},
            "task_index": {"dtype": "int64", "shape": [1]},
        }
    )
    info = {
        "codebase_version": "v2.1",
        "robot_type": "kmu26_auv_real",
        "total_episodes": len(episode_rows),
        "total_frames": global_index,
        "total_tasks": len(tasks),
        "total_videos": len(episode_rows) * len(VIDEO_KEYS),
        "total_chunks": 1,
        "chunks_size": max(1000, len(episode_rows)),
        "fps": fps,
        "splits": {"train": f"0:{len(episode_rows)}"},
        "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
        "video_path": (
            "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4"
        ),
        "features": features,
    }
    state_array = np.concatenate(all_states, axis=0)
    action_array = np.concatenate(all_actions, axis=0)

    (metadata_dir / "info.json").write_text(json.dumps(info, indent=2) + "\n")
    (metadata_dir / "modality.json").write_text(
        json.dumps(_modality(), indent=2) + "\n"
    )
    (metadata_dir / "stats.json").write_text(
        json.dumps(
            {
                "observation.state": _statistics(state_array),
                "action": _statistics(action_array),
            },
            indent=2,
        )
        + "\n"
    )
    _jsonl(
        metadata_dir / "tasks.jsonl",
        ({"task_index": index, "task": task} for task, index in task_indices.items()),
    )
    _jsonl(metadata_dir / "episodes.jsonl", episode_rows)
    # Keep the complete acquisition contract, including interruption and calibration.
    _jsonl(metadata_dir / "source_manifests.jsonl", manifests)

    print(
        f"Exported {len(episode_rows)} episodes, {global_index} frames, "
        f"and {len(tasks)} tasks to {output_root}"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("staging_root", type=Path)
    parser.add_argument("output_root", type=Path)
    parser.add_argument("--fps", type=float, default=None)
    args = parser.parse_args()
    export_dataset(args.staging_root, args.output_root, args.fps)


if __name__ == "__main__":
    main()
