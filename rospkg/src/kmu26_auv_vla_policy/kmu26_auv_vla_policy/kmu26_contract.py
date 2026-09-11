"""KMU26 policy validation and bounded primary-channel RC conversion."""

import json
from pathlib import Path

import numpy as np

STATE_DIMS = {
    "prev_command": 4,
    "dvl_velocity": 3,
    "angular_velocity": 3,
    "linear_acceleration": 3,
    "attitude": 4,
    "depth": 1,
    "altitude": 1,
    "validity": 4,
}


def validate_checkpoint(path: str, embodiment: str = "new_embodiment") -> None:
    """Reject incompatible U0/GR00T checkpoints before allocating GPU memory."""
    metadata_file = Path(path).expanduser() / "experiment_cfg" / "metadata.json"
    if not metadata_file.is_file():
        raise ValueError(
            "KMU26 inference requires a local fine-tuned checkpoint with "
            "experiment_cfg/metadata.json; download/train it first"
        )
    metadata = json.loads(metadata_file.read_text()).get(embodiment, {})
    modalities = metadata.get("modalities", {})
    for name, dim in STATE_DIMS.items():
        if modalities.get("state", {}).get(name, {}).get("shape") != [dim]:
            raise ValueError(
                f"Checkpoint lacks KMU26 state.{name} dimension {dim}; fine-tune first"
            )
    if modalities.get("action", {}).get("motion", {}).get("shape") != [4]:
        raise ValueError("Checkpoint must predict action.motion [surge, sway, heave, yaw]")
    if not {"ego", "buoy_release"} <= modalities.get("video", {}).keys():
        raise ValueError("Checkpoint needs both KMU26 cameras")
    stats = metadata.get("statistics", {}).get("action", {}).get("motion", {})
    for name in ("min", "max"):
        value = np.asarray(stats.get(name, []), dtype=float)
        if value.shape != (4,) or not np.all(np.isfinite(value)):
            raise ValueError("Checkpoint lacks KMU26 action normalization statistics")


def motion_chunk(response: dict) -> np.ndarray:
    """Validate one unbatched or singleton-batched 16-step motion response."""
    chunk = np.asarray(response.get("action.motion", []), dtype=float)
    if chunk.shape == (1, 16, 4):
        chunk = chunk[0]
    if chunk.shape != (16, 4) or not np.all(np.isfinite(chunk)):
        raise ValueError("Expected finite action.motion with shape (16, 4) or (1, 16, 4)")
    if np.any(np.abs(chunk) > 1.00001):
        raise ValueError("Policy returned an action outside normalized [-1, 1]")
    return np.clip(chunk, -1, 1)


class CommandLimiter:
    """Bound normalized commands and slew rate while preserving RC channel semantics."""

    def __init__(self, limit=0.3, slew_per_second=0.5, neutral=1500, span=300):
        if not all(np.isfinite(x) for x in (limit, slew_per_second, neutral, span)):
            raise ValueError("Command limits must be finite")
        if not 0 < limit <= 1 or slew_per_second <= 0 or span <= 0:
            raise ValueError("Invalid command limits")
        if neutral - span < 1000 or neutral + span > 2000:
            raise ValueError("PWM range must remain inside 1000..2000")
        self.limit, self.slew = limit, slew_per_second
        self.neutral, self.span = int(neutral), int(span)
        self.previous = np.zeros(4)

    def reset(self):
        self.previous[:] = 0

    def apply(self, action, dt):
        action = np.asarray(action, dtype=float)
        if action.shape != (4,) or not np.all(np.isfinite(action)) or not 0 < dt <= 0.2:
            raise ValueError("Invalid command or control timing")
        target = np.clip(action, -self.limit, self.limit)
        self.previous += np.clip(target - self.previous, -self.slew * dt, self.slew * dt)
        channels = [65535] * 18
        for index, value in zip((4, 5, 2, 3), self.previous):
            channels[index] = int(round(self.neutral + self.span * value))
        return channels


def release_channels():
    """Release only the four controlled primary channels; leave all others alone."""
    channels = [65535] * 18
    for index in (4, 5, 2, 3):
        channels[index] = 0
    return channels
