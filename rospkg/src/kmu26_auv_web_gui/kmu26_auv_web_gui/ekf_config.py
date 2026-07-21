import re
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory


MATRIX_SIZE = 15
STATE_NAMES = [
    "x",
    "y",
    "z",
    "roll",
    "pitch",
    "yaw",
    "vx",
    "vy",
    "vz",
    "vroll",
    "vpitch",
    "vyaw",
    "ax",
    "ay",
    "az",
]


def read_process_noise_covariance() -> dict:
    path = _installed_config_path()
    values = _load_covariance(path)
    return {
        "path": str(path),
        "state_names": STATE_NAMES,
        "size": MATRIX_SIZE,
        "values": values,
        "diagonal": [values[index * MATRIX_SIZE + index] for index in range(MATRIX_SIZE)],
        "mirrors": [str(item) for item in _config_paths()],
    }


def write_process_noise_covariance(values: list[float]) -> dict:
    clean_values = _validate_values(values)
    written = []
    for path in _config_paths():
        if path.exists():
            _replace_covariance(path, clean_values)
            written.append(str(path))
    if not written:
        path = _installed_config_path()
        _replace_covariance(path, clean_values)
        written.append(str(path))
    return {
        "state_names": STATE_NAMES,
        "size": MATRIX_SIZE,
        "values": clean_values,
        "diagonal": [
            clean_values[index * MATRIX_SIZE + index] for index in range(MATRIX_SIZE)
        ],
        "written": written,
    }


def _installed_config_path() -> Path:
    package_share = Path(get_package_share_directory("hit25_auv_ros2"))
    return package_share / "config" / "auv_ekf.yaml"


def _config_paths() -> list[Path]:
    installed = _installed_config_path()
    paths = [installed]
    workspace = _workspace_from_install_path(installed)
    if workspace is not None:
        source = workspace / "src" / "kmu26_auv" / "config" / "auv_ekf.yaml"
        if source != installed:
            paths.append(source)
    return paths


def _workspace_from_install_path(path: Path) -> Path | None:
    for parent in path.parents:
        if parent.name == "install":
            return parent.parent
    return None


def _load_covariance(path: Path) -> list[float]:
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    try:
        values = data["ekf_filter_node"]["ros__parameters"]["process_noise_covariance"]
    except (KeyError, TypeError) as exc:
        raise ValueError("process_noise_covariance not found in EKF YAML") from exc
    return _validate_values(values)


def _validate_values(values: list[float]) -> list[float]:
    if not isinstance(values, list):
        raise ValueError("process_noise_covariance must be a list")
    expected = MATRIX_SIZE * MATRIX_SIZE
    if len(values) != expected:
        raise ValueError(f"process_noise_covariance must contain {expected} values")
    clean_values = []
    for value in values:
        try:
            clean_values.append(float(value))
        except (TypeError, ValueError) as exc:
            raise ValueError("process_noise_covariance values must be numeric") from exc
    return clean_values


def _replace_covariance(path: Path, values: list[float]) -> None:
    text = path.read_text(encoding="utf-8")
    pattern = re.compile(r"^(\s*process_noise_covariance:\s*)\[[\s\S]*?\]", re.MULTILINE)
    match = pattern.search(text)
    if match is None:
        raise ValueError(f"process_noise_covariance not found in {path}")

    replacement = _format_covariance(match.group(1), values)
    updated = text[: match.start()] + replacement + text[match.end() :]
    path.write_text(updated, encoding="utf-8")

    # Validate the written file while preserving the rest of the YAML format.
    _load_covariance(path)


def _format_covariance(prefix: str, values: list[float]) -> str:
    continuation = " " * len(prefix)
    rows = []
    for row_index in range(MATRIX_SIZE):
        start = row_index * MATRIX_SIZE
        row = ", ".join(_format_number(value) for value in values[start : start + MATRIX_SIZE])
        if row_index == 0:
            rows.append(f"{prefix}[{row},")
        elif row_index == MATRIX_SIZE - 1:
            rows.append(f"{continuation}{row}]")
        else:
            rows.append(f"{continuation}{row},")
    return "\n".join(rows)


def _format_number(value: float) -> str:
    if value == 0.0:
        return "0.0"
    return f"{value:.12g}"
