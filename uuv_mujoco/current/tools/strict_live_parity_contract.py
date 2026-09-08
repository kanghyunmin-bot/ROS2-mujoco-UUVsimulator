"""Pure parsers and verdicts for the strict live-parity probe."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
import math
import re
from typing import Any


EXPECTED_FIRMWARE = "ArduSub V4.1.2"
REQUIRED_TOPICS = (
    "/clock",
    "/mavros/state",
    "/mavros/imu/data",
    "/odometry/filtered",
    "/dvl/data",
    "/dvl/position",
    "/dvl/twist",
)
PARAMETER_NAMES = (
    "AHRS_EKF_TYPE",
    "RC3_TRIM",
    "RC_OPTIONS",
    "ARMING_CHECK",
    "FS_GCS_ENABLE",
    "FS_PILOT_INPUT",
    "FS_PILOT_TIMEOUT",
)
RC_OPTIONS_ARMING_CHECK_MASK = 1 << 5


FIRMWARE_PATTERN = re.compile(
    r"\bArduSub V\d+(?:\.\d+)+(?:-[A-Za-z0-9_.-]+)?"
)
PARAMETER_LINE_PATTERN = re.compile(
    rf"^\s*({'|'.join(map(re.escape, PARAMETER_NAMES))})\s*(?:=|\s)\s*"
    r"(-?(?:\d+(?:\.\d*)?|\.\d+))\s*$",
    re.MULTILINE,
)


# ROS graph endpoint metadata exposes node names, not package names. The A50
# driver's package contract (auv_dvl_a50) appears on the graph as dvl_a50_node.
PUBLISHER_OWNERSHIP_RULES: dict[str, dict[str, Any]] = {
    "/dvl/data": {
        "contract_owner": "auv_dvl_a50",
        "node_basenames": ("dvl_a50_node",),
    },
    "/dvl/position": {
        "contract_owner": "auv_dvl_a50",
        "node_basenames": ("dvl_a50_node",),
    },
    "/dvl/twist": {
        "contract_owner": "auv_dvl_a50 pipeline adapter",
        "node_basenames": ("dvl_to_twist_bridge",),
    },
    "/odometry/filtered": {
        "contract_owner": "robot_localization",
        "node_basenames": ("ekf_filter_node",),
    },
    "/mavros/imu/data": {
        "contract_owner": "MAVROS",
        # MAVROS 2.14 gives each plugin its own ROS node.  The IMU plugin is
        # therefore the exact /mavros/imu owner, not the UAS container node.
        "node_fqns": ("/mavros/imu",),
    },
}


ROS_PARAMETER_NOT_SET = 0
ROS_PARAMETER_INTEGER = 2
ROS_PARAMETER_DOUBLE = 3


def decode_ros_numeric_parameters(
    names: Sequence[str],
    parameter_values: Sequence[Any],
) -> tuple[dict[str, float], dict[str, str]]:
    """Decode numeric values returned by the ROS 2 parameter API."""

    values: dict[str, float] = {}
    errors: dict[str, str] = {}
    for index, requested_name in enumerate(names):
        name = str(requested_name)
        if index >= len(parameter_values):
            errors[name] = "response omitted parameter value"
            continue
        parameter_value = parameter_values[index]
        value_type = int(getattr(parameter_value, "type", ROS_PARAMETER_NOT_SET))
        if value_type == ROS_PARAMETER_INTEGER:
            values[name] = float(getattr(parameter_value, "integer_value", 0))
        elif value_type == ROS_PARAMETER_DOUBLE:
            values[name] = float(getattr(parameter_value, "double_value", 0.0))
        elif value_type == ROS_PARAMETER_NOT_SET:
            errors[name] = "parameter not present in live MAVROS cache"
        else:
            errors[name] = f"unsupported ROS parameter type {value_type}"
    return values, errors


def parse_parameter_text(text: str) -> dict[str, float]:
    """Parse the strict-contract parameters from log or parm text."""

    values: dict[str, float] = {}
    for match in PARAMETER_LINE_PATTERN.finditer(text):
        values[match.group(1)] = float(match.group(2))
    return values


def parse_sitl_log(text: str) -> dict[str, Any]:
    """Extract selected firmware, startup parameters, and binary from a SITL log."""

    ap_versions = [
        match.group(1)
        for match in re.finditer(
            rf"^AP:\s*({FIRMWARE_PATTERN.pattern})",
            text,
            re.MULTILINE,
        )
    ]
    all_versions = FIRMWARE_PATTERN.findall(text)
    unique_versions = list(dict.fromkeys(all_versions))
    firmware = ap_versions[-1] if ap_versions else (all_versions[-1] if all_versions else None)

    binary_matches = re.findall(
        r"RiTW:\s*Starting ArduSub\s*:\s*[\"']?([^\s\"']*/ardusub)\b",
        text,
    )
    if not binary_matches:
        binary_matches = re.findall(
            r"[\"']([^\"']*/build/sitl/bin/ardusub)[\"']",
            text,
        )
    snapshot_counts = [
        int(match)
        for match in re.findall(
            r"Saved\s+(\d+)\s+parameters\s+to\s+mav\.parm",
            text,
        )
    ]
    return {
        "firmware": firmware,
        "firmware_mentions": unique_versions,
        "parameters": parse_parameter_text(text),
        "ardusub_binary": binary_matches[-1] if binary_matches else None,
        "parameter_snapshot_saved": bool(snapshot_counts),
        "parameter_snapshot_count": snapshot_counts[-1] if snapshot_counts else None,
    }


def assess_firmware(actual: str | None) -> dict[str, Any]:
    """Require the physical-stack parity firmware version."""

    return {
        "passed": actual == EXPECTED_FIRMWARE,
        "expected": EXPECTED_FIRMWARE,
        "actual": actual,
    }


def _integer_parameter(value: float | int | None) -> int | None:
    if value is None:
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(parsed) or abs(parsed - round(parsed)) > 1.0e-6:
        return None
    return int(round(parsed))


def assess_parameters(
    values: Mapping[str, float | int],
    *,
    source: str,
    source_fresh: bool | None = None,
) -> dict[str, Any]:
    """Check estimator, RC, arming, and loss-of-control contracts."""

    ahrs = _integer_parameter(values.get("AHRS_EKF_TYPE"))
    trim = _integer_parameter(values.get("RC3_TRIM"))
    options = _integer_parameter(values.get("RC_OPTIONS"))
    arming_check = _integer_parameter(values.get("ARMING_CHECK"))
    gcs_failsafe = _integer_parameter(values.get("FS_GCS_ENABLE"))
    pilot_failsafe = _integer_parameter(values.get("FS_PILOT_INPUT"))
    pilot_timeout = _integer_parameter(values.get("FS_PILOT_TIMEOUT"))
    checks = {
        "AHRS_EKF_TYPE": {
            "passed": ahrs == 3,
            "expected": 3,
            "actual": ahrs,
        },
        "RC3_TRIM": {
            "passed": trim == 1100,
            "expected": 1100,
            "actual": trim,
        },
        "RC_OPTIONS_bit5": {
            "passed": options is not None
            and bool(options & RC_OPTIONS_ARMING_CHECK_MASK),
            "expected_mask": RC_OPTIONS_ARMING_CHECK_MASK,
            "actual": options,
            "bit5_set": None
            if options is None
            else bool(options & RC_OPTIONS_ARMING_CHECK_MASK),
        },
        "ARMING_CHECK": {
            "passed": arming_check == 194,
            "expected": 194,
            "actual": arming_check,
        },
        "FS_GCS_ENABLE": {
            "passed": gcs_failsafe == 2,
            "expected": 2,
            "actual": gcs_failsafe,
        },
        "FS_PILOT_INPUT": {
            "passed": pilot_failsafe == 2,
            "expected": 2,
            "actual": pilot_failsafe,
        },
        "FS_PILOT_TIMEOUT": {
            "passed": pilot_timeout == 3,
            "expected": 3,
            "actual": pilot_timeout,
        },
    }
    return {
        "passed": all(item["passed"] for item in checks.values()),
        "source": source,
        "source_fresh": source_fresh,
        "checks": checks,
    }


def assess_clock(clock_nanoseconds: Sequence[int]) -> dict[str, Any]:
    """Require a ROS clock that advances without moving backwards."""

    samples = [int(value) for value in clock_nanoseconds]
    regressions = [
        {"index": index, "previous_ns": previous, "current_ns": current}
        for index, (previous, current) in enumerate(
            zip(samples, samples[1:]),
            start=1,
        )
        if current < previous
    ]
    duplicate_count = sum(
        current == previous for previous, current in zip(samples, samples[1:])
    )
    advances = len(samples) >= 2 and samples[-1] > samples[0]
    return {
        "passed": len(samples) >= 2 and not regressions and advances,
        "sample_count": len(samples),
        "first_ns": samples[0] if samples else None,
        "last_ns": samples[-1] if samples else None,
        "advance_ns": samples[-1] - samples[0] if len(samples) >= 2 else None,
        "duplicate_count": duplicate_count,
        "regressions": regressions,
    }


def assess_topic_counts(counts: Mapping[str, int]) -> dict[str, Any]:
    """Require at least one message from every strict surface topic."""

    checks = {
        topic: {
            "passed": int(counts.get(topic, 0)) > 0,
            "message_count": int(counts.get(topic, 0)),
        }
        for topic in REQUIRED_TOPICS
    }
    return {
        "passed": all(item["passed"] for item in checks.values()),
        "checks": checks,
    }


def publisher_basename(node_name: str) -> str:
    """Return the basename from a fully qualified ROS node name."""

    return str(node_name).strip().rstrip("/").rsplit("/", 1)[-1]


def assess_publisher_ownership(
    publishers_by_topic: Mapping[str, Sequence[str]],
) -> dict[str, Any]:
    """Require one canonical publisher family and reject duplicate owners."""

    checks: dict[str, Any] = {}
    for topic, rule in PUBLISHER_OWNERSHIP_RULES.items():
        publishers = sorted({str(name) for name in publishers_by_topic.get(topic, ())})
        expected_fqns = set(rule.get("node_fqns", ()))
        expected_basenames = set(rule.get("node_basenames", ()))
        if expected_fqns:
            matched = [name for name in publishers if name in expected_fqns]
            unexpected = [name for name in publishers if name not in expected_fqns]
        else:
            matched = [
                name
                for name in publishers
                if publisher_basename(name) in expected_basenames
            ]
            unexpected = [
                name
                for name in publishers
                if publisher_basename(name) not in expected_basenames
            ]
        checks[topic] = {
            "passed": len(publishers) == 1 and len(matched) == 1 and not unexpected,
            "contract_owner": rule["contract_owner"],
            "expected_node_basenames": sorted(expected_basenames),
            "expected_node_fqns": sorted(expected_fqns),
            "publishers": publishers,
            "matched": matched,
            "unexpected": unexpected,
            "duplicate_owner_count": max(0, len(publishers) - 1),
        }
    return {
        "passed": all(item["passed"] for item in checks.values()),
        "checks": checks,
    }


ARMING_LOW_PATTERNS = (
    re.compile(r"\barming[-_ ]*low\b", re.IGNORECASE),
    re.compile(r"\barm(?:ing)?\b.*\brc3\s*[=:]\s*1100\b", re.IGNORECASE),
    re.compile(r"\brc3\s*[=:]\s*1100\b.*\barm(?:ing)?\b", re.IGNORECASE),
)
ARMED_PATTERNS = (
    re.compile(r"\barmed\s*(?:->|=|:)\s*true\b", re.IGNORECASE),
    re.compile(r"\bstate\s+armed\s*=\s*true\b", re.IGNORECASE),
)
NEUTRAL_PATTERNS = (
    re.compile(r"\bneutral\b", re.IGNORECASE),
    re.compile(r"\brc3\s*[=:]\s*1500\b", re.IGNORECASE),
)


def _first_matching_event(
    events: Sequence[str],
    patterns: Sequence[re.Pattern[str]],
    *,
    start: int,
) -> tuple[int | None, str | None]:
    for index in range(max(0, start), len(events)):
        text = str(events[index])
        if any(pattern.search(text) for pattern in patterns):
            return index, text
    return None, None


def assess_arm_event_sequence(events_chronological: Sequence[str]) -> dict[str, Any]:
    """Check the firmware-specific low-RC arm transition and neutral handoff."""

    events = [str(event) for event in events_chronological]
    low_index, low_event = _first_matching_event(events, ARMING_LOW_PATTERNS, start=0)
    armed_index, armed_event = _first_matching_event(
        events,
        ARMED_PATTERNS,
        start=0 if low_index is None else low_index + 1,
    )
    neutral_index, neutral_event = _first_matching_event(
        events,
        NEUTRAL_PATTERNS,
        start=0 if armed_index is None else armed_index + 1,
    )
    passed = (
        low_index is not None
        and armed_index is not None
        and neutral_index is not None
        and low_index < armed_index < neutral_index
    )
    return {
        "passed": passed,
        "event_count": len(events),
        "arming_low": {"index": low_index, "event": low_event},
        "armed": {"index": armed_index, "event": armed_event},
        "neutral": {"index": neutral_index, "event": neutral_event},
        "events": events,
    }


def assess_neutral_heave(
    samples: Sequence[Mapping[str, float]],
    *,
    max_abs_vertical_speed_mps: float,
    max_vertical_drift_m: float,
) -> dict[str, Any]:
    """Reject sustained or runaway heave while zero pilot input is held."""

    velocities = [
        float(sample["vertical_speed_mps"])
        for sample in samples
        if "vertical_speed_mps" in sample
        and math.isfinite(float(sample["vertical_speed_mps"]))
    ]
    positions = [
        float(sample["position_z_m"])
        for sample in samples
        if "position_z_m" in sample
        and math.isfinite(float(sample["position_z_m"]))
    ]
    max_speed = max((abs(value) for value in velocities), default=None)
    drift = abs(positions[-1] - positions[0]) if len(positions) >= 2 else None
    span = max(positions) - min(positions) if len(positions) >= 2 else None
    enough_data = len(velocities) >= 2 and len(positions) >= 2
    speed_ok = max_speed is not None and max_speed <= max_abs_vertical_speed_mps
    drift_ok = drift is not None and drift <= max_vertical_drift_m
    return {
        "passed": enough_data and speed_ok and drift_ok,
        "sample_count": len(samples),
        "velocity_sample_count": len(velocities),
        "position_sample_count": len(positions),
        "max_abs_vertical_speed_mps": max_speed,
        "vertical_drift_m": drift,
        "vertical_span_m": span,
        "limits": {
            "max_abs_vertical_speed_mps": float(max_abs_vertical_speed_mps),
            "max_vertical_drift_m": float(max_vertical_drift_m),
        },
    }


__all__ = [
    "EXPECTED_FIRMWARE",
    "PARAMETER_NAMES",
    "PUBLISHER_OWNERSHIP_RULES",
    "RC_OPTIONS_ARMING_CHECK_MASK",
    "REQUIRED_TOPICS",
    "assess_arm_event_sequence",
    "assess_clock",
    "assess_firmware",
    "assess_neutral_heave",
    "assess_parameters",
    "assess_publisher_ownership",
    "assess_topic_counts",
    "parse_parameter_text",
    "parse_sitl_log",
    "publisher_basename",
]
