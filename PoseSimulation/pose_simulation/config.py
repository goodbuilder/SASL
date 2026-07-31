"""Independent configuration contract for the PoseSimulation application.

This module deliberately has no ROS, Gazebo, PX4, or Simulation imports.  It
parses the PoseSimulation-owned ``KEY=VALUE`` files instead of sourcing shell
code, validates the application boundary, and returns an immutable resolved
snapshot suitable for a read-only diagnostic launcher.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import shlex
import sys
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Mapping, Sequence


ROOT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG_PATH = ROOT_DIR / "config" / "posesimulation.env"
APPLICATION_NAMESPACE = "/pose_simulation"
OUTPUT_TOPIC_PREFIX = "/pose_simulation/"
POSESIM_NAME_PATTERN = re.compile(r"^POSESIM_[A-Z0-9_]+$")


class ConfigError(ValueError):
    """Raised when a PoseSimulation configuration violates its contract."""


@dataclass(frozen=True)
class ConfigProvenance:
    """Source information for a resolved configuration."""

    config_path: str
    profile_path: str | None
    explicit_override_names: tuple[str, ...]


@dataclass(frozen=True)
class ResolvedConfig:
    """Validated, immutable configuration and its provenance."""

    values: Mapping[str, object]
    provenance: ConfigProvenance
    warnings: tuple[str, ...] = ()


# The schema is intentionally explicit.  Unknown POSESIM_* names are rejected
# so a typo cannot silently become a different runtime contract.
TYPE_BY_KEY: dict[str, str] = {
    # Application identity and ownership.
    "POSESIM_APPLICATION_NAME": "string",
    "POSESIM_MODE": "enum",
    "POSESIM_NODE_NAME": "string",
    "POSESIM_NAMESPACE": "string",
    "POSESIM_READ_ONLY": "bool",
    "POSESIM_SOURCE_APPLICATION": "string",
    "POSESIM_START_SOURCE_APPLICATION": "bool",
    "POSESIM_OWN_SOURCE_PROCESSES": "bool",
    # Source interface profile.
    "POSESIM_SOURCE_PROFILE_ID": "string",
    "POSESIM_GT_TOPIC": "topic",
    "POSESIM_GT_MESSAGE_TYPE": "string",
    "POSESIM_GT_REFERENCE_NAME": "string",
    "POSESIM_GT_FALLBACK_POLICY": "enum",
    "POSESIM_LEFT_CAMERA_POSE_TOPIC": "topic",
    "POSESIM_RIGHT_CAMERA_POSE_TOPIC": "topic",
    "POSESIM_CAMERA_POSE_MESSAGE_TYPE": "string",
    "POSESIM_CAMERA_POSE_SCHEMA": "string",
    "POSESIM_CAMERA_POSE_MODE": "enum",
    "POSESIM_LEFT_CAMERA_ENTITY_NAME": "string",
    "POSESIM_RIGHT_CAMERA_ENTITY_NAME": "string",
    "POSESIM_RTK_TOPIC": "topic",
    "POSESIM_RTK_MESSAGE_TYPE": "string",
    "POSESIM_ENABLE_FUSED_GLOBAL_DIAGNOSTIC": "bool",
    "POSESIM_FUSED_GLOBAL_TOPIC": "topic",
    "POSESIM_FUSED_GLOBAL_MESSAGE_TYPE": "string",
    # Coordinate contracts.
    "POSESIM_EXPECTED_WORLD_FRAME": "enum",
    "POSESIM_EXPECTED_CAMERA_FRAME": "enum",
    "POSESIM_EXPECTED_CAMERA_FORWARD_AXIS": "enum",
    "POSESIM_OUTPUT_FRAME": "enum",
    "POSESIM_OUTPUT_ORIGIN": "string",
    "POSESIM_OUTPUT_AXES": "enum",
    "POSESIM_STEREO_POLICY": "enum",
    "POSESIM_RIG_CENTER_SOURCE": "enum",
    "POSESIM_ROLL_REFERENCE": "enum",
    "POSESIM_POSE_CONTRACT_PATH": "string",
    "POSESIM_ALLOW_GT_FALLBACK": "bool",
    "POSESIM_ALLOW_PX4_AS_GROUND_TRUTH": "bool",
    "POSESIM_ALLOW_PLANNED_POSITION_AS_GROUND_TRUTH": "bool",
    # External georeference interface.
    "POSESIM_GEOREFERENCE_SOURCE": "enum",
    "POSESIM_SOURCE_WORLD_ORIENTATION": "enum",
    "POSESIM_SOURCE_SURFACE_MODEL": "enum",
    "POSESIM_SOURCE_ORIGIN_LATITUDE_DEG": "float",
    "POSESIM_SOURCE_ORIGIN_LONGITUDE_DEG": "float",
    "POSESIM_SOURCE_ORIGIN_ELEVATION_M": "float",
    "POSESIM_SOURCE_ORIGIN_ELEVATION_DATUM": "enum",
    "POSESIM_GAZEBO_ORIGIN_EAST_M": "float",
    "POSESIM_GAZEBO_ORIGIN_NORTH_M": "float",
    "POSESIM_GAZEBO_ORIGIN_UP_M": "float",
    # RTK altitude and comparison policy.
    "POSESIM_RTK_ALTITUDE_FIELD": "enum",
    "POSESIM_RTK_ALTITUDE_DATUM": "enum",
    "POSESIM_ENABLE_HORIZONTAL_RTK_COMPARISON": "bool",
    "POSESIM_ENABLE_VERTICAL_RTK_COMPARISON": "bool",
    "POSESIM_ALLOW_UNRESOLVED_VERTICAL_DATUM": "bool",
    # Physical target point.
    "POSESIM_GT_PHYSICAL_POINT": "string",
    "POSESIM_RTK_PHYSICAL_POINT": "string",
    "POSESIM_TARGET_POINT_ALIGNMENT_MODE": "enum",
    "POSESIM_LEVER_ARM_APPLIED": "bool",
    "POSESIM_LEVER_ARM_CONFIG_PATH": "optional_string",
    "POSESIM_REQUIRE_ALIGNMENT_PROVENANCE": "bool",
    # Evaluation and time alignment.
    "POSESIM_USE_SIM_TIME": "bool",
    "POSESIM_EVALUATION_HZ": "float",
    "POSESIM_EVALUATION_TIME_SOURCE": "enum",
    "POSESIM_TIME_ALIGNMENT_MODE": "enum",
    "POSESIM_MAX_GT_TIME_RESIDUAL_MS": "float",
    "POSESIM_MAX_LEFT_POSE_TIME_RESIDUAL_MS": "float",
    "POSESIM_MAX_RIGHT_POSE_TIME_RESIDUAL_MS": "float",
    "POSESIM_MAX_RTK_TIME_RESIDUAL_MS": "float",
    "POSESIM_MAX_INTERPOLATION_GAP_MS": "float",
    "POSESIM_MAX_GT_SOURCE_AGE_MS": "float",
    "POSESIM_STALE_AFTER_MS": "float",
    "POSESIM_ALLOW_FUTURE_EXTRAPOLATION": "bool",
    "POSESIM_REJECT_OUT_OF_ORDER": "bool",
    "POSESIM_REJECT_DUPLICATE_STAMPS": "bool",
    # RTK validity policy.
    "POSESIM_ALLOW_NON_FIXED_RTK_FOR_DIAGNOSTICS": "bool",
    "POSESIM_REQUIRE_RTK_FIXED_FOR_OPERATIONAL": "bool",
    "POSESIM_RTK_FIXED_STATUS_VALUES": "optional_string",
    "POSESIM_RTK_MAX_EPH_M": "optional_float",
    "POSESIM_RTK_MAX_EPV_M": "optional_float",
    "POSESIM_RTK_MAX_HORIZONTAL_JUMP_M": "optional_float",
    "POSESIM_RTK_MAX_VERTICAL_JUMP_M": "optional_float",
    "POSESIM_RTK_MAX_SOURCE_AGE_MS": "float",
    # PoseSimulation-owned output topics.
    "POSESIM_POSE_ERROR_TOPIC": "topic",
    "POSESIM_STEREO_GEOMETRY_TOPIC": "topic",
    "POSESIM_STATUS_TOPIC": "topic",
    "POSESIM_GT_IDEAL_POSE_TOPIC": "topic",
    "POSESIM_GT_IDEAL_RIG_POSE_TOPIC": "topic",
    "POSESIM_GT_IDEAL_LEFT_POSE_TOPIC": "topic",
    "POSESIM_GT_IDEAL_RIGHT_POSE_TOPIC": "topic",
    "POSESIM_RTK_IDEAL_POSE_TOPIC": "topic",
    "POSESIM_DIAGNOSTICS_TOPIC": "topic",
    "POSESIM_MARKER_TOPIC": "topic",
    # Recording policy.
    "POSESIM_ENABLE_RECORDING": "bool",
    "POSESIM_OUTPUT_DIR": "optional_string",
    "POSESIM_RECORD_FORMAT": "enum",
    "POSESIM_RECORD_PROVENANCE": "bool",
    "POSESIM_RECORD_INPUT_MESSAGES": "bool",
    # UI defaults.
    "POSESIM_START_RQT": "bool",
    "POSESIM_TREND_WINDOW_S": "float",
    "POSESIM_TREND_BUFFER_S": "float",
    "POSESIM_WARN_RELATIVE_YAW_ERROR_DEG": "optional_float",
    "POSESIM_FAIL_RELATIVE_YAW_ERROR_DEG": "optional_float",
    "POSESIM_WARN_OPTICAL_AXIS_ERROR_DEG": "optional_float",
    "POSESIM_FAIL_OPTICAL_AXIS_ERROR_DEG": "optional_float",
    "POSESIM_WARN_BASELINE_LENGTH_ERROR_M": "optional_float",
    "POSESIM_FAIL_BASELINE_LENGTH_ERROR_M": "optional_float",
    "POSESIM_WARN_BASELINE_DIRECTION_ERROR_DEG": "optional_float",
    "POSESIM_FAIL_BASELINE_DIRECTION_ERROR_DEG": "optional_float",
    "POSESIM_WARN_RTK_LOS_ERROR_DEG": "optional_float",
    "POSESIM_FAIL_RTK_LOS_ERROR_DEG": "optional_float",
    # RViz defaults.
    "POSESIM_START_RVIZ": "bool",
    "POSESIM_RVIZ_FIXED_FRAME": "enum",
    "POSESIM_RVIZ_RAY_MODE": "enum",
    "POSESIM_RVIZ_RAY_LENGTH_M": "float",
    "POSESIM_RVIZ_MARKER_LIFETIME_S": "float",
    "POSESIM_RVIZ_SHOW_ACTUAL": "bool",
    "POSESIM_RVIZ_SHOW_GT_IDEAL": "bool",
    "POSESIM_RVIZ_SHOW_RTK_IDEAL": "bool",
    "POSESIM_RVIZ_SHOW_GT_LOS": "bool",
    "POSESIM_RVIZ_SHOW_RTK_LOS": "bool",
    "POSESIM_RVIZ_SHOW_ERROR_ARCS": "bool",
    "POSESIM_RVIZ_SHOW_TEXT": "bool",
}

SELECTION_KEYS = {"POSESIM_CONFIG", "POSESIM_PROFILE_CONFIG"}

ENUM_VALUES: dict[str, set[str]] = {
    "POSESIM_MODE": {"diagnostic_only"},
    "POSESIM_GT_FALLBACK_POLICY": {"fail_closed"},
    "POSESIM_CAMERA_POSE_MODE": {"static_snapshot", "dynamic_stream"},
    "POSESIM_EXPECTED_WORLD_FRAME": {"gazebo_world_enu"},
    "POSESIM_EXPECTED_CAMERA_FRAME": {"camera_optical_rdf"},
    "POSESIM_EXPECTED_CAMERA_FORWARD_AXIS": {"positive_z"},
    "POSESIM_OUTPUT_FRAME": {"baseline_center_frd"},
    "POSESIM_OUTPUT_AXES": {"forward_right_down"},
    "POSESIM_STEREO_POLICY": {"parallel_rigid"},
    "POSESIM_RIG_CENTER_SOURCE": {"actual_optical_midpoint", "configured_rig_center_link"},
    "POSESIM_ROLL_REFERENCE": {"world_up"},
    "POSESIM_GEOREFERENCE_SOURCE": {"explicit_source_profile"},
    "POSESIM_SOURCE_WORLD_ORIENTATION": {"ENU"},
    "POSESIM_SOURCE_SURFACE_MODEL": {"EARTH_WGS84"},
    "POSESIM_SOURCE_ORIGIN_ELEVATION_DATUM": {"gazebo_spherical_coordinates_elevation"},
    "POSESIM_RTK_ALTITUDE_DATUM": {"msl", "ellipsoid"},
    "POSESIM_RTK_ALTITUDE_FIELD": {"altitude_msl_m", "altitude_ellipsoid_m"},
    "POSESIM_TARGET_POINT_ALIGNMENT_MODE": {"zero_lever_arm_assumption", "lever_arm"},
    "POSESIM_EVALUATION_TIME_SOURCE": {"gazebo_sim_time", "ros_time"},
    "POSESIM_TIME_ALIGNMENT_MODE": {"exact", "interpolate", "nearest"},
    "POSESIM_RECORD_FORMAT": {"jsonl"},
    "POSESIM_RVIZ_FIXED_FRAME": {"gazebo_world_enu"},
    "POSESIM_RVIZ_RAY_MODE": {"normalized", "full_distance"},
}

REQUIRED_NONEMPTY = {
    key
    for key, value_type in TYPE_BY_KEY.items()
    if value_type in {"string", "topic"}
}

TOPIC_KEYS = {
    key for key, value_type in TYPE_BY_KEY.items() if value_type == "topic"
}
OUTPUT_TOPIC_KEYS = {
    "POSESIM_POSE_ERROR_TOPIC",
    "POSESIM_STEREO_GEOMETRY_TOPIC",
    "POSESIM_STATUS_TOPIC",
    "POSESIM_GT_IDEAL_POSE_TOPIC",
    "POSESIM_GT_IDEAL_RIG_POSE_TOPIC",
    "POSESIM_GT_IDEAL_LEFT_POSE_TOPIC",
    "POSESIM_GT_IDEAL_RIGHT_POSE_TOPIC",
    "POSESIM_RTK_IDEAL_POSE_TOPIC",
    "POSESIM_DIAGNOSTICS_TOPIC",
    "POSESIM_MARKER_TOPIC",
}
INPUT_TOPIC_KEYS = TOPIC_KEYS - OUTPUT_TOPIC_KEYS

NONNEGATIVE_FLOAT_KEYS = {
    key
    for key, value_type in TYPE_BY_KEY.items()
    if value_type in {"float", "optional_float"}
} - {
    # Geographic coordinates and world offsets are signed by definition.
    "POSESIM_SOURCE_ORIGIN_LATITUDE_DEG",
    "POSESIM_SOURCE_ORIGIN_LONGITUDE_DEG",
    "POSESIM_SOURCE_ORIGIN_ELEVATION_M",
    "POSESIM_GAZEBO_ORIGIN_EAST_M",
    "POSESIM_GAZEBO_ORIGIN_NORTH_M",
    "POSESIM_GAZEBO_ORIGIN_UP_M",
}


def _parse_env_file(path: Path) -> dict[str, str]:
    """Parse a data-only env file without executing shell syntax."""

    if not path.is_file():
        raise ConfigError(f"configuration file does not exist: {path}")

    values: dict[str, str] = {}
    for line_number, raw_line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("export "):
            line = line[len("export ") :].lstrip()
        if "=" not in line:
            raise ConfigError(f"{path}:{line_number}: expected KEY=VALUE")
        key, raw_value = line.split("=", 1)
        key = key.strip()
        if key not in TYPE_BY_KEY:
            raise ConfigError(f"{path}:{line_number}: unknown configuration key {key!r}")
        if not POSESIM_NAME_PATTERN.fullmatch(key):
            raise ConfigError(f"{path}:{line_number}: invalid configuration key {key!r}")
        if key in values:
            raise ConfigError(f"{path}:{line_number}: duplicate configuration key {key!r}")
        try:
            tokens = shlex.split(raw_value, comments=True, posix=True)
        except ValueError as exc:
            raise ConfigError(f"{path}:{line_number}: invalid quoting: {exc}") from exc
        if len(tokens) > 1:
            raise ConfigError(f"{path}:{line_number}: value must be a single token")
        values[key] = tokens[0] if tokens else ""
    return values


def _capture_explicit_environment(environ: Mapping[str, str]) -> tuple[dict[str, str], tuple[str, ...]]:
    overrides: dict[str, str] = {}
    names: list[str] = []
    for key, value in environ.items():
        if not key.startswith("POSESIM_"):
            continue
        if key in SELECTION_KEYS:
            continue
        if key not in TYPE_BY_KEY:
            raise ConfigError(f"unknown PoseSimulation environment override: {key}")
        overrides[key] = value
        names.append(key)
    return overrides, tuple(sorted(names))


def _resolve_profile_path(raw_path: str, *, cwd: Path) -> Path:
    candidate = Path(raw_path).expanduser()
    if candidate.is_absolute():
        return candidate.resolve()
    candidates = (
        cwd / candidate,
        ROOT_DIR / "config" / "profiles" / candidate,
        ROOT_DIR / candidate,
    )
    for path in candidates:
        if path.is_file():
            return path.resolve()
    searched = ", ".join(str(path) for path in candidates)
    raise ConfigError(f"profile file does not exist; searched: {searched}")


def _resolve_config_path(raw_path: str | Path | None, *, cwd: Path, environ: Mapping[str, str]) -> Path:
    if raw_path is None:
        raw_path = environ.get("POSESIM_CONFIG")
    if raw_path is None or str(raw_path) == "":
        return DEFAULT_CONFIG_PATH.resolve()
    path = Path(str(raw_path)).expanduser()
    if not path.is_absolute():
        path = cwd / path
    return path.resolve()


def _parse_bool(key: str, raw: str) -> bool:
    normalized = raw.strip().lower()
    if normalized in {"true", "1", "yes", "on"}:
        return True
    if normalized in {"false", "0", "no", "off"}:
        return False
    raise ConfigError(f"{key} must be a boolean, got {raw!r}")


def _parse_float(key: str, raw: str, *, optional: bool) -> float | None:
    if optional and raw.strip() == "":
        return None
    try:
        value = float(raw)
    except (TypeError, ValueError) as exc:
        raise ConfigError(f"{key} must be a finite number, got {raw!r}") from exc
    if not math.isfinite(value):
        raise ConfigError(f"{key} must be finite, got {raw!r}")
    return value


def _parse_int(key: str, raw: str, *, optional: bool = False) -> int | None:
    if optional and raw.strip() == "":
        return None
    try:
        value = int(raw, 10)
    except (TypeError, ValueError) as exc:
        raise ConfigError(f"{key} must be an integer, got {raw!r}") from exc
    return value


def _parse_values(raw_values: Mapping[str, str]) -> dict[str, object]:
    parsed: dict[str, object] = {}
    for key, value_type in TYPE_BY_KEY.items():
        raw = raw_values.get(key, "")
        if value_type == "bool":
            parsed[key] = _parse_bool(key, raw)
        elif value_type == "float":
            parsed[key] = _parse_float(key, raw, optional=False)
        elif value_type == "optional_float":
            parsed[key] = _parse_float(key, raw, optional=True)
        elif value_type == "enum":
            parsed[key] = raw.strip()
            allowed = ENUM_VALUES.get(key, set())
            if parsed[key] not in allowed:
                allowed_text = ", ".join(sorted(allowed))
                raise ConfigError(f"{key} must be one of {{{allowed_text}}}, got {raw!r}")
        elif value_type == "topic":
            parsed[key] = raw.strip()
        elif value_type == "optional_string":
            parsed[key] = raw
        elif value_type == "string":
            parsed[key] = raw.strip()
        else:  # pragma: no cover - schema programming error
            raise ConfigError(f"unsupported schema type {value_type!r} for {key}")
    return parsed


def _require_absolute_topic(values: Mapping[str, object], key: str) -> None:
    topic = str(values[key])
    if not topic or not topic.startswith("/") or topic.startswith("//") or any(
        character.isspace() for character in topic
    ):
        raise ConfigError(f"{key} must be a non-empty absolute ROS topic: {topic!r}")


def _validate(values: Mapping[str, object]) -> tuple[str, ...]:
    for key in REQUIRED_NONEMPTY:
        if not str(values[key]).strip():
            raise ConfigError(f"required configuration value is empty: {key}")

    if values["POSESIM_NAMESPACE"] != APPLICATION_NAMESPACE:
        raise ConfigError(
            f"POSESIM_NAMESPACE must be {APPLICATION_NAMESPACE!r}; "
            f"got {values['POSESIM_NAMESPACE']!r}"
        )
    if values["POSESIM_APPLICATION_NAME"] != "PoseSimulation":
        raise ConfigError("POSESIM_APPLICATION_NAME must be PoseSimulation")
    if values["POSESIM_NODE_NAME"] != "pose_simulation":
        raise ConfigError("POSESIM_NODE_NAME must be pose_simulation")
    if values["POSESIM_MODE"] != "diagnostic_only":
        raise ConfigError("PoseSimulation mode must be diagnostic_only")
    if values["POSESIM_READ_ONLY"] is not True:
        raise ConfigError("POSESIM_READ_ONLY must remain true")
    if values["POSESIM_START_SOURCE_APPLICATION"] is True:
        raise ConfigError("PoseSimulation cannot start the source application")
    if values["POSESIM_OWN_SOURCE_PROCESSES"] is True:
        raise ConfigError("PoseSimulation cannot own source application processes")

    for key in TOPIC_KEYS:
        _require_absolute_topic(values, key)
    for key in OUTPUT_TOPIC_KEYS:
        if not str(values[key]).startswith(OUTPUT_TOPIC_PREFIX):
            raise ConfigError(
                f"{key} must be owned by the /pose_simulation namespace: {values[key]!r}"
            )
    for key in INPUT_TOPIC_KEYS:
        if str(values[key]).startswith(OUTPUT_TOPIC_PREFIX):
            raise ConfigError(f"{key} cannot be owned by the PoseSimulation output namespace")

    if str(values["POSESIM_SOURCE_APPLICATION"]).strip().lower() == "simulation":
        for key in (
            "POSESIM_GT_TOPIC",
            "POSESIM_LEFT_CAMERA_POSE_TOPIC",
            "POSESIM_RIGHT_CAMERA_POSE_TOPIC",
        ):
            if not str(values[key]).startswith("/simulation/"):
                raise ConfigError(f"{key} must remain a /simulation input for the Simulation source")

    for key in (
        "POSESIM_ALLOW_GT_FALLBACK",
        "POSESIM_ALLOW_PX4_AS_GROUND_TRUTH",
        "POSESIM_ALLOW_PLANNED_POSITION_AS_GROUND_TRUTH",
    ):
        if values[key] is not False:
            raise ConfigError(f"{key} must be false; Ground Truth fallback is fail-closed")
    if values["POSESIM_GT_FALLBACK_POLICY"] != "fail_closed":
        raise ConfigError("POSESIM_GT_FALLBACK_POLICY must be fail_closed")
    if values["POSESIM_GT_MESSAGE_TYPE"] != "geometry_msgs/msg/PoseStamped":
        raise ConfigError(
            "POSESIM_GT_MESSAGE_TYPE must be geometry_msgs/msg/PoseStamped for the M2 adapter"
        )
    if values["POSESIM_CAMERA_POSE_MESSAGE_TYPE"] != "std_msgs/msg/String":
        raise ConfigError(
            "POSESIM_CAMERA_POSE_MESSAGE_TYPE must be std_msgs/msg/String for the M2 adapter"
        )
    if values["POSESIM_CAMERA_POSE_SCHEMA"] != "camera_pose_comparison/v1":
        raise ConfigError(
            "POSESIM_CAMERA_POSE_SCHEMA must be camera_pose_comparison/v1 for the M2 adapter"
        )

    if values["POSESIM_EXPECTED_WORLD_FRAME"] != "gazebo_world_enu":
        raise ConfigError("POSESIM_EXPECTED_WORLD_FRAME must be gazebo_world_enu")
    if values["POSESIM_EXPECTED_CAMERA_FRAME"] != "camera_optical_rdf":
        raise ConfigError("POSESIM_EXPECTED_CAMERA_FRAME must be camera_optical_rdf")
    if values["POSESIM_OUTPUT_FRAME"] != "baseline_center_frd":
        raise ConfigError("POSESIM_OUTPUT_FRAME must be baseline_center_frd")
    if values["POSESIM_EXPECTED_CAMERA_FORWARD_AXIS"] != "positive_z":
        raise ConfigError("POSESIM_EXPECTED_CAMERA_FORWARD_AXIS must be positive_z")
    if values["POSESIM_CAMERA_POSE_MODE"] not in {"static_snapshot", "dynamic_stream"}:
        raise ConfigError("POSESIM_CAMERA_POSE_MODE must be static_snapshot or dynamic_stream")

    if not -90.0 <= float(values["POSESIM_SOURCE_ORIGIN_LATITUDE_DEG"]) <= 90.0:
        raise ConfigError("POSESIM_SOURCE_ORIGIN_LATITUDE_DEG must be within [-90, 90]")
    if not -180.0 <= float(values["POSESIM_SOURCE_ORIGIN_LONGITUDE_DEG"]) <= 180.0:
        raise ConfigError("POSESIM_SOURCE_ORIGIN_LONGITUDE_DEG must be within [-180, 180]")

    if values["POSESIM_TARGET_POINT_ALIGNMENT_MODE"] == "zero_lever_arm_assumption":
        if values["POSESIM_LEVER_ARM_APPLIED"] is True:
            raise ConfigError("zero-lever-arm alignment cannot claim a lever arm was applied")
    elif values["POSESIM_LEVER_ARM_APPLIED"] is not True:
        raise ConfigError("lever_arm alignment requires POSESIM_LEVER_ARM_APPLIED=true")
    if values["POSESIM_LEVER_ARM_APPLIED"] is True and not str(
        values["POSESIM_LEVER_ARM_CONFIG_PATH"]
    ).strip():
        raise ConfigError("POSESIM_LEVER_ARM_CONFIG_PATH is required when lever arm is applied")
    if values["POSESIM_REQUIRE_ALIGNMENT_PROVENANCE"] is not True:
        raise ConfigError("POSESIM_REQUIRE_ALIGNMENT_PROVENANCE must remain true")

    if float(values["POSESIM_EVALUATION_HZ"]) <= 0.0:
        raise ConfigError("POSESIM_EVALUATION_HZ must be positive")
    for key in NONNEGATIVE_FLOAT_KEYS:
        value = values[key]
        if value is not None and float(value) < 0.0:
            raise ConfigError(f"{key} cannot be negative")
    if float(values["POSESIM_TREND_WINDOW_S"]) <= 0.0:
        raise ConfigError("POSESIM_TREND_WINDOW_S must be positive")
    if float(values["POSESIM_TREND_BUFFER_S"]) <= 0.0:
        raise ConfigError("POSESIM_TREND_BUFFER_S must be positive")
    if float(values["POSESIM_RVIZ_RAY_LENGTH_M"]) <= 0.0:
        raise ConfigError("POSESIM_RVIZ_RAY_LENGTH_M must be positive")
    if values["POSESIM_ALLOW_FUTURE_EXTRAPOLATION"] is True:
        raise ConfigError("future extrapolation is not permitted by the diagnostic configuration")

    if values["POSESIM_ENABLE_VERTICAL_RTK_COMPARISON"] is True:
        source_datum = values["POSESIM_SOURCE_ORIGIN_ELEVATION_DATUM"]
        rtk_datum = values["POSESIM_RTK_ALTITUDE_DATUM"]
        if source_datum != rtk_datum and values["POSESIM_ALLOW_UNRESOLVED_VERTICAL_DATUM"] is not True:
            raise ConfigError(
                "vertical RTK comparison requires an explicit datum conversion contract; "
                f"source={source_datum!r}, rtk={rtk_datum!r}"
            )

    output_topics = [str(values[key]) for key in OUTPUT_TOPIC_KEYS]
    if len(output_topics) != len(set(output_topics)):
        raise ConfigError("PoseSimulation output topics must be unique")

    warnings: list[str] = []
    if values["POSESIM_REQUIRE_RTK_FIXED_FOR_OPERATIONAL"] and not str(
        values["POSESIM_RTK_FIXED_STATUS_VALUES"]
    ).strip():
        warnings.append("RTK Fixed status values are unresolved; operational RTK validity is fail-closed")
    if not values["POSESIM_ENABLE_VERTICAL_RTK_COMPARISON"]:
        warnings.append("vertical RTK comparison is disabled until altitude datums are explicitly aligned")
    return tuple(warnings)


def resolve_config(
    *,
    config_path: str | Path | None = None,
    profile_path: str | Path | None = None,
    explicit_overrides: Mapping[str, str] | None = None,
    cli_overrides: Mapping[str, str] | None = None,
    environ: Mapping[str, str] | None = None,
    cwd: str | Path | None = None,
) -> ResolvedConfig:
    """Resolve and validate config, profile, environment, and CLI layers."""

    environment = dict(os.environ if environ is None else environ)
    current_dir = Path.cwd() if cwd is None else Path(cwd).expanduser().resolve()
    environment_overrides, environment_names = _capture_explicit_environment(environment)
    if explicit_overrides:
        for key, value in explicit_overrides.items():
            if key not in TYPE_BY_KEY:
                raise ConfigError(f"unknown explicit configuration key: {key}")
            environment_overrides[key] = str(value)
        environment_names = tuple(sorted(set(environment_names) | set(explicit_overrides)))

    resolved_config_path = _resolve_config_path(config_path, cwd=current_dir, environ=environment)
    selected_profile = profile_path
    if selected_profile is None:
        selected_profile = environment.get("POSESIM_PROFILE_CONFIG") or None
    resolved_profile_path: Path | None = None
    raw_values = _parse_env_file(resolved_config_path)
    if selected_profile:
        resolved_profile_path = _resolve_profile_path(str(selected_profile), cwd=current_dir)
        raw_values.update(_parse_env_file(resolved_profile_path))
    raw_values.update(environment_overrides)
    provenance_names = set(environment_names)
    if cli_overrides:
        for key, value in cli_overrides.items():
            if key not in TYPE_BY_KEY:
                raise ConfigError(f"unknown command-line configuration key: {key}")
            raw_values[key] = str(value)
            provenance_names.add(key)

    values = _parse_values(raw_values)
    warnings = _validate(values)
    provenance = ConfigProvenance(
        config_path=str(resolved_config_path),
        profile_path=str(resolved_profile_path) if resolved_profile_path else None,
        explicit_override_names=tuple(sorted(provenance_names)),
    )
    return ResolvedConfig(MappingProxyType(values), provenance, warnings)


def _format_value(value: object) -> str:
    if value is None:
        return '""'
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, float):
        return format(value, ".15g")
    return json.dumps(str(value), ensure_ascii=False)


def format_snapshot(config: ResolvedConfig) -> str:
    """Return a deterministic, human-readable resolved snapshot."""

    provenance = config.provenance
    lines = [
        f"provenance.config_path={provenance.config_path}",
        f"provenance.profile_path={provenance.profile_path or '<none>'}",
        "provenance.explicit_override_names="
        + (",".join(provenance.explicit_override_names) or "<none>"),
        f"provenance.source_profile_id={config.values['POSESIM_SOURCE_PROFILE_ID']}",
        f"provenance.application_namespace={config.values['POSESIM_NAMESPACE']}",
        f"provenance.source_application={config.values['POSESIM_SOURCE_APPLICATION']}",
        f"provenance.read_only={_format_value(config.values['POSESIM_READ_ONLY'])}",
    ]
    lines.extend(f"{key}={_format_value(config.values[key])}" for key in sorted(config.values))
    lines.extend(f"validation.warning={warning}" for warning in config.warnings)
    return "\n".join(lines)


def _parse_cli_overrides(items: Sequence[str]) -> dict[str, str]:
    overrides: dict[str, str] = {}
    for item in items:
        if "=" not in item:
            raise ConfigError(f"--set requires POSESIM_KEY=VALUE, got {item!r}")
        key, value = item.split("=", 1)
        key = key.strip()
        if key not in TYPE_BY_KEY:
            raise ConfigError(f"unknown command-line configuration key: {key}")
        if key in SELECTION_KEYS:
            raise ConfigError(f"{key} must be supplied with --config or --profile")
        overrides[key] = value
    return overrides


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Validate or print the independent PoseSimulation configuration."
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--check-config", action="store_true", help="validate the resolved configuration")
    mode.add_argument("--print-config", action="store_true", help="print the resolved configuration")
    parser.add_argument("--config", help="PoseSimulation base env file")
    parser.add_argument("--profile", help="optional profile env file")
    parser.add_argument(
        "--set",
        dest="overrides",
        action="append",
        default=[],
        metavar="POSESIM_KEY=VALUE",
        help="highest-precedence configuration override; may be repeated",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    try:
        config = resolve_config(
            config_path=args.config,
            profile_path=args.profile,
            cli_overrides=_parse_cli_overrides(args.overrides),
        )
    except ConfigError as exc:
        print(f"PoseSimulation configuration error: {exc}", file=sys.stderr)
        return 2

    if args.print_config:
        print(format_snapshot(config))
    else:
        print("PoseSimulation configuration valid.")
        print(f"config_path={config.provenance.config_path}")
        print(f"profile_path={config.provenance.profile_path or '<none>'}")
        for warning in config.warnings:
            print(f"warning={warning}")
    return 0


if __name__ == "__main__":  # pragma: no cover - exercised by the runner
    raise SystemExit(main())
