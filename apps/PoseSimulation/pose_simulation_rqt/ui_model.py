"""Pure, read-only view-model for the PoseSimulation rqt plugin.

This module is deliberately kept inside the rqt package.  It adapts the
already-published typed PoseSimulation messages for display; it does not
recompute attitude, coordinate transforms, or time alignment.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from types import MappingProxyType
from typing import Any, Mapping


WORLD_FRAME = "gazebo_world_enu"
RIG_FRAME = "baseline_center_frd"
OPTICAL_FRAME = "camera_optical_rdf"
CLOCK_DOMAIN = "Gazebo simulation time"
UNRESOLVED_CLOCK_DOMAIN = "Unresolved clock domain"


def _value(message: Any, key: str, default: Any = None) -> Any:
    if isinstance(message, Mapping):
        return message.get(key, default)
    return getattr(message, key, default)


def _finite(value: Any) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def _integer(value: Any) -> int | None:
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        return None
    return parsed


def _stamp_ns(message: Any) -> int | None:
    header = _value(message, "header")
    stamp = _value(header, "stamp")
    sec = _integer(_value(stamp, "sec"))
    nanosec = _integer(_value(stamp, "nanosec"))
    if sec is not None and nanosec is not None:
        value = sec * 1_000_000_000 + nanosec
        return value if value >= 0 else None
    for key in ("evaluation_timestamp_ns", "evaluation_stamp_ns", "stamp_ns"):
        value = _integer(_value(message, key))
        if value is not None and value >= 0:
            return value
    return None


def _evaluation_id(message: Any) -> int | None:
    value = _integer(_value(message, "evaluation_id"))
    if value is not None and value > 0:
        return value
    # Legacy JSON did not have a separate sequence ID.  The UI does not
    # manufacture one; a timestamp is used only as a compatibility identity.
    timestamp = _integer(_value(message, "evaluation_timestamp_ns"))
    if timestamp is not None and timestamp > 0:
        return timestamp
    return None


def _bool_value(message: Any, key: str, default: bool = False) -> bool:
    value = _value(message, key, default)
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "valid"}
    return bool(value)


def _mapping_copy(message: Any, fields: tuple[str, ...]) -> Mapping[str, Any]:
    if isinstance(message, Mapping):
        return MappingProxyType(dict(message))
    return MappingProxyType({field: _value(message, field) for field in fields})


@dataclass(frozen=True)
class CameraErrorMetrics:
    """Displayed camera errors; angles are degrees and ``None`` is invalid."""

    roll_deg: float | None
    pitch_deg: float | None
    yaw_deg: float | None
    rotation_deg: float | None
    los_deg: float | None
    valid: bool
    gimbal_lock: bool


@dataclass(frozen=True)
class StereoMetrics:
    """Displayed stereo metrics; distances are metres and angles degrees."""

    relative_roll_deg: float | None
    relative_pitch_deg: float | None
    relative_yaw_deg: float | None
    relative_rotation_deg: float | None
    optical_axis_separation_deg: float | None
    actual_baseline_length_m: float | None
    nominal_baseline_length_m: float | None
    baseline_length_error_m: float | None
    baseline_direction_error_deg: float | None
    actual_baseline_vector_world_enu_m: tuple[float | None, float | None, float | None]
    actual_midpoint_world_enu_m: tuple[float | None, float | None, float | None]
    valid: bool


@dataclass(frozen=True)
class UiSnapshot:
    """One atomically paired result sample for the Qt main thread."""

    evaluation_id: int | None
    evaluation_stamp_ns: int | None
    sim_time_s: float | None
    aligned: bool
    valid: bool
    overall_state: str
    overall_label: str
    reason: str
    raw_state: str
    gt_state: str
    left_state: str
    right_state: str
    frame_state: str
    pose_error: CameraErrorMetrics
    right_error: CameraErrorMetrics
    stereo: StereoMetrics
    status: Mapping[str, Any]
    pose_error_raw: Mapping[str, Any]
    stereo_raw: Mapping[str, Any]
    source_stamp_ns: Mapping[str, int | None]
    camera_source_mode: str
    clock_domain: str
    source_topics: Mapping[str, str]

    @property
    def token(self) -> tuple[Any, ...]:
        return (self.evaluation_id, self.evaluation_stamp_ns, self.raw_state, self.reason)


def status_reason(raw_state: str | None, detail: str | None = None) -> str:
    """Return the single user-facing reason used by every UI surface."""

    raw = str(raw_state or "UNAVAILABLE").strip().upper()
    mapping = {
        "VALID": "",
        "WAITING_FOR_GT": "Waiting for Ground Truth",
        "WAITING_FOR_LEFT_CAMERA": "Waiting for left camera pose",
        "WAITING_FOR_RIGHT_CAMERA": "Waiting for right camera pose",
        "CAMERA_SCHEMA_MISMATCH": "Camera message format does not match this PoseSimulation version",
        "CAMERA_SIDE_MISMATCH": "Camera message side does not match the configured source",
        "CAMERA_ENTITY_CHANGED": "Camera entity identity changed during this run",
        "CAMERA_SDF_HASH_CHANGED": "Camera source model identity changed during this run",
        "TIME_ALIGNMENT_FAILED": "Source timestamps cannot be aligned",
        "PAIR_ID_MISMATCH": "Result messages belong to different evaluations",
        "TIMESTAMP_MISMATCH": "Result timestamps do not match",
        "FRAME_MISMATCH": "Incoming frame does not match gazebo_world_enu",
        "GT_FRAME_MISMATCH": "Ground Truth frame does not match gazebo_world_enu",
        "STALE": "A required source is stale",
        "SOURCE_STALE": "A required source is stale",
        "INVALID_FIX": "The current input reference is invalid",
        "NONFINITE": "A source contains a non-finite value",
        "UNAVAILABLE": "Waiting for PoseSimulation sources",
    }
    text = mapping.get(raw, "PoseSimulation input is invalid")
    detail_text = str(detail or "").strip()
    if detail_text and raw not in {"VALID", "UNAVAILABLE"}:
        # Keep the semantic mapping first, then expose a concise producer
        # reason without leaking an all-caps enum into the overview.
        return f"{text}: {detail_text}"
    return text


def state_label(state: str) -> str:
    """Compact label for badges; raw enums remain in Diagnostics."""

    value = str(state or "UNAVAILABLE").upper()
    return {
        "VALID": "✓",
        "WARNING": "△",
        "DEGRADED": "△",
        "UNALIGNED": "△",
        "PAIR ID MISMATCH": "△",
        "TIMESTAMP MISMATCH": "△",
        "FRAME MISMATCH": "✕",
        "INVALID": "✕",
        "STALE": "○",
        "UNAVAILABLE": "—",
        "WAITING": "—",
    }.get(value, "✕")


def source_state_label(state: str) -> str:
    return {
        "VALID": "Valid",
        "WARNING": "Warning",
        "UNALIGNED": "Unaligned",
        "INVALID": "Invalid",
        "STALE": "Stale",
        "WAITING": "Waiting",
        "UNAVAILABLE": "Unavailable",
    }.get(str(state or "UNAVAILABLE").upper(), "Invalid")


def format_metric(value: Any, *, unit: str = "", precision: int = 3) -> str:
    """Format a finite metric; invalid values are never rendered as zero."""

    parsed = _finite(value)
    if parsed is None:
        return "—"
    return f"{parsed:+.{precision}f}{unit}" if unit == "°" else f"{parsed:.{precision}f}{unit}"


def _metric_value(message: Any, direct_key: str, nested: Any, nested_key: str) -> float | None:
    value = _value(message, direct_key)
    if value is None and isinstance(nested, Mapping):
        value = nested.get(nested_key)
    return _finite(value)


def _camera_metrics(message: Any, side: str) -> CameraErrorMetrics:
    prefix = f"{side}_actual_vs_gt_ideal"
    nested = _value(message, prefix)
    valid = _bool_value(nested, "valid") if isinstance(nested, Mapping) else _bool_value(message, "valid")
    return CameraErrorMetrics(
        _metric_value(message, f"{side}_roll_error_deg", nested, "roll_error_deg"),
        _metric_value(message, f"{side}_pitch_error_deg", nested, "pitch_error_deg"),
        _metric_value(message, f"{side}_yaw_error_deg", nested, "yaw_error_deg"),
        _metric_value(message, f"{side}_rotation_error_deg", nested, "rotation_error_deg"),
        _metric_value(
            message,
            f"{side}_optical_axis_error_deg",
            nested,
            "optical_axis_to_gt_target_error_deg",
        ),
        valid,
        _bool_value(message, f"{side}_gimbal_lock")
        if not isinstance(nested, Mapping)
        else _bool_value(nested, "gimbal_lock"),
    )


def _vector(message: Any, direct_key: str, nested: Any, nested_key: str) -> tuple[float | None, float | None, float | None]:
    value = _value(message, direct_key)
    if value is None and isinstance(nested, Mapping):
        value = nested.get(nested_key)
    if value is not None and not isinstance(value, Mapping):
        try:
            if len(value) == 3:
                return tuple(_finite(component) for component in value)  # type: ignore[return-value]
        except (TypeError, ValueError):
            pass
    if value is not None and isinstance(value, Mapping):
        return (_finite(value.get("x")), _finite(value.get("y")), _finite(value.get("z")))
    if value is not None and all(hasattr(value, axis) for axis in ("x", "y", "z")):
        return (_finite(getattr(value, "x")), _finite(getattr(value, "y")), _finite(getattr(value, "z")))
    return (None, None, None)


def _stereo_metrics(message: Any) -> StereoMetrics:
    baseline = _value(message, "baseline")
    valid = _bool_value(baseline, "valid") if isinstance(baseline, Mapping) else _bool_value(message, "valid")
    return StereoMetrics(
        _metric_value(message, "relative_roll_error_deg", message, "relative_roll_error_deg"),
        _metric_value(message, "relative_pitch_error_deg", message, "relative_pitch_error_deg"),
        _metric_value(message, "relative_yaw_error_deg", message, "relative_yaw_error_deg"),
        _metric_value(message, "relative_rotation_error_deg", message, "relative_rotation_error_deg"),
        _metric_value(message, "optical_axis_angle_deg", message, "left_right_optical_axis_angle_deg"),
        _metric_value(message, "actual_baseline_length_m", baseline, "actual_length_m"),
        _metric_value(message, "nominal_baseline_length_m", baseline, "nominal_length_m"),
        _metric_value(message, "baseline_length_error_m", baseline, "length_error_m"),
        _metric_value(message, "baseline_direction_error_deg", baseline, "direction_error_deg"),
        _vector(message, "actual_baseline_vector_world_enu_m", baseline, "vector_world_enu_m"),
        _vector(message, "actual_midpoint_world_enu_m", baseline, "midpoint_world_enu_m"),
        valid,
    )


def _status_mapping(message: Any) -> Mapping[str, Any]:
    fields = (
        "evaluation_id",
        "valid",
        "state",
        "reason",
        "gt_topic",
        "gt_reference_name",
        "left_camera_topic",
        "right_camera_topic",
        "left_entity_name",
        "left_entity_id",
        "right_entity_name",
        "right_entity_id",
        "left_sdf_sha256",
        "right_sdf_sha256",
        "pose_contract_sha256",
        "camera_source_mode",
        "read_only",
        "pose_command_enabled",
        "rtk_comparison_implemented",
    )
    return _mapping_copy(message, fields)


def _frame_state(pose_error: Any, stereo: Any) -> str:
    values = (
        _value(pose_error, "world_frame"),
        _value(pose_error, "error_frame"),
        _value(stereo, "world_frame"),
        _value(stereo, "rig_frame"),
    )
    if all(value is None for value in values):
        return "UNAVAILABLE"
    expected = (WORLD_FRAME, OPTICAL_FRAME, WORLD_FRAME, RIG_FRAME)
    return "VALID" if all(value in (None, expected[index]) for index, value in enumerate(values)) else "INVALID"


def _source_states(raw_state: str, *, aligned: bool, valid: bool, frame_state: str) -> tuple[str, str, str, str]:
    raw = raw_state.upper()
    if valid:
        return "VALID", "VALID", "VALID", "VALID"
    if raw == "WAITING_FOR_GT":
        return "WAITING", "UNAVAILABLE", "UNAVAILABLE", "WAITING"
    if raw == "WAITING_FOR_LEFT_CAMERA":
        return "VALID", "WAITING", "UNAVAILABLE", "WAITING"
    if raw == "WAITING_FOR_RIGHT_CAMERA":
        return "VALID", "UNAVAILABLE", "WAITING", "WAITING"
    if not aligned:
        return "VALID", "VALID", "VALID", "UNALIGNED"
    if frame_state == "INVALID":
        return "INVALID", "INVALID", "INVALID", "INVALID"
    if raw in {"STALE", "SOURCE_STALE"}:
        return "STALE", "STALE", "STALE", "STALE"
    return "INVALID", "INVALID", "INVALID", "VALID" if frame_state == "VALID" else "UNAVAILABLE"


def build_ui_snapshot(
    pose_error: Any | None,
    stereo_geometry: Any | None,
    status: Any | None,
) -> UiSnapshot:
    """Pair current typed results without calculating any diagnostic metric."""

    pose_id = _evaluation_id(pose_error)
    stereo_id = _evaluation_id(stereo_geometry)
    status_id = _evaluation_id(status)
    stamp_values = [_stamp_ns(value) for value in (pose_error, stereo_geometry, status)]
    present_ids = [value for value in (pose_id, stereo_id, status_id) if value is not None]
    raw_state = str(_value(status, "state", "UNAVAILABLE") or "UNAVAILABLE").upper()
    detail = str(_value(status, "reason", "") or "")
    status_map = _status_mapping(status) if status is not None else MappingProxyType({})
    pose_raw = _mapping_copy(
        pose_error,
        (
            "evaluation_id",
            "valid",
            "status",
            "reference_type",
            "world_frame",
            "error_frame",
            "euler_convention",
            "gt_stamp_ns",
            "left_camera_source_stamp_ns",
            "right_camera_source_stamp_ns",
            "left_roll_error_deg",
            "left_pitch_error_deg",
            "left_yaw_error_deg",
            "left_rotation_error_deg",
            "left_optical_axis_error_deg",
            "left_gimbal_lock",
            "right_roll_error_deg",
            "right_pitch_error_deg",
            "right_yaw_error_deg",
            "right_rotation_error_deg",
            "right_optical_axis_error_deg",
            "right_gimbal_lock",
        ),
    ) if pose_error is not None else MappingProxyType({})
    stereo_raw = _mapping_copy(
        stereo_geometry,
        (
            "evaluation_id",
            "valid",
            "status",
            "world_frame",
            "rig_frame",
            "relative_roll_error_deg",
            "relative_pitch_error_deg",
            "relative_yaw_error_deg",
            "relative_rotation_error_deg",
            "optical_axis_angle_deg",
            "actual_baseline_length_m",
            "nominal_baseline_length_m",
            "baseline_length_error_m",
            "baseline_direction_error_deg",
        ),
    ) if stereo_geometry is not None else MappingProxyType({})
    source_topics = MappingProxyType(
        {
            "gt": str(_value(status, "gt_topic", "") or ""),
            "left": str(_value(status, "left_camera_topic", "") or ""),
            "right": str(_value(status, "right_camera_topic", "") or ""),
        }
    )

    if not present_ids:
        aligned = False
        evaluation_id = None
        evaluation_stamp = None
        state = "WAITING_FOR_SOURCES"
        reason = status_reason(raw_state, detail)
    else:
        aligned = (
            pose_id is not None
            and stereo_id is not None
            and status_id is not None
            and len(set((pose_id, stereo_id, status_id))) == 1
            and all(stamp is not None for stamp in stamp_values)
            and len(set(stamp for stamp in stamp_values if stamp is not None)) == 1
        )
        evaluation_id = pose_id if aligned else (present_ids[0] if len(set(present_ids)) == 1 else None)
        evaluation_stamp = stamp_values[0] if aligned else None
        if not aligned:
            state = "ALIGNMENT_PENDING"
            reason = "Result messages do not share one evaluation sample"
        else:
            state = raw_state
            reason = status_reason(raw_state, detail)

    frame = _frame_state(pose_error, stereo_geometry)
    message_valid = (
        aligned
        and _bool_value(pose_error, "valid")
        and _bool_value(stereo_geometry, "valid")
        and _bool_value(status, "valid")
        and frame == "VALID"
    )
    if not aligned and present_ids:
        overall_state = "ALIGNMENT_PENDING"
        overall_label = "Alignment pending"
    elif message_valid:
        overall_state = "VALID"
        overall_label = "Measuring"
    elif raw_state in {"WAITING_FOR_GT", "WAITING_FOR_LEFT_CAMERA", "WAITING_FOR_RIGHT_CAMERA", "WAITING_FOR_SOURCES", "UNAVAILABLE"}:
        overall_state = "WAITING_FOR_SOURCES"
        overall_label = "Waiting for sources"
    elif raw_state in {"STALE", "SOURCE_STALE"}:
        overall_state = "STALE"
        overall_label = "Source stale"
    else:
        overall_state = "INPUT_INVALID"
        overall_label = "Input invalid"
    if not aligned and not present_ids:
        overall_state = "WAITING_FOR_SOURCES"
        overall_label = "Waiting for sources"
    if not reason:
        reason = ""

    left = _camera_metrics(pose_error, "left") if pose_error is not None and message_valid else CameraErrorMetrics(None, None, None, None, None, False, False)
    right = _camera_metrics(pose_error, "right") if pose_error is not None and message_valid else CameraErrorMetrics(None, None, None, None, None, False, False)
    stereo = _stereo_metrics(stereo_geometry) if stereo_geometry is not None and message_valid else StereoMetrics(None, None, None, None, None, None, None, None, None, (None, None, None), (None, None, None), False)
    gt_state, left_state, right_state, frame_header_state = _source_states(
        raw_state, aligned=aligned, valid=message_valid, frame_state=frame
    )
    if pose_error is None:
        gt_state = "WAITING"
        if raw_state == "WAITING_FOR_LEFT_CAMERA":
            left_state = "WAITING"
        elif raw_state == "WAITING_FOR_RIGHT_CAMERA":
            right_state = "WAITING"
    if frame == "INVALID":
        frame_header_state = "INVALID"
    camera_mode = str(_value(status, "camera_source_mode", "") or "")
    clock_domain = CLOCK_DOMAIN if evaluation_stamp is not None else UNRESOLVED_CLOCK_DOMAIN
    source_stamps = MappingProxyType(
        {
            "gt": _integer(_value(pose_error, "gt_stamp_ns")),
            "left": _integer(_value(pose_error, "left_camera_source_stamp_ns")),
            "right": _integer(_value(pose_error, "right_camera_source_stamp_ns")),
        }
    )
    return UiSnapshot(
        evaluation_id=evaluation_id,
        evaluation_stamp_ns=evaluation_stamp,
        sim_time_s=evaluation_stamp / 1.0e9 if evaluation_stamp is not None else None,
        aligned=aligned,
        valid=message_valid,
        overall_state=overall_state,
        overall_label=overall_label,
        reason=reason,
        raw_state=raw_state,
        gt_state=gt_state,
        left_state=left_state,
        right_state=right_state,
        frame_state=frame_header_state,
        pose_error=left,
        right_error=right,
        stereo=stereo,
        status=status_map,
        pose_error_raw=pose_raw,
        stereo_raw=stereo_raw,
        source_stamp_ns=source_stamps,
        camera_source_mode=camera_mode,
        clock_domain=clock_domain,
        source_topics=source_topics,
    )


__all__ = [
    "CLOCK_DOMAIN",
    "CameraErrorMetrics",
    "OPTICAL_FRAME",
    "RIG_FRAME",
    "StereoMetrics",
    "UiSnapshot",
    "WORLD_FRAME",
    "build_ui_snapshot",
    "format_metric",
    "source_state_label",
    "state_label",
    "status_reason",
]
