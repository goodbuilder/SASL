"""Typed, evaluation-atomic inputs for PoseSimulation RViz diagnostics.

This module is deliberately independent of ROS and of the PoseSimulation
measurement implementation.  It validates the already-published typed
messages and assembles one immutable world-frame snapshot.  It does not
recompute attitude, LOS, baseline, or time alignment values.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Any, Mapping


WORLD_FRAME = "gazebo_world_enu"
RIG_FRAME = "baseline_center_frd"
LEFT_OPTICAL_FRAME = "left_camera_optical_rdf"
RIGHT_OPTICAL_FRAME = "right_camera_optical_rdf"
VEHICLE_REFERENCE_FRAME = "vehicle_reference_link"


Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]


@dataclass(frozen=True)
class WorldPose:
    """Pose of a named local frame, expressed in ``gazebo_world_enu``.

    Position is metres and orientation is ROS ``(x, y, z, w)``.  The ROS
    ``PoseStamped.header.frame_id`` remains the world frame; ``local_frame``
    records the frame represented by the pose topic.
    """

    local_frame: str
    position_world_enu_m: Vector3
    orientation_xyzw: Quaternion
    stamp_ns: int
    source_topic: str


@dataclass(frozen=True)
class CameraErrorMetrics:
    """M3 camera error values in degrees, copied without recalculation."""

    roll_error_deg: float
    pitch_error_deg: float
    yaw_error_deg: float
    rotation_error_deg: float
    optical_axis_error_deg: float
    gimbal_lock: bool


@dataclass(frozen=True)
class StereoGeometryMetrics:
    """M3 stereo geometry values in metres/degrees, copied without recalculation."""

    relative_roll_error_deg: float
    relative_pitch_error_deg: float
    relative_yaw_error_deg: float
    relative_rotation_error_deg: float
    optical_axis_angle_deg: float
    actual_baseline_vector_world_enu_m: Vector3
    actual_baseline_length_m: float
    nominal_baseline_length_m: float
    baseline_length_error_m: float
    baseline_direction_error_deg: float
    actual_midpoint_world_enu_m: Vector3


@dataclass(frozen=True)
class VisualizationSnapshot:
    """One evaluation identity and all geometry that belongs to it."""

    evaluation_id: int
    evaluation_stamp_ns: int
    world_frame: str
    valid: bool
    state: str
    invalid_reason: str
    ground_truth_source: str = ""
    gt_reference_name: str = ""
    source_identity: Mapping[str, str] = field(default_factory=dict)
    gt_target: WorldPose | None = None
    rtk_target: WorldPose | None = None
    actual_left: WorldPose | None = None
    actual_right: WorldPose | None = None
    ideal_gt_rig: WorldPose | None = None
    ideal_gt_left: WorldPose | None = None
    ideal_gt_right: WorldPose | None = None
    ideal_rtk_rig: WorldPose | None = None
    ideal_rtk_left: WorldPose | None = None
    ideal_rtk_right: WorldPose | None = None
    left_error: CameraErrorMetrics | None = None
    right_error: CameraErrorMetrics | None = None
    stereo: StereoGeometryMetrics | None = None
    rtk_available: bool = False

    def __post_init__(self) -> None:
        object.__setattr__(self, "source_identity", MappingProxyType(dict(self.source_identity)))


def _attribute(message: Any, name: str, default: Any = None) -> Any:
    if isinstance(message, Mapping):
        return message.get(name, default)
    return getattr(message, name, default)


def _header(message: Any) -> Any:
    return _attribute(message, "header")


def stamp_ns(message: Any) -> int | None:
    header = _header(message)
    stamp = _attribute(header, "stamp")
    if stamp is None:
        return None
    try:
        seconds = int(_attribute(stamp, "sec"))
        nanoseconds = int(_attribute(stamp, "nanosec"))
    except (TypeError, ValueError):
        return None
    if seconds < 0 or nanoseconds < 0 or nanoseconds >= 1_000_000_000:
        return None
    return seconds * 1_000_000_000 + nanoseconds


def _header_frame(message: Any) -> str:
    return str(_attribute(_header(message), "frame_id", "") or "")


def _finite_tuple(value: Any, size: int) -> tuple[float, ...] | None:
    if value is None:
        return None
    if isinstance(value, Mapping):
        components = [value.get(axis) for axis in ("x", "y", "z", "w")[:size]]
    else:
        try:
            components = list(value)
        except TypeError:
            components = [
                _attribute(value, axis, None)
                for axis in ("x", "y", "z", "w")[:size]
            ]
    if len(components) != size:
        return None
    try:
        parsed = tuple(float(component) for component in components)
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(component) for component in parsed):
        return None
    return parsed


def _pose_components(message: Any) -> tuple[Vector3, Quaternion] | None:
    pose = _attribute(message, "pose")
    position = _attribute(pose, "position")
    orientation = _attribute(pose, "orientation")
    position_values = _finite_tuple(position, 3)
    orientation_values = _finite_tuple(orientation, 4)
    if position_values is None or orientation_values is None:
        return None
    norm = math.sqrt(sum(value * value for value in orientation_values))
    if norm <= 1.0e-12 or not math.isfinite(norm):
        return None
    return (
        position_values,  # type: ignore[return-value]
        tuple(value / norm for value in orientation_values),  # type: ignore[return-value]
    )


def world_pose_from_pose_stamped(
    message: Any,
    *,
    local_frame: str,
    source_topic: str,
    expected_stamp_ns: int | None = None,
) -> WorldPose | None:
    """Validate one world-frame ``PoseStamped`` without changing its meaning."""

    if message is None or _header_frame(message) != WORLD_FRAME:
        return None
    message_stamp = stamp_ns(message)
    if message_stamp is None or (expected_stamp_ns is not None and message_stamp != expected_stamp_ns):
        return None
    components = _pose_components(message)
    if components is None:
        return None
    position, orientation = components
    return WorldPose(local_frame, position, orientation, message_stamp, source_topic)


def _message_identity(message: Any) -> tuple[int | None, int | None]:
    value = _attribute(message, "evaluation_id")
    try:
        evaluation_id = int(value)
    except (TypeError, ValueError):
        evaluation_id = None
    return evaluation_id, stamp_ns(message)


def _finite_metric(message: Any, field_name: str) -> float | None:
    try:
        value = float(_attribute(message, field_name))
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _vector_field(message: Any, field_name: str) -> Vector3 | None:
    value = _attribute(message, field_name)
    return _finite_tuple(value, 3)  # type: ignore[return-value]


def _camera_error(message: Any, prefix: str) -> CameraErrorMetrics | None:
    values = tuple(
        _finite_metric(message, f"{prefix}_{field}")
        for field in (
            "roll_error_deg",
            "pitch_error_deg",
            "yaw_error_deg",
            "rotation_error_deg",
            "optical_axis_error_deg",
        )
    )
    if any(value is None for value in values):
        return None
    return CameraErrorMetrics(*values, bool(_attribute(message, f"{prefix}_gimbal_lock", False)))  # type: ignore[arg-type]


def _stereo_metrics(message: Any) -> StereoGeometryMetrics | None:
    scalar_names = (
        "relative_roll_error_deg",
        "relative_pitch_error_deg",
        "relative_yaw_error_deg",
        "relative_rotation_error_deg",
        "optical_axis_angle_deg",
        "actual_baseline_length_m",
        "nominal_baseline_length_m",
        "baseline_length_error_m",
        "baseline_direction_error_deg",
    )
    values = tuple(_finite_metric(message, name) for name in scalar_names)
    vector = _vector_field(message, "actual_baseline_vector_world_enu_m")
    midpoint = _vector_field(message, "actual_midpoint_world_enu_m")
    if any(value is None for value in values) or vector is None or midpoint is None:
        return None
    return StereoGeometryMetrics(
        values[0], values[1], values[2], values[3], values[4],  # type: ignore[arg-type]
        vector,
        values[5], values[6], values[7], values[8],  # type: ignore[arg-type]
        midpoint,
    )


def _identity_matches(message: Any, evaluation_id: int, evaluation_stamp_ns: int) -> bool:
    message_id, message_stamp = _message_identity(message)
    return message_id == evaluation_id and message_stamp == evaluation_stamp_ns


def _source_identity(status: Any) -> dict[str, str]:
    fields = (
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
    )
    return {field: str(_attribute(status, field, "") or "") for field in fields}


def _invalid_snapshot(
    *,
    evaluation_id: int,
    evaluation_stamp_ns: int,
    state: str,
    reason: str,
    status: Any,
) -> VisualizationSnapshot:
    return VisualizationSnapshot(
        evaluation_id=evaluation_id,
        evaluation_stamp_ns=evaluation_stamp_ns,
        world_frame=WORLD_FRAME,
        valid=False,
        state=state,
        invalid_reason=reason,
        ground_truth_source=str(_attribute(status, "gt_reference_name", "") or ""),
        gt_reference_name=str(_attribute(status, "gt_reference_name", "") or ""),
        source_identity=_source_identity(status),
    )


def build_visualization_snapshot(
    *,
    pose_error: Any,
    stereo_geometry: Any,
    status: Any,
    gt_target: Any = None,
    actual_left: Any = None,
    actual_right: Any = None,
    ideal_gt_rig: Any = None,
    ideal_gt_left: Any = None,
    ideal_gt_right: Any = None,
    rtk_target: Any = None,
    ideal_rtk_rig: Any = None,
    ideal_rtk_left: Any = None,
    ideal_rtk_right: Any = None,
    source_topics: Mapping[str, str] | None = None,
) -> VisualizationSnapshot:
    """Pair typed PoseSimulation messages into one atomic visualization sample."""

    if status is None:
        return _invalid_snapshot(
            evaluation_id=0,
            evaluation_stamp_ns=0,
            state="WAITING_FOR_STATUS",
            reason="Waiting for PoseSimulation status",
            status={},
        )
    status_id, status_stamp = _message_identity(status)
    evaluation_id = 0 if status_id is None else status_id
    evaluation_stamp_ns = 0 if status_stamp is None else status_stamp
    source_identity = _source_identity(status)
    if source_topics:
        source_identity = {**source_identity, **{str(k): str(v) for k, v in source_topics.items()}}

    if status_stamp is None or status_stamp < 0 or status_id is None:
        return _invalid_snapshot(
            evaluation_id=evaluation_id,
            evaluation_stamp_ns=max(0, evaluation_stamp_ns),
            state="INVALID_IDENTITY",
            reason="PoseSimulation evaluation identity is unavailable",
            status=status,
        )

    if not bool(_attribute(status, "valid", False)):
        raw_state = str(_attribute(status, "state", "INPUT_INVALID") or "INPUT_INVALID")
        reason = str(_attribute(status, "reason", "Current evaluation is invalid") or "Current evaluation is invalid")
        return _invalid_snapshot(
            evaluation_id=evaluation_id,
            evaluation_stamp_ns=evaluation_stamp_ns,
            state=raw_state,
            reason=reason,
            status=status,
        )

    typed_messages = (pose_error, stereo_geometry)
    if any(message is None for message in typed_messages):
        return _invalid_snapshot(
            evaluation_id=evaluation_id,
            evaluation_stamp_ns=evaluation_stamp_ns,
            state="WAITING_FOR_RESULTS",
            reason="Waiting for aligned PoseSimulation results",
            status=status,
        )
    if any(not _identity_matches(message, evaluation_id, evaluation_stamp_ns) for message in typed_messages):
        return _invalid_snapshot(
            evaluation_id=evaluation_id,
            evaluation_stamp_ns=evaluation_stamp_ns,
            state="EVALUATION_MISMATCH",
            reason="PoseSimulation results belong to different evaluations",
            status=status,
        )
    if not bool(_attribute(pose_error, "valid", False)) or not bool(_attribute(stereo_geometry, "valid", False)):
        return _invalid_snapshot(
            evaluation_id=evaluation_id,
            evaluation_stamp_ns=evaluation_stamp_ns,
            state="INPUT_INVALID",
            reason="PoseSimulation result metrics are invalid",
            status=status,
        )

    pose_inputs = (
        (gt_target, VEHICLE_REFERENCE_FRAME, "target/ground_truth"),
        (actual_left, LEFT_OPTICAL_FRAME, "actual_pose/left"),
        (actual_right, RIGHT_OPTICAL_FRAME, "actual_pose/right"),
        (ideal_gt_rig, RIG_FRAME, "ideal_pose/ground_truth/rig"),
        (ideal_gt_left, LEFT_OPTICAL_FRAME, "ideal_pose/ground_truth/left"),
        (ideal_gt_right, RIGHT_OPTICAL_FRAME, "ideal_pose/ground_truth/right"),
    )
    parsed: list[WorldPose | None] = []
    for message, local_frame, default_topic in pose_inputs:
        parsed.append(
            world_pose_from_pose_stamped(
                message,
                local_frame=local_frame,
                source_topic=(source_topics or {}).get(default_topic, default_topic),
                expected_stamp_ns=evaluation_stamp_ns,
            )
        )
    if any(pose is None for pose in parsed):
        return _invalid_snapshot(
            evaluation_id=evaluation_id,
            evaluation_stamp_ns=evaluation_stamp_ns,
            state="POSE_SOURCE_INVALID",
            reason="A required world-frame pose is missing or invalid",
            status=status,
        )

    left_error = _camera_error(pose_error, "left")
    right_error = _camera_error(pose_error, "right")
    stereo = _stereo_metrics(stereo_geometry)
    if left_error is None or right_error is None or stereo is None:
        return _invalid_snapshot(
            evaluation_id=evaluation_id,
            evaluation_stamp_ns=evaluation_stamp_ns,
            state="METRIC_INVALID",
            reason="PoseSimulation metrics contain a non-finite value",
            status=status,
        )

    rtk_enabled = bool(_attribute(status, "rtk_comparison_implemented", False))
    parsed_rtk: list[WorldPose | None] = [None, None, None, None]
    if rtk_enabled:
        rtk_inputs = (
            (rtk_target, VEHICLE_REFERENCE_FRAME, "target/rtk"),
            (ideal_rtk_rig, RIG_FRAME, "ideal_pose/rtk/rig"),
            (ideal_rtk_left, LEFT_OPTICAL_FRAME, "ideal_pose/rtk/left"),
            (ideal_rtk_right, RIGHT_OPTICAL_FRAME, "ideal_pose/rtk/right"),
        )
        for index, (message, local_frame, default_topic) in enumerate(rtk_inputs):
            parsed_rtk[index] = world_pose_from_pose_stamped(
                message,
                local_frame=local_frame,
                source_topic=(source_topics or {}).get(default_topic, default_topic),
                expected_stamp_ns=evaluation_stamp_ns,
            )
    rtk_available = rtk_enabled and all(pose is not None for pose in parsed_rtk)

    return VisualizationSnapshot(
        evaluation_id=evaluation_id,
        evaluation_stamp_ns=evaluation_stamp_ns,
        world_frame=WORLD_FRAME,
        valid=True,
        state=str(_attribute(status, "state", "VALID") or "VALID"),
        invalid_reason="",
        ground_truth_source=str(_attribute(status, "gt_reference_name", "") or ""),
        gt_reference_name=str(_attribute(status, "gt_reference_name", "") or ""),
        source_identity=source_identity,
        gt_target=parsed[0],
        actual_left=parsed[1],
        actual_right=parsed[2],
        ideal_gt_rig=parsed[3],
        ideal_gt_left=parsed[4],
        ideal_gt_right=parsed[5],
        rtk_target=parsed_rtk[0],
        ideal_rtk_rig=parsed_rtk[1],
        ideal_rtk_left=parsed_rtk[2],
        ideal_rtk_right=parsed_rtk[3],
        left_error=left_error,
        right_error=right_error,
        stereo=stereo,
        rtk_available=rtk_available,
    )


__all__ = [
    "CameraErrorMetrics",
    "LEFT_OPTICAL_FRAME",
    "RIGHT_OPTICAL_FRAME",
    "RIG_FRAME",
    "StereoGeometryMetrics",
    "VEHICLE_REFERENCE_FRAME",
    "VisualizationSnapshot",
    "WORLD_FRAME",
    "WorldPose",
    "build_visualization_snapshot",
    "stamp_ns",
    "world_pose_from_pose_stamped",
]
