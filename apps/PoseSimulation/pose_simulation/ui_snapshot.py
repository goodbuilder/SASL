"""Immutable UI-facing snapshots for the diagnostic-only PoseSimulation app.

The ROS adapters publish JSON and ``PoseStamped`` messages independently.  This
module is the boundary that combines them for display.  It deliberately does
not recompute pose metrics: values are copied from PoseSimulation's existing
evaluation output and are marked unavailable when the evaluation identity,
frame contract, or source validity cannot be verified.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from types import MappingProxyType
from typing import Any, Mapping, Sequence


WORLD_FRAME = "gazebo_world_enu"
RIG_FRAME = "baseline_center_frd"
OPTICAL_FRAME = "camera_optical_rdf"
ANGLE_UNIT = "deg"


def _freeze(value: Any) -> Any:
    """Recursively freeze a JSON-compatible value for cross-thread delivery."""

    if isinstance(value, Mapping):
        return MappingProxyType({str(key): _freeze(item) for key, item in value.items()})
    if isinstance(value, (list, tuple)):
        return tuple(_freeze(item) for item in value)
    return value


def _finite_tuple(value: Any, size: int, name: str) -> tuple[float, ...] | None:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != size:
        return None
    try:
        parsed = tuple(float(component) for component in value)
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(component) for component in parsed):
        return None
    return parsed


def _integer(value: Any) -> int | None:
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        return None
    return parsed


def _evaluation_id(payload: Mapping[str, Any] | None) -> str | None:
    if not isinstance(payload, Mapping):
        return None
    timestamp = _integer(payload.get("evaluation_timestamp_ns"))
    if timestamp is None or timestamp < 0:
        return None
    # The Phase 1 core has no separate sequence ID.  The core evaluation
    # timestamp is therefore the only safe identity available to UI adapters.
    return f"evaluation:{timestamp}"


def _mapping(payload: Any) -> Mapping[str, Any] | None:
    return payload if isinstance(payload, Mapping) else None


def _path(payload: Mapping[str, Any] | None, *keys: str) -> Any:
    current: Any = payload
    for key in keys:
        if not isinstance(current, Mapping):
            return None
        current = current.get(key)
    return current


@dataclass(frozen=True)
class PoseRecord:
    """A validated world pose with the represented local frame identity.

    ``position_world_enu_m`` and ``orientation_xyzw`` are expressed in the
    Gazebo world, while ``frame_id`` identifies the local frame whose world
    pose is represented (for example ``camera_optical_rdf``).
    """

    frame_id: str
    position_world_enu_m: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    source_timestamp_ns: int
    source: str


def pose_record_from_mapping(
    payload: Any,
    *,
    source: str,
    default_timestamp_ns: int | None = None,
    expected_frame_id: str | None = None,
) -> PoseRecord | None:
    """Convert an ideal-pose JSON mapping into a validated pose record.

    PoseSimulation's ideal-pose JSON stores a world pose of a local frame, so
    ``frame_id`` identifies the represented local frame rather than the
    coordinate frame in which the position is expressed.  The caller supplies
    the expected local frame from the coordinate contract.
    """

    if not isinstance(payload, Mapping) or not bool(payload.get("valid", False)):
        return None
    frame_id = str(payload.get("frame_id", "") or "")
    position = _finite_tuple(payload.get("position_world_enu_m"), 3, "position")
    orientation = _finite_tuple(payload.get("orientation_xyzw"), 4, "orientation")
    timestamp = _integer(payload.get("source_timestamp_ns"))
    if timestamp is None:
        timestamp = default_timestamp_ns
    if not frame_id or (expected_frame_id is not None and frame_id != expected_frame_id):
        return None
    if position is None or orientation is None or timestamp is None:
        return None
    norm = math.sqrt(sum(component * component for component in orientation))
    if norm <= 1.0e-12:
        return None
    normalized = tuple(component / norm for component in orientation)
    return PoseRecord(frame_id, position, normalized, timestamp, source)


@dataclass(frozen=True)
class SnapshotState:
    """Text-friendly source states shared by rqt and the diagnostics adapter."""

    application: str
    ground_truth: str
    rtk: str
    left_optical: str
    right_optical: str
    time_alignment: str
    frame_contract: str
    marker_alignment: str


@dataclass(frozen=True)
class PoseSimulationSnapshot:
    """Immutable, already-paired data for one UI evaluation sample."""

    evaluation_id: str | None
    evaluation_timestamp_ns: int | None
    pose_error: Mapping[str, Any]
    stereo_geometry: Mapping[str, Any]
    status: Mapping[str, Any]
    poses: Mapping[str, PoseRecord]
    target_gt_world_enu_m: tuple[float, float, float] | None
    target_rtk_world_enu_m: tuple[float, float, float] | None
    state: SnapshotState
    pair_state: str
    reason: str
    receive_timestamp_ns: int | None

    @property
    def valid(self) -> bool:
        return self.pair_state == "ALIGNED"

    @property
    def validity(self) -> Mapping[str, Any]:
        value = self.status.get("validity", {})
        return value if isinstance(value, Mapping) else MappingProxyType({})

    def get(self, *keys: str, default: Any = None) -> Any:
        """Read a nested value from the combined snapshot payloads."""

        for payload in (self.pose_error, self.stereo_geometry, self.status):
            value = _path(payload, *keys)
            if value is not None:
                return value
        return default


def _empty_snapshot(*, reason: str, now_receive_ns: int | None) -> PoseSimulationSnapshot:
    empty = MappingProxyType({})
    return PoseSimulationSnapshot(
        evaluation_id=None,
        evaluation_timestamp_ns=None,
        pose_error=empty,
        stereo_geometry=empty,
        status=empty,
        poses=MappingProxyType({}),
        target_gt_world_enu_m=None,
        target_rtk_world_enu_m=None,
        state=SnapshotState(
            "UNAVAILABLE",
            "UNAVAILABLE",
            "UNAVAILABLE",
            "UNAVAILABLE",
            "UNAVAILABLE",
            "UNAVAILABLE",
            "UNAVAILABLE",
            "UNALIGNED SAMPLE",
        ),
        pair_state="UNAVAILABLE",
        reason=reason,
        receive_timestamp_ns=now_receive_ns,
    )


def _state_for_source(
    *,
    available: bool,
    valid: bool,
    fallback: bool = False,
    status: str = "",
) -> str:
    if not available:
        return "UNAVAILABLE"
    if status.upper() in {"STALE", "○ STALE"}:
        return "STALE"
    if not valid:
        return "INVALID"
    return "WARNING" if fallback else "VALID"


def _time_state(status: Mapping[str, Any]) -> str:
    residuals = status.get("time_residuals", {})
    if not isinstance(residuals, Mapping):
        return "UNAVAILABLE"
    alignment = str(residuals.get("alignment_status", "") or "")
    if alignment == "VALID":
        return "VALID"
    if alignment in {"TIME_ALIGNMENT_FAILED", "FAILED", "OUT_OF_ORDER"}:
        return "INVALID"
    return "UNAVAILABLE"


def _frame_state(
    pose_error: Mapping[str, Any],
    stereo_geometry: Mapping[str, Any],
    status: Mapping[str, Any],
    poses: Mapping[str, PoseRecord],
) -> str:
    frame_values = [pose_error.get("frame_id"), stereo_geometry.get("frame_id")]
    coordinate_frames = status.get("coordinate_frames", {})
    if isinstance(coordinate_frames, Mapping):
        frame_values.append(coordinate_frames.get("world"))
    if any(value not in (None, WORLD_FRAME) for value in frame_values):
        return "INVALID"
    if not isinstance(coordinate_frames, Mapping):
        return "UNAVAILABLE"
    expected_world = str(coordinate_frames.get("world", WORLD_FRAME) or WORLD_FRAME)
    expected_rig = str(coordinate_frames.get("rig", RIG_FRAME) or RIG_FRAME)
    expected_optical = str(coordinate_frames.get("camera_optical", OPTICAL_FRAME) or OPTICAL_FRAME)
    for name, record in poses.items():
        if name.startswith("actual_"):
            expected = expected_world
        elif name.endswith("_rig"):
            expected = expected_rig
        elif name.endswith("_left") or name.endswith("_right"):
            expected = expected_optical
        else:
            return "INVALID"
        if record.frame_id != expected:
            return "INVALID"
    if not frame_values or all(value is None for value in frame_values):
        return "UNAVAILABLE"
    return "VALID"


def _marker_alignment_state(
    status: Mapping[str, Any],
    poses: Mapping[str, PoseRecord],
    frame_state: str,
) -> str:
    if frame_state != "VALID":
        return "FRAME MISMATCH"
    expected = {
        "actual_left": status.get("left_camera_pose_timestamp_ns"),
        "actual_right": status.get("right_camera_pose_timestamp_ns"),
    }
    for name, timestamp in expected.items():
        parsed = _integer(timestamp)
        record = poses.get(name)
        if parsed is None or record is None:
            return "UNALIGNED SAMPLE"
        if record.source_timestamp_ns != parsed:
            return "TIMESTAMP MISMATCH"
    return "ALIGNED"


def build_ui_snapshot(
    pose_error: Mapping[str, Any] | None,
    stereo_geometry: Mapping[str, Any] | None,
    status: Mapping[str, Any] | None,
    *,
    actual_poses: Mapping[str, PoseRecord] | None = None,
    now_receive_ns: int | None = None,
) -> PoseSimulationSnapshot:
    """Build one immutable snapshot without recomputing any diagnostic metric."""

    pose_error_map = _mapping(pose_error)
    stereo_map = _mapping(stereo_geometry)
    status_map = _mapping(status)
    if pose_error_map is None or stereo_map is None or status_map is None:
        return _empty_snapshot(reason="WAITING FOR POSE_ERROR, STEREO_GEOMETRY, AND STATUS", now_receive_ns=now_receive_ns)

    ids = (_evaluation_id(pose_error_map), _evaluation_id(stereo_map), _evaluation_id(status_map))
    if any(value is None for value in ids):
        return _empty_snapshot(reason="MISSING EVALUATION ID", now_receive_ns=now_receive_ns)
    if len(set(ids)) != 1:
        frozen_error = _freeze(pose_error_map)
        frozen_stereo = _freeze(stereo_map)
        frozen_status = _freeze(status_map)
        return PoseSimulationSnapshot(
            evaluation_id=ids[2],
            evaluation_timestamp_ns=_integer(status_map.get("evaluation_timestamp_ns")),
            pose_error=frozen_error,
            stereo_geometry=frozen_stereo,
            status=frozen_status,
            poses=MappingProxyType({}),
            target_gt_world_enu_m=None,
            target_rtk_world_enu_m=None,
            state=SnapshotState(
                "UNALIGNED",
                "UNALIGNED",
                "UNALIGNED",
                "UNALIGNED",
                "UNALIGNED",
                "UNALIGNED",
                "UNALIGNED",
                "PAIR ID MISMATCH",
            ),
            pair_state="PAIR ID MISMATCH",
            reason="POSE_ERROR, STEREO_GEOMETRY, AND STATUS HAVE DIFFERENT EVALUATION TIMESTAMPS",
            receive_timestamp_ns=now_receive_ns,
        )

    records: dict[str, PoseRecord] = {}
    coordinate_frames = status_map.get("coordinate_frames", {})
    if not isinstance(coordinate_frames, Mapping):
        coordinate_frames = {}
    optical_frame = str(coordinate_frames.get("camera_optical", OPTICAL_FRAME) or OPTICAL_FRAME)
    rig_frame = str(coordinate_frames.get("rig", RIG_FRAME) or RIG_FRAME)
    ideal_root = _path(pose_error_map, "ideal_pose")
    if isinstance(ideal_root, Mapping):
        for source_key, source_name in (("ground_truth", "ideal_gt"), ("rtk", "ideal_rtk")):
            source_root = ideal_root.get(source_key)
            if not isinstance(source_root, Mapping):
                continue
            for side_key, record_key in (("left_optical", "left"), ("right_optical", "right"), ("rig", "rig")):
                record = pose_record_from_mapping(
                    source_root.get(side_key),
                    source=f"{source_name}_{record_key}",
                    expected_frame_id=rig_frame if side_key == "rig" else optical_frame,
                    default_timestamp_ns=(
                        _integer(status_map.get("evaluation_timestamp_ns"))
                        if source_key == "rtk"
                        else None
                    ),
                )
                if record is not None:
                    records[f"{source_name}_{record_key}"] = record
    if actual_poses:
        records.update({str(key): value for key, value in actual_poses.items() if value is not None})

    frozen_error = _freeze(pose_error_map)
    frozen_stereo = _freeze(stereo_map)
    frozen_status = _freeze(status_map)
    frozen_records = MappingProxyType(dict(records))
    validity = status_map.get("validity", {})
    validity = validity if isinstance(validity, Mapping) else {}
    gt_source = str(status_map.get("ground_truth_source", "") or "")
    rtk_status = str(status_map.get("rtk_status", "") or "")
    gt_state = _state_for_source(
        available=True,
        valid=bool(validity.get("ground_truth_ideal", False)),
        fallback=bool(status_map.get("fallback_used", False)),
        status=gt_source,
    )
    rtk_state = _state_for_source(
        available=True,
        valid=bool(status_map.get("rtk_reference_valid_for_operational_evaluation", False))
        and bool(validity.get("rtk_ideal", False)),
        status=rtk_status,
    )
    frame_state = _frame_state(pose_error_map, stereo_map, status_map, records)
    marker_state = _marker_alignment_state(status_map, records, frame_state)
    required_pose_inputs = bool(validity.get("required_pose_inputs", False))
    left_state = "VALID" if required_pose_inputs else "INVALID"
    right_state = "VALID" if required_pose_inputs else "INVALID"
    if actual_poses is not None:
        left_state = "VALID" if "actual_left" in records else "UNAVAILABLE"
        right_state = "VALID" if "actual_right" in records else "UNAVAILABLE"
    time_state = _time_state(status_map)
    application_state = "VALID" if ids[0] is not None and frame_state == "VALID" else "WARNING"
    if marker_state != "ALIGNED" and actual_poses is not None:
        application_state = "WARNING"
    pair_state = "ALIGNED" if ids[0] is not None and frame_state == "VALID" else "UNALIGNED"
    if actual_poses is not None and marker_state != "ALIGNED":
        pair_state = marker_state
    gt_target = _finite_tuple(_path(pose_error_map, "ideal_pose", "ground_truth", "target_world_enu_m"), 3, "gt target")
    rtk_target = _finite_tuple(_path(pose_error_map, "ideal_pose", "rtk", "target_world_enu_m"), 3, "rtk target")
    return PoseSimulationSnapshot(
        evaluation_id=ids[0],
        evaluation_timestamp_ns=_integer(status_map.get("evaluation_timestamp_ns")),
        pose_error=frozen_error,
        stereo_geometry=frozen_stereo,
        status=frozen_status,
        poses=frozen_records,
        target_gt_world_enu_m=gt_target,
        target_rtk_world_enu_m=rtk_target if rtk_state == "VALID" else None,
        state=SnapshotState(
            application_state,
            gt_state,
            rtk_state,
            left_state,
            right_state,
            time_state,
            frame_state,
            marker_state,
        ),
        pair_state=pair_state,
        reason="" if pair_state == "ALIGNED" else marker_state,
        receive_timestamp_ns=now_receive_ns,
    )


__all__ = [
    "ANGLE_UNIT",
    "PoseRecord",
    "PoseSimulationSnapshot",
    "OPTICAL_FRAME",
    "RIG_FRAME",
    "SnapshotState",
    "WORLD_FRAME",
    "build_ui_snapshot",
    "pose_record_from_mapping",
]
