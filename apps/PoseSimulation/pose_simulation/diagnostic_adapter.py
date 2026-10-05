"""Pure mapping from a PoseSimulation UI snapshot to ROS diagnostics."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

from .ui_snapshot import PoseSimulationSnapshot


DIAGNOSTIC_OK = 0
DIAGNOSTIC_WARN = 1
DIAGNOSTIC_ERROR = 2
DIAGNOSTIC_STALE = 3


@dataclass(frozen=True)
class DiagnosticRecord:
    name: str
    level: int
    message: str
    values: tuple[tuple[str, str], ...]


def _level(state: str) -> int:
    value = str(state).upper()
    if value == "VALID":
        return DIAGNOSTIC_OK
    if value == "STALE":
        return DIAGNOSTIC_STALE
    if value in {"WARNING", "DEGRADED", "UNALIGNED", "UNALIGNED SAMPLE", "PAIR ID MISMATCH", "TIMESTAMP MISMATCH"}:
        return DIAGNOSTIC_WARN
    return DIAGNOSTIC_ERROR


def _values(snapshot: PoseSimulationSnapshot, **extra: Any) -> tuple[tuple[str, str], ...]:
    values = {
        "evaluation_id": snapshot.evaluation_id or "",
        "evaluation_timestamp_ns": "" if snapshot.evaluation_timestamp_ns is None else str(snapshot.evaluation_timestamp_ns),
        "frame_id": str(snapshot.status.get("coordinate_frames", {}).get("world", ""))
        if isinstance(snapshot.status.get("coordinate_frames", {}), Mapping)
        else "gazebo_world_enu",
    }
    values.update({str(key): "" if value is None else str(value) for key, value in extra.items()})
    return tuple(sorted(values.items()))


def build_diagnostic_records(snapshot: PoseSimulationSnapshot) -> tuple[DiagnosticRecord, ...]:
    """Return stable DiagnosticStatus-like records for one UI sample."""

    state = snapshot.state
    status = snapshot.status
    validity = snapshot.validity
    records = [
        DiagnosticRecord(
            "apps/PoseSimulation/Inputs/GroundTruth",
            _level(state.ground_truth),
            state.ground_truth if state.ground_truth != "WARNING" else "WARNING — FALLBACK GROUND-TRUTH SOURCE",
            _values(
                snapshot,
                ground_truth_source=status.get("ground_truth_source"),
                target_entity_name=status.get("target_entity_name"),
                fallback_used=status.get("fallback_used"),
            ),
        ),
        DiagnosticRecord(
            "apps/PoseSimulation/Inputs/RTK",
            _level(state.rtk),
            f"RTK Operational Reference: {state.rtk}",
            _values(
                snapshot,
                rtk_status=status.get("rtk_status"),
                operational_valid=status.get("rtk_reference_valid_for_operational_evaluation"),
                source_topic=status.get("rtk_source_topic"),
            ),
        ),
        DiagnosticRecord(
            "apps/PoseSimulation/Inputs/LeftOpticalPose",
            _level(state.left_optical),
            state.left_optical,
            _values(snapshot, timestamp_ns=status.get("left_camera_pose_timestamp_ns")),
        ),
        DiagnosticRecord(
            "apps/PoseSimulation/Inputs/RightOpticalPose",
            _level(state.right_optical),
            state.right_optical,
            _values(snapshot, timestamp_ns=status.get("right_camera_pose_timestamp_ns")),
        ),
        DiagnosticRecord(
            "apps/PoseSimulation/TimeAlignment",
            _level(state.time_alignment),
            state.time_alignment,
            _values(snapshot, **(status.get("time_residuals", {}) if isinstance(status.get("time_residuals", {}), Mapping) else {})),
        ),
        DiagnosticRecord(
            "apps/PoseSimulation/FrameContract",
            _level(state.frame_contract),
            state.frame_contract,
            _values(snapshot, coordinate_frames=status.get("coordinate_frames")),
        ),
        DiagnosticRecord(
            "apps/PoseSimulation/PoseError",
            DIAGNOSTIC_OK if bool(validity.get("actual_vs_gt", False)) else DIAGNOSTIC_ERROR,
            "VALID" if bool(validity.get("actual_vs_gt", False)) else "INVALID",
            _values(snapshot, actual_vs_gt=validity.get("actual_vs_gt"), actual_vs_rtk=validity.get("actual_vs_rtk")),
        ),
        DiagnosticRecord(
            "apps/PoseSimulation/StereoGeometry",
            DIAGNOSTIC_OK if bool(validity.get("stereo_relative", False)) and bool(validity.get("baseline", False)) else DIAGNOSTIC_ERROR,
            "VALID" if bool(validity.get("stereo_relative", False)) and bool(validity.get("baseline", False)) else "INVALID",
            _values(snapshot, stereo_relative=validity.get("stereo_relative"), baseline=validity.get("baseline")),
        ),
        DiagnosticRecord(
            "apps/PoseSimulation/WriteGuard",
            DIAGNOSTIC_OK,
            "OK — DIAGNOSTIC_ONLY — POSE_COMMAND_DISABLED",
            _values(snapshot, pose_write="DISABLED", set_pose="NOT_CALLED", correction="NOT_APPLIED"),
        ),
    ]
    return tuple(records)


__all__ = [
    "DIAGNOSTIC_ERROR",
    "DIAGNOSTIC_OK",
    "DIAGNOSTIC_STALE",
    "DIAGNOSTIC_WARN",
    "DiagnosticRecord",
    "build_diagnostic_records",
]
