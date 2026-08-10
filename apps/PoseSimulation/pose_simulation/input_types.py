"""Immutable external-pose input contracts for PoseSimulation M2.

These types sit between ROS/source adapters and the M1 mathematics.  They
carry source provenance and validity explicitly so an invalid or stale input
cannot be mistaken for the last valid sample.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

from .pose_math import Matrix3, QuaternionXYZW, Vector3


WORLD_FRAME = "gazebo_world_enu"
CAMERA_OPTICAL_FRAME = "camera_optical_rdf"
GAZEBO_SIM_CLOCK = "gazebo_sim_time"
STATIC_SNAPSHOT_MODE = "static_snapshot"
DYNAMIC_STREAM_MODE = "dynamic_stream"

# Input-source and alignment states are strings intentionally: they can be
# emitted unchanged by a probe or a future diagnostics message.
VALID = "VALID"
WAITING_FOR_GT = "WAITING_FOR_GT"
WAITING_FOR_LEFT_CAMERA = "WAITING_FOR_LEFT_CAMERA"
WAITING_FOR_RIGHT_CAMERA = "WAITING_FOR_RIGHT_CAMERA"
GT_FRAME_MISMATCH = "GT_FRAME_MISMATCH"
GT_NONFINITE = "GT_NONFINITE"
GT_TIMESTAMP_INVALID = "GT_TIMESTAMP_INVALID"
GT_OUT_OF_ORDER = "GT_OUT_OF_ORDER"
GT_DUPLICATE_STAMP = "GT_DUPLICATE_STAMP"
GT_SOURCE_STALE = "GT_SOURCE_STALE"
CAMERA_SCHEMA_MISMATCH = "CAMERA_SCHEMA_MISMATCH"
CAMERA_SIDE_MISMATCH = "CAMERA_SIDE_MISMATCH"
CAMERA_NONFINITE = "CAMERA_NONFINITE"
CAMERA_QUATERNION_INVALID = "CAMERA_QUATERNION_INVALID"
CAMERA_FIELD_MISSING = "CAMERA_FIELD_MISSING"
CAMERA_TIMESTAMP_INVALID = "CAMERA_TIMESTAMP_INVALID"
CAMERA_MALFORMED_JSON = "CAMERA_MALFORMED_JSON"
CAMERA_ENTITY_CHANGED = "CAMERA_ENTITY_CHANGED"
CAMERA_SDF_HASH_CHANGED = "CAMERA_SDF_HASH_CHANGED"
CAMERA_STATIC_SNAPSHOT_CHANGED = "CAMERA_STATIC_SNAPSHOT_CHANGED"
CLOCK_DOMAIN_UNRESOLVED = "CLOCK_DOMAIN_UNRESOLVED"
TIME_ALIGNMENT_FAILED = "TIME_ALIGNMENT_FAILED"
SOURCE_MODE_MISMATCH = "SOURCE_MODE_MISMATCH"
SOURCE_UNAVAILABLE = "SOURCE_UNAVAILABLE"
QOS_INCOMPATIBLE = "QOS_INCOMPATIBLE"


@dataclass(frozen=True)
class GroundTruthPoseSample:
    """One validated ``geometry_msgs/msg/PoseStamped`` in World ENU.

    ``stamp_ns`` is the source ``header.stamp`` in nanoseconds.  Position is
    metres and the quaternion is ROS order ``(x, y, z, w)`` representing
    ``R_world_from_reference``.  Receive time is monotonic bookkeeping only.
    """

    stamp_ns: int | None
    frame_id: str
    position_world_enu_m: Vector3 | None
    orientation_world_from_reference_xyzw: QuaternionXYZW | None
    source_topic: str
    reference_name: str
    receive_timestamp_ns: int | None
    clock_domain: str | None
    valid: bool
    status: str = VALID
    invalid_reason: str = ""


@dataclass(frozen=True)
class CameraOpticalPoseSample:
    """One validated observed camera optical pose in World ENU.

    The source JSON orientation is an entity orientation.  The adapter applies
    the verified entity-to-optical fixed rotation before storing the resulting
    ``R_world_from_optical`` quaternion here.
    """

    camera_side: Literal["left", "right"] | str
    gazebo_stamp_ns: int | None
    position_world_enu_m: Vector3 | None
    orientation_world_from_optical_xyzw: QuaternionXYZW | None
    entity_name: str
    entity_id: int | None
    generated_sdf_sha256: str
    source_topic: str
    source_schema: str
    source_mode: str
    receive_timestamp_ns: int | None
    clock_domain: str | None
    valid: bool
    status: str = VALID
    invalid_reason: str = ""

    @property
    def rotation_world_from_optical(self) -> Matrix3 | None:
        """Return the matrix only when the sample has a valid quaternion."""

        if self.orientation_world_from_optical_xyzw is None:
            return None
        from .pose_math import quaternion_xyzw_to_matrix

        return quaternion_xyzw_to_matrix(self.orientation_world_from_optical_xyzw)


@dataclass(frozen=True)
class AlignedPoseInputs:
    """One input-only evaluation sample after time/source validation.

    No ideal pose or attitude error is stored here.  Static camera residuals
    are ``None`` because a validated immutable run snapshot is intentionally
    not treated as a time-varying measurement.
    """

    evaluation_id: str | None
    evaluation_stamp_ns: int | None
    ground_truth: GroundTruthPoseSample | None
    left_actual_camera: CameraOpticalPoseSample | None
    right_actual_camera: CameraOpticalPoseSample | None
    alignment_method: str
    gt_time_residual_ns: int | None
    left_time_residual_ns: int | None
    right_time_residual_ns: int | None
    clock_domain: str | None
    valid: bool
    status: str
    invalid_reason: str = ""


__all__ = [
    "AlignedPoseInputs",
    "CAMERA_ENTITY_CHANGED",
    "CAMERA_FIELD_MISSING",
    "CAMERA_MALFORMED_JSON",
    "CAMERA_NONFINITE",
    "CAMERA_QUATERNION_INVALID",
    "CAMERA_SCHEMA_MISMATCH",
    "CAMERA_SDF_HASH_CHANGED",
    "CAMERA_SIDE_MISMATCH",
    "CAMERA_STATIC_SNAPSHOT_CHANGED",
    "CAMERA_TIMESTAMP_INVALID",
    "CAMERA_OPTICAL_FRAME",
    "CLOCK_DOMAIN_UNRESOLVED",
    "DYNAMIC_STREAM_MODE",
    "GAZEBO_SIM_CLOCK",
    "GT_DUPLICATE_STAMP",
    "GT_FRAME_MISMATCH",
    "GT_NONFINITE",
    "GT_OUT_OF_ORDER",
    "GT_SOURCE_STALE",
    "GT_TIMESTAMP_INVALID",
    "GroundTruthPoseSample",
    "CameraOpticalPoseSample",
    "SOURCE_MODE_MISMATCH",
    "SOURCE_UNAVAILABLE",
    "QOS_INCOMPATIBLE",
    "STATIC_SNAPSHOT_MODE",
    "TIME_ALIGNMENT_FAILED",
    "VALID",
    "WAITING_FOR_GT",
    "WAITING_FOR_LEFT_CAMERA",
    "WAITING_FOR_RIGHT_CAMERA",
    "WORLD_FRAME",
]
