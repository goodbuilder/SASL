"""Pure adapters for the external PoseSimulation M2 input contracts."""

from __future__ import annotations

import json
import math
from dataclasses import replace
from typing import Any, Mapping

from .input_types import (
    CAMERA_ENTITY_CHANGED,
    CAMERA_FIELD_MISSING,
    CAMERA_MALFORMED_JSON,
    CAMERA_NONFINITE,
    CAMERA_QUATERNION_INVALID,
    CAMERA_SCHEMA_MISMATCH,
    CAMERA_SDF_HASH_CHANGED,
    CAMERA_SIDE_MISMATCH,
    CAMERA_STATIC_SNAPSHOT_CHANGED,
    CAMERA_TIMESTAMP_INVALID,
    CLOCK_DOMAIN_UNRESOLVED,
    DYNAMIC_STREAM_MODE,
    GAZEBO_SIM_CLOCK,
    GT_DUPLICATE_STAMP,
    GT_FRAME_MISMATCH,
    GT_NONFINITE,
    GT_OUT_OF_ORDER,
    GT_TIMESTAMP_INVALID,
    GroundTruthPoseSample,
    CameraOpticalPoseSample,
    SOURCE_MODE_MISMATCH,
    STATIC_SNAPSHOT_MODE,
    VALID,
    WORLD_FRAME,
)
from .pose_math import (
    GAZEBO_LINK_FLU_FROM_OPTICAL_RDF,
    Matrix3,
    QuaternionXYZW,
    Vector3,
    as_matrix3,
    matrix_multiply,
    matrix_to_quaternion_xyzw,
    quaternion_xyzw_to_matrix,
    validate_rotation_matrix,
)


def _invalid_gt(
    *,
    source_topic: str,
    reference_name: str,
    status: str,
    reason: str,
    stamp_ns: int | None = None,
    frame_id: str = "",
    receive_timestamp_ns: int | None = None,
    clock_domain: str | None = None,
) -> GroundTruthPoseSample:
    return GroundTruthPoseSample(
        stamp_ns=stamp_ns,
        frame_id=frame_id,
        position_world_enu_m=None,
        orientation_world_from_reference_xyzw=None,
        source_topic=source_topic,
        reference_name=reference_name,
        receive_timestamp_ns=receive_timestamp_ns,
        clock_domain=clock_domain,
        valid=False,
        status=status,
        invalid_reason=reason,
    )


def _invalid_camera(
    *,
    camera_side: str,
    source_topic: str,
    source_schema: str,
    source_mode: str,
    status: str,
    reason: str,
    gazebo_stamp_ns: int | None = None,
    receive_timestamp_ns: int | None = None,
    clock_domain: str | None = None,
    entity_name: str = "",
    entity_id: int | None = None,
    generated_sdf_sha256: str = "",
) -> CameraOpticalPoseSample:
    return CameraOpticalPoseSample(
        camera_side=camera_side,
        gazebo_stamp_ns=gazebo_stamp_ns,
        position_world_enu_m=None,
        orientation_world_from_optical_xyzw=None,
        entity_name=entity_name,
        entity_id=entity_id,
        generated_sdf_sha256=generated_sdf_sha256,
        source_topic=source_topic,
        source_schema=source_schema,
        source_mode=source_mode,
        receive_timestamp_ns=receive_timestamp_ns,
        clock_domain=clock_domain,
        valid=False,
        status=status,
        invalid_reason=reason,
    )


def _stamp_ns_from_ros_message(message: Any) -> int:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        raise ValueError("header.stamp is missing")
    raw_seconds = getattr(stamp, "sec")
    raw_nanoseconds = getattr(stamp, "nanosec")
    if not isinstance(raw_seconds, int) or isinstance(raw_seconds, bool):
        raise ValueError("header.stamp.sec must be an integer")
    if not isinstance(raw_nanoseconds, int) or isinstance(raw_nanoseconds, bool):
        raise ValueError("header.stamp.nanosec must be an integer")
    seconds = raw_seconds
    nanoseconds = raw_nanoseconds
    if nanoseconds < 0 or nanoseconds >= 1_000_000_000:
        raise ValueError("header.stamp.nanosec is out of range")
    result = seconds * 1_000_000_000 + nanoseconds
    if result < 0:
        raise ValueError("header.stamp must be non-negative")
    return result


def _finite_vector(value: Any, name: str) -> Vector3:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{name} must contain exactly three values")
    parsed = tuple(float(component) for component in value)
    if not all(math.isfinite(component) for component in parsed):
        raise FloatingPointError(f"{name} contains non-finite data")
    return parsed  # type: ignore[return-value]


def _normalized_quaternion(value: Any, name: str) -> QuaternionXYZW:
    if not isinstance(value, (list, tuple)) or len(value) != 4:
        raise ValueError(f"{name} must contain exactly four values")
    parsed = tuple(float(component) for component in value)
    if not all(math.isfinite(component) for component in parsed):
        raise FloatingPointError(f"{name} contains non-finite data")
    magnitude = math.sqrt(sum(component * component for component in parsed))
    if magnitude <= 0.0 or not math.isfinite(magnitude):
        raise ValueError(f"{name} has zero or invalid norm")
    return tuple(component / magnitude for component in parsed)  # type: ignore[return-value]


def _clock_status(clock_domain: str | None, expected_clock_domain: str) -> str | None:
    if clock_domain is None or str(clock_domain).strip() != expected_clock_domain:
        return CLOCK_DOMAIN_UNRESOLVED
    return None


def adapt_ground_truth_pose(
    message: Any,
    *,
    source_topic: str,
    reference_name: str = "vehicle_reference_link",
    expected_frame: str = WORLD_FRAME,
    receive_timestamp_ns: int | None = None,
    clock_domain: str | None = GAZEBO_SIM_CLOCK,
) -> GroundTruthPoseSample:
    """Adapt one ``PoseStamped`` without using receive time as measurement time."""

    try:
        stamp_ns = _stamp_ns_from_ros_message(message)
    except (AttributeError, TypeError, ValueError, OverflowError) as exc:
        return _invalid_gt(
            source_topic=source_topic,
            reference_name=reference_name,
            status=GT_TIMESTAMP_INVALID,
            reason=str(exc),
            receive_timestamp_ns=receive_timestamp_ns,
            clock_domain=clock_domain,
        )

    header = getattr(message, "header", None)
    frame_id = str(getattr(header, "frame_id", "") or "")
    if frame_id != expected_frame:
        return _invalid_gt(
            source_topic=source_topic,
            reference_name=reference_name,
            status=GT_FRAME_MISMATCH,
            reason=f"expected frame {expected_frame!r}, got {frame_id!r}",
            stamp_ns=stamp_ns,
            frame_id=frame_id,
            receive_timestamp_ns=receive_timestamp_ns,
            clock_domain=clock_domain,
        )
    clock_error = _clock_status(clock_domain, GAZEBO_SIM_CLOCK)
    if clock_error is not None:
        return _invalid_gt(
            source_topic=source_topic,
            reference_name=reference_name,
            status=clock_error,
            reason="Gazebo source clock domain was not established",
            stamp_ns=stamp_ns,
            frame_id=frame_id,
            receive_timestamp_ns=receive_timestamp_ns,
            clock_domain=clock_domain,
        )

    try:
        pose = getattr(message, "pose")
        position = _finite_vector(
            [pose.position.x, pose.position.y, pose.position.z],
            "ground_truth.position_world_enu_m",
        )
        quaternion = _normalized_quaternion(
            [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ],
            "ground_truth.orientation_xyzw",
        )
        # Force a rotation-matrix validation at the adapter boundary and keep
        # a normalized quaternion in the immutable sample.
        validate_rotation_matrix(quaternion_xyzw_to_matrix(quaternion))
    except FloatingPointError as exc:
        return _invalid_gt(
            source_topic=source_topic,
            reference_name=reference_name,
            status=GT_NONFINITE,
            reason=str(exc),
            stamp_ns=stamp_ns,
            frame_id=frame_id,
            receive_timestamp_ns=receive_timestamp_ns,
            clock_domain=clock_domain,
        )
    except (AttributeError, TypeError, ValueError, OverflowError) as exc:
        return _invalid_gt(
            source_topic=source_topic,
            reference_name=reference_name,
            status=GT_NONFINITE,
            reason=str(exc),
            stamp_ns=stamp_ns,
            frame_id=frame_id,
            receive_timestamp_ns=receive_timestamp_ns,
            clock_domain=clock_domain,
        )
    return GroundTruthPoseSample(
        stamp_ns=stamp_ns,
        frame_id=frame_id,
        position_world_enu_m=position,
        orientation_world_from_reference_xyzw=quaternion,
        source_topic=source_topic,
        reference_name=reference_name,
        receive_timestamp_ns=receive_timestamp_ns,
        clock_domain=clock_domain,
        valid=True,
    )


def _payload_value(payload: Mapping[str, Any], key: str) -> Any:
    if key not in payload:
        raise KeyError(key)
    return payload[key]


def adapt_camera_pose_diagnostic(
    payload: Mapping[str, Any] | str,
    *,
    expected_side: str,
    source_topic: str,
    expected_schema: str = "camera_pose_comparison/v1",
    source_mode: str = STATIC_SNAPSHOT_MODE,
    expected_entity_name: str | None = None,
    expected_entity_id: int | None = None,
    expected_sdf_sha256: str | None = None,
    receive_timestamp_ns: int | None = None,
    clock_domain: str | None = GAZEBO_SIM_CLOCK,
    camera_link_rotation_from_optical: Matrix3 = GAZEBO_LINK_FLU_FROM_OPTICAL_RDF,
) -> CameraOpticalPoseSample:
    """Adapt the observed optical pose from ``camera_pose_comparison/v1``.

    Unknown JSON fields are ignored.  All required source fields are checked;
    desired-pose fields are never read as measurements.
    """

    raw_payload = payload
    if isinstance(payload, str):
        try:
            raw_payload = json.loads(payload)
        except (TypeError, json.JSONDecodeError) as exc:
            return _invalid_camera(
                camera_side=expected_side,
                source_topic=source_topic,
                source_schema="",
                source_mode=source_mode,
                status=CAMERA_MALFORMED_JSON,
                reason=str(exc),
                receive_timestamp_ns=receive_timestamp_ns,
                clock_domain=clock_domain,
            )
    if not isinstance(raw_payload, Mapping):
        return _invalid_camera(
            camera_side=expected_side,
            source_topic=source_topic,
            source_schema="",
            source_mode=source_mode,
            status=CAMERA_MALFORMED_JSON,
            reason="camera payload must be a JSON object",
            receive_timestamp_ns=receive_timestamp_ns,
            clock_domain=clock_domain,
        )
    source_schema = str(raw_payload.get("schema", ""))
    camera_label = str(raw_payload.get("camera_label", ""))
    if source_schema != expected_schema:
        return _invalid_camera(
            camera_side=expected_side,
            source_topic=source_topic,
            source_schema=source_schema,
            source_mode=source_mode,
            status=CAMERA_SCHEMA_MISMATCH,
            reason=f"expected schema {expected_schema!r}, got {source_schema!r}",
            receive_timestamp_ns=receive_timestamp_ns,
            clock_domain=clock_domain,
        )
    if camera_label != expected_side:
        return _invalid_camera(
            camera_side=expected_side,
            source_topic=source_topic,
            source_schema=source_schema,
            source_mode=source_mode,
            status=CAMERA_SIDE_MISMATCH,
            reason=f"expected camera side {expected_side!r}, got {camera_label!r}",
            receive_timestamp_ns=receive_timestamp_ns,
            clock_domain=clock_domain,
        )
    if source_mode not in {STATIC_SNAPSHOT_MODE, DYNAMIC_STREAM_MODE}:
        return _invalid_camera(
            camera_side=expected_side,
            source_topic=source_topic,
            source_schema=source_schema,
            source_mode=source_mode,
            status=SOURCE_MODE_MISMATCH,
            reason=f"unsupported camera source mode {source_mode!r}",
            receive_timestamp_ns=receive_timestamp_ns,
            clock_domain=clock_domain,
        )

    entity_name_value = raw_payload.get("observed_optical_entity_name")
    entity_name = entity_name_value if isinstance(entity_name_value, str) else ""
    entity_id_value = raw_payload.get("observed_optical_entity_id")
    generated_hash = raw_payload.get("generated_sdf_sha256")
    try:
        if not entity_name:
            raise KeyError("observed_optical_entity_name")
        if not isinstance(entity_id_value, int) or isinstance(entity_id_value, bool):
            raise KeyError("observed_optical_entity_id")
        if not isinstance(generated_hash, str) or not generated_hash:
            raise KeyError("generated_sdf_sha256")
        if expected_entity_name is not None and entity_name != expected_entity_name:
            return _invalid_camera(
                camera_side=expected_side,
                source_topic=source_topic,
                source_schema=source_schema,
                source_mode=source_mode,
                status=CAMERA_ENTITY_CHANGED,
                reason=f"expected entity {expected_entity_name!r}, got {entity_name!r}",
                receive_timestamp_ns=receive_timestamp_ns,
                clock_domain=clock_domain,
                entity_name=entity_name,
                entity_id=entity_id_value,
                generated_sdf_sha256=generated_hash,
            )
        if expected_entity_id is not None and entity_id_value != expected_entity_id:
            return _invalid_camera(
                camera_side=expected_side,
                source_topic=source_topic,
                source_schema=source_schema,
                source_mode=source_mode,
                status=CAMERA_ENTITY_CHANGED,
                reason=f"expected entity id {expected_entity_id}, got {entity_id_value}",
                receive_timestamp_ns=receive_timestamp_ns,
                clock_domain=clock_domain,
                entity_name=entity_name,
                entity_id=entity_id_value,
                generated_sdf_sha256=generated_hash,
            )
        if expected_sdf_sha256 is not None and generated_hash != expected_sdf_sha256:
            return _invalid_camera(
                camera_side=expected_side,
                source_topic=source_topic,
                source_schema=source_schema,
                source_mode=source_mode,
                status=CAMERA_SDF_HASH_CHANGED,
                reason="generated_sdf_sha256 does not match the expected run identity",
                receive_timestamp_ns=receive_timestamp_ns,
                clock_domain=clock_domain,
                entity_name=entity_name,
                entity_id=entity_id_value,
                generated_sdf_sha256=generated_hash,
            )
        stamp_value = _payload_value(raw_payload, "gazebo_stamp_ns")
        if not isinstance(stamp_value, int) or isinstance(stamp_value, bool) or stamp_value < 0:
            raise OverflowError("gazebo_stamp_ns must be a non-negative integer")
        position = _finite_vector(
            _payload_value(raw_payload, "observed_optical_position_world_enu_m"),
            "observed_optical_position_world_enu_m",
        )
        raw_quaternion = _normalized_quaternion(
            _payload_value(
                raw_payload, "observed_optical_orientation_world_from_entity_xyzw"
            ),
            "observed_optical_orientation_world_from_entity_xyzw",
        )
        fixed_extrinsic = as_matrix3(
            camera_link_rotation_from_optical,
            "camera_link_rotation_from_optical",
        )
        validate_rotation_matrix(fixed_extrinsic)
        world_from_entity = quaternion_xyzw_to_matrix(raw_quaternion)
        world_from_optical = matrix_multiply(world_from_entity, fixed_extrinsic)
        validate_rotation_matrix(world_from_optical)
        optical_quaternion = matrix_to_quaternion_xyzw(world_from_optical)
    except KeyError as exc:
        return _invalid_camera(
            camera_side=expected_side,
            source_topic=source_topic,
            source_schema=source_schema,
            source_mode=source_mode,
            status=CAMERA_FIELD_MISSING,
            reason=f"missing required field {exc.args[0]!r}",
            receive_timestamp_ns=receive_timestamp_ns,
            clock_domain=clock_domain,
            entity_name=entity_name,
            entity_id=entity_id_value if isinstance(entity_id_value, int) else None,
            generated_sdf_sha256=generated_hash if isinstance(generated_hash, str) else "",
        )
    except FloatingPointError as exc:
        return _invalid_camera(
            camera_side=expected_side,
            source_topic=source_topic,
            source_schema=source_schema,
            source_mode=source_mode,
            status=CAMERA_NONFINITE,
            reason=str(exc),
            receive_timestamp_ns=receive_timestamp_ns,
            clock_domain=clock_domain,
            entity_name=entity_name,
            entity_id=entity_id_value if isinstance(entity_id_value, int) else None,
            generated_sdf_sha256=generated_hash if isinstance(generated_hash, str) else "",
        )
    except OverflowError as exc:
        return _invalid_camera(
            camera_side=expected_side,
            source_topic=source_topic,
            source_schema=source_schema,
            source_mode=source_mode,
            status=CAMERA_TIMESTAMP_INVALID,
            reason=str(exc),
            receive_timestamp_ns=receive_timestamp_ns,
            clock_domain=clock_domain,
            entity_name=entity_name,
            entity_id=entity_id_value if isinstance(entity_id_value, int) else None,
            generated_sdf_sha256=generated_hash if isinstance(generated_hash, str) else "",
        )
    except (TypeError, ValueError) as exc:
        status = CAMERA_QUATERNION_INVALID if "orientation" in str(exc) or "quaternion" in str(exc) or "norm" in str(exc) else CAMERA_NONFINITE
        return _invalid_camera(
            camera_side=expected_side,
            source_topic=source_topic,
            source_schema=source_schema,
            source_mode=source_mode,
            status=status,
            reason=str(exc),
            receive_timestamp_ns=receive_timestamp_ns,
            clock_domain=clock_domain,
            entity_name=entity_name,
            entity_id=entity_id_value if isinstance(entity_id_value, int) else None,
            generated_sdf_sha256=generated_hash if isinstance(generated_hash, str) else "",
        )

    clock_error = _clock_status(clock_domain, GAZEBO_SIM_CLOCK)
    if clock_error is not None:
        return _invalid_camera(
            camera_side=expected_side,
            source_topic=source_topic,
            source_schema=source_schema,
            source_mode=source_mode,
            status=clock_error,
            reason="Gazebo source clock domain was not established",
            gazebo_stamp_ns=stamp_value,
            receive_timestamp_ns=receive_timestamp_ns,
            clock_domain=clock_domain,
            entity_name=entity_name,
            entity_id=entity_id_value,
            generated_sdf_sha256=generated_hash,
        )
    return CameraOpticalPoseSample(
        camera_side=expected_side,
        gazebo_stamp_ns=stamp_value,
        position_world_enu_m=position,
        orientation_world_from_optical_xyzw=optical_quaternion,
        entity_name=entity_name,
        entity_id=entity_id_value,
        generated_sdf_sha256=generated_hash,
        source_topic=source_topic,
        source_schema=source_schema,
        source_mode=source_mode,
        receive_timestamp_ns=receive_timestamp_ns,
        clock_domain=clock_domain,
        valid=True,
    )


def _camera_identity(sample: CameraOpticalPoseSample) -> tuple[str, int | None, str]:
    return sample.entity_name, sample.entity_id, sample.generated_sdf_sha256


def _camera_pose_equal(left: CameraOpticalPoseSample, right: CameraOpticalPoseSample) -> bool:
    return (
        left.position_world_enu_m == right.position_world_enu_m
        and left.rotation_world_from_optical == right.rotation_world_from_optical
        and left.source_schema == right.source_schema
        and left.source_mode == right.source_mode
    )


class StaticCameraSnapshotStore:
    """Keep the first valid static camera pose immutable for one run."""

    def __init__(self, camera_side: str) -> None:
        self.camera_side = str(camera_side)
        self.snapshot: CameraOpticalPoseSample | None = None
        self.closed_status: str | None = None
        self.closed_reason: str = ""

    def accept(self, sample: CameraOpticalPoseSample) -> CameraOpticalPoseSample:
        if not sample.valid:
            return sample
        if sample.camera_side != self.camera_side:
            return replace(
                sample,
                valid=False,
                status=CAMERA_SIDE_MISMATCH,
                invalid_reason=f"expected {self.camera_side!r}, got {sample.camera_side!r}",
            )
        if sample.source_mode != STATIC_SNAPSHOT_MODE:
            return replace(
                sample,
                valid=False,
                status=SOURCE_MODE_MISMATCH,
                invalid_reason="static snapshot store received a non-static sample",
            )
        if self.closed_status is not None:
            return replace(
                sample,
                valid=False,
                status=self.closed_status,
                invalid_reason=self.closed_reason,
            )
        if self.snapshot is None:
            self.snapshot = sample
            return sample
        if _camera_identity(sample) != _camera_identity(self.snapshot):
            if sample.generated_sdf_sha256 != self.snapshot.generated_sdf_sha256:
                status = CAMERA_SDF_HASH_CHANGED
                reason = "static camera generated SDF identity changed"
            else:
                status = CAMERA_ENTITY_CHANGED
                reason = "static camera entity identity changed"
            self.closed_status, self.closed_reason = status, reason
            return replace(sample, valid=False, status=status, invalid_reason=reason)
        if not _camera_pose_equal(sample, self.snapshot):
            status = CAMERA_STATIC_SNAPSHOT_CHANGED
            reason = "static camera pose changed within one run"
            self.closed_status, self.closed_reason = status, reason
            return replace(sample, valid=False, status=status, invalid_reason=reason)
        # Return the original immutable sample, including its first source
        # timestamp.  A repeated transient-local message is not a new measure.
        return self.snapshot


class GroundTruthStreamValidator:
    """Reject GT time regressions and configured duplicate stamps."""

    def __init__(
        self,
        *,
        reject_duplicate_stamps: bool = True,
        reject_out_of_order: bool = True,
    ) -> None:
        self.reject_duplicate_stamps = bool(reject_duplicate_stamps)
        self.reject_out_of_order = bool(reject_out_of_order)
        self.last_valid: GroundTruthPoseSample | None = None

    def accept(self, sample: GroundTruthPoseSample) -> GroundTruthPoseSample:
        if not sample.valid or sample.stamp_ns is None:
            return sample
        if self.last_valid is not None and self.last_valid.stamp_ns is not None:
            if sample.stamp_ns < self.last_valid.stamp_ns and self.reject_out_of_order:
                return replace(
                    sample,
                    valid=False,
                    status=GT_OUT_OF_ORDER,
                    invalid_reason=(
                        f"GT stamp moved backwards from {self.last_valid.stamp_ns} to {sample.stamp_ns}"
                    ),
                )
            if sample.stamp_ns == self.last_valid.stamp_ns and self.reject_duplicate_stamps:
                return replace(
                    sample,
                    valid=False,
                    status=GT_DUPLICATE_STAMP,
                    invalid_reason="duplicate GT source timestamp is rejected",
                )
        self.last_valid = sample
        return sample


__all__ = [
    "GroundTruthStreamValidator",
    "StaticCameraSnapshotStore",
    "adapt_camera_pose_diagnostic",
    "adapt_ground_truth_pose",
]
